#include <chrono>
#include <digital_twin_msgs/msg/supply_input.hpp>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "parse_dewetron.h"
#include "rclcpp/rclcpp.hpp"

/*  A Node which simulates the flow of current and voltage.
 *  In the initialization, it reads the supplied file from the parameters
 *  and processes the data from the file in the ParseDewetron. Data is then played
 *  back (streamed) to ROS topics at a specified frequency (also specified in params).
 */
class InputCurrentVoltage : public rclcpp::Node {
 public:
  InputCurrentVoltage() : Node("current_simulator") {
    // initialize parameters
    init_ros_params();
    this->run_forever_ = run_forever_param_.as_bool();

    // initializing publishers/subscribers
    supply_input_publisher_ =
        this->create_publisher<digital_twin_msgs::msg::SupplyInput>("supply_input", 10);

    // cast double Hz to duration in secs of type double
    auto interval_sec =
        std::chrono::duration<double, std::ratio<1>>(1.0 / this->pub_frequency_param_.as_int());

    // cast duration of double to chrono::milliseconds
    auto interval_msec = std::chrono::duration_cast<std::chrono::milliseconds>(interval_sec);
    timer_ = this->create_wall_timer(interval_msec,
                                     std::bind(&InputCurrentVoltage::publish_messages, this));

    // create instance of dewetron_ Parser and process data.
    // get number of columns in a file and filename from declared parameters (must be casted).
    dewetron_ =
        std::make_unique<ParseDewetron>(filename_param_.as_string(), num_of_cols_param_.as_int());
    process_values();
  }

 private:
  std::vector<std::vector<float>> array_of_processed_data_;
  std::unique_ptr<ParseDewetron> dewetron_;
  digital_twin_msgs::msg::SupplyInput input_msg_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<digital_twin_msgs::msg::SupplyInput>::SharedPtr supply_input_publisher_;

  // all expected parameters are declared here
  rclcpp::Parameter num_of_cols_param_;
  rclcpp::Parameter filename_param_;
  rclcpp::Parameter run_forever_param_;
  rclcpp::Parameter pub_frequency_param_;

  int arr_idx_ = 0;
  bool run_forever_;

  void process_values() { array_of_processed_data_ = dewetron_->get_only_values(); }

  void wrap_to_msg_array(int index) {
    // According to indexes in the file
    input_msg_.currents.current1 = array_of_processed_data_[index][5];
    input_msg_.currents.current2 = array_of_processed_data_[index][4];
    input_msg_.currents.current3 = array_of_processed_data_[index][3];
    input_msg_.voltages.voltage1 = array_of_processed_data_[index][0];
    input_msg_.voltages.voltage2 = array_of_processed_data_[index][1];
    input_msg_.voltages.voltage3 = array_of_processed_data_[index][2];
    input_msg_.header.stamp = rclcpp::Node::now();
  }

  void publish_messages() {
    // If run_forever_ is activated, the index will be reset
    // to 0  when end of file is reached
    if (run_forever_) {
      if (arr_idx_ >= dewetron_->get_number_of_rows()) {
        arr_idx_ = 0;
      }
    }

    wrap_to_msg_array(arr_idx_);
    supply_input_publisher_->publish(input_msg_);
    arr_idx_ += 1;
  }

  void init_ros_params() {
    this->declare_parameter("num_of_cols");
    this->declare_parameter("filename");
    this->declare_parameter("run_forever");
    this->declare_parameter("frequency", 500);

    try {
      num_of_cols_param_ = this->get_parameter("num_of_cols");
      filename_param_ = this->get_parameter("filename");
      run_forever_param_ = this->get_parameter("run_forever");
      pub_frequency_param_ = this->get_parameter("frequency");
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException &e) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Failed to declare parameters for data_processor. Perhaps "
                   "you did not define the namespace correctly?");
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Terminating...");
      exit(1);
    }
  }
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InputCurrentVoltage>());
  rclcpp::shutdown();

  return 0;
}
