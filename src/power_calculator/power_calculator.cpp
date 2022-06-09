#include <stdlib.h>

#include <chrono>
#include <digital_twin_msgs/msg/power.hpp>
#include <digital_twin_msgs/msg/supply_input.hpp>
#include <functional>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/* A PowerValues struct holding values
 *  of all powers that each phase is carrying
 */
struct PowerValues {
  float phase[3] = {0.0};
  float total = 0;
};

/* Node that calculates the electrical and reactive power of an electrical drive based
 *  on the input current and voltage. Current values are buffered in a currents_buffer_
 *  and voltages_buffer_ and used to calculate the powers. Since reactive is computed
 *  almost immediately while electrical power takes time, they are computed asynchronously.
 *  Race conditions and locks are prevented by use of mutex_reactive_ and mutex_electrical_
 *  Subscribers are saving data into input_volatage_ and input_current_ arrays, and
 *  these variables are used by calculate_reactive_power() and calculate_electrical_power,
 *  mutexes ARE REQUIRED to prevent either of callbacks accessing those arrays during calculation
 *  or buffer copying.
 */
class PowerCalculator : public rclcpp::Node {
 public:
  PowerCalculator() : Node("power_calculator") {
    currents_.resize(3);
    voltages_.resize(3);
    set_params();

    power_reactive_publisher_ =
        this->create_publisher<digital_twin_msgs::msg::Power>("motor_power/reactive_power", 100);
    power_electrical_publisher_ =
        this->create_publisher<digital_twin_msgs::msg::Power>("motor_power/electrical_power", 100);
    input_subscriber_ = this->create_subscription<digital_twin_msgs::msg::SupplyInput>(
        "supply_input", 100,
        std::bind(&PowerCalculator::input_callback, this, std::placeholders::_1));

    timer_electrical_ =
        this->create_wall_timer(100ms, std::bind(&PowerCalculator::publish_electrical_power,
                                                 this));  // Here frequency maybe faulty - check it
    timer_reactive_ =
        this->create_wall_timer(100ms, std::bind(&PowerCalculator::publish_reactive_power, this));
  }

 private:
  // https://fortop.co.uk/knowledge/white-papers/cos-phi-compensation-cosinus/

  const float COS_PHI = 0.84;
  int buffer_size_;

  float input_current_[3];
  float input_voltage_[3];
  std::vector<std::vector<float>> currents_;
  std::vector<std::vector<float>> voltages_;
  std::vector<std::vector<float>> current_buffer_;
  std::vector<std::vector<float>> voltage_buffer_;
  int count = 0;
  float rms_currents_[3] = {0.0};
  float rms_voltages_[3] = {0.0};
  bool input_ready_ = false;
  PowerValues reactive_;
  PowerValues electrical_;

  std::mutex mutex_reactive_;
  std::mutex mutex_electrical_;

  /* Declaration of publisher and subscriber shared pointers */

  rclcpp::Subscription<digital_twin_msgs::msg::SupplyInput>::SharedPtr input_subscriber_;
  rclcpp::Publisher<digital_twin_msgs::msg::Power>::SharedPtr power_electrical_publisher_;
  rclcpp::Publisher<digital_twin_msgs::msg::Power>::SharedPtr power_reactive_publisher_;
  rclcpp::TimerBase::SharedPtr timer_electrical_;
  rclcpp::TimerBase::SharedPtr timer_reactive_;
  digital_twin_msgs::msg::Power power_react_msg_;
  digital_twin_msgs::msg::Power power_el_msg_;

  /* parameters declaration */

  rclcpp::Parameter buffer_size_param;

  void set_params() {
    try {
      this->declare_parameter("buffer_size");
      buffer_size_param = this->get_parameter("buffer_size");
      buffer_size_ = buffer_size_param.as_int();
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                  "Passed parameter for buffer_size was empty or invalid. Setting the default "
                  "buffer_size to 1000");
      buffer_size_ = 1000;
    }
  }

  void calculate_RMS() {
    float mean_[2][3] = {0.0};
    float square_[2][3] = {0.0};

    /* square_ */
    std::unique_lock<std::mutex> lock(mutex_electrical_);
    for (int j = 0; j < buffer_size_; j++) {
      square_[0][0] += current_buffer_[0][j] * current_buffer_[0][j];
      square_[0][1] += current_buffer_[1][j] * current_buffer_[1][j];
      square_[0][2] += current_buffer_[2][j] * current_buffer_[2][j];

      square_[1][0] += voltage_buffer_[0][j] * voltage_buffer_[0][j];
      square_[1][1] += voltage_buffer_[1][j] * voltage_buffer_[1][j];
      square_[1][2] += voltage_buffer_[2][j] * voltage_buffer_[2][j];
    }

    lock.unlock();

    for (int j = 0; j < 3; j++) {
      mean_[0][j] = (square_[0][j] / buffer_size_);
      mean_[1][j] = (square_[1][j] / buffer_size_);
    }
    for (int j = 0; j < 3; j++) {
      rms_currents_[j] = std::sqrt(mean_[0][j]);
      rms_voltages_[j] = std::sqrt(mean_[1][j]);
    }
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 3; j++) {
        square_[i][j] = 0;
      }
    }
  }
  void input_callback(const digital_twin_msgs::msg::SupplyInput::SharedPtr msg) {
    std::unique_lock<std::mutex> lock_current(mutex_reactive_);

    input_current_[0] = msg->currents.current1;
    input_current_[1] = msg->currents.current2;
    input_current_[2] = msg->currents.current3;

    input_voltage_[0] = msg->voltages.voltage1;
    input_voltage_[1] = msg->voltages.voltage2;
    input_voltage_[2] = msg->voltages.voltage3;

    lock_current.unlock();

    if (count >= buffer_size_) {
      /* lock guard will automatically release the lock_reactive mutex when
       *  it goes out of scope.
       */
      std::lock_guard<std::mutex> lock_reactive(mutex_electrical_);
      current_buffer_ = currents_;
      voltage_buffer_ = voltages_;
      input_ready_ = true;
      /* Required to clear (in order to avoid overflow of values) and resize to 3, to push back*/
      voltages_.clear();
      currents_.clear();
      voltages_.resize(3);
      currents_.resize(3);
      count = 0;
    }

    voltages_.at(0).push_back(msg->voltages.voltage1);
    voltages_.at(1).push_back(msg->voltages.voltage2);
    voltages_.at(2).push_back(msg->voltages.voltage3);
    currents_.at(0).push_back(msg->currents.current1);
    currents_.at(1).push_back(msg->currents.current2);
    currents_.at(2).push_back(msg->currents.current3);

    count++;
  }

  void calculate_power_reactive() {
    std::lock_guard<std::mutex> lock(mutex_reactive_);
    for (int i = 0; i < 3; i++) {
      reactive_.phase[i] = (abs(input_current_[i] * input_voltage_[i])) / 3;
    }
    reactive_.total = reactive_.phase[0] + reactive_.phase[1] + reactive_.phase[2];
  }

  void calculate_power_electrical() {
    calculate_RMS();

    for (int i = 0; i < 3; i++) {
      electrical_.phase[i] = rms_currents_[i] * rms_voltages_[i] * COS_PHI;
    }
    electrical_.total = electrical_.phase[0] + electrical_.phase[1] + electrical_.phase[2];
  }

  void publish_electrical_power() {
    if (input_ready_) {
      calculate_power_electrical();
      clear_buffers();
      input_ready_ = false;
    }
    power_el_msg_.phase1 = electrical_.phase[0];
    power_el_msg_.phase2 = electrical_.phase[1];
    power_el_msg_.phase3 = electrical_.phase[2];
    power_el_msg_.total = electrical_.total;
    power_el_msg_.header.stamp = rclcpp::Node::now();
    power_electrical_publisher_->publish(power_el_msg_);
  }

  void publish_reactive_power() {
    calculate_power_reactive();
    power_react_msg_.phase1 = reactive_.phase[0];
    power_react_msg_.phase2 = reactive_.phase[1];
    power_react_msg_.phase3 = reactive_.phase[2];
    power_react_msg_.total = reactive_.total;
    power_react_msg_.header.stamp = rclcpp::Node::now();
    power_reactive_publisher_->publish(power_react_msg_);
  }

  void clear_buffers() {
    current_buffer_.clear();
    voltage_buffer_.clear();
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PowerCalculator>());
  rclcpp::shutdown();
  return 0;
}