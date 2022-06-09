#include <array>
#include <chrono>
#include <cmath>
#include <digital_twin_msgs/msg/supply_input.hpp>
#include <iostream>
#include <memory>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

using namespace std::chrono_literals;

/* Checks current input for abnormal values.
 *  Accumulates current data into a buffer currents_buffer_ and
 *  calculates root mean square (RMS) values. RMS values of each phase
 *  are then used to get a reference value (used as coefficient) and
 *  compare phase values. If there is a significant difference bigger
 *  than ERR_MARGIN then a warning message is fired.
 *  The phase_checker() works asynchronously with the main (subscriber thread),
 *  The asynchronous control is handled through variables can_calculate_ and not_terminated_
 */
class WindingErrorChecker : public rclcpp::Node {
 public:
  std::vector<std::vector<float>> currents_;
  std::vector<std::vector<float>> currents_buffer_;
  bool can_calculate_ = false;
  std_msgs::msg::String warning_msg_;

  WindingErrorChecker() : Node("windings_checker") {
    currents_.resize(3);
    can_calculate_ = false;
    t_phase_checker = std::thread(&WindingErrorChecker::phase_checker, this);

    currents_listener_ = this->create_subscription<digital_twin_msgs::msg::SupplyInput>(
        "tb_lm/supply_input", 10,
        std::bind(&WindingErrorChecker::current_callback, this, std::placeholders::_1));

    warning_publisher_ =
        this->create_publisher<std_msgs::msg::String>("diagnostics/warnings/windings", 10);
  }
  ~WindingErrorChecker() {
    not_terminated_ = false;
    t_phase_checker.join();
    currents_.clear();
    currents_buffer_.clear();
  }

 private:
  rclcpp::Subscription<digital_twin_msgs::msg::SupplyInput>::SharedPtr currents_listener_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr warning_publisher_;

  bool not_terminated_ = true;
  std::array<float, 3> rms_currents_ = {0.0, 0.0, 0.0};
  float error_coefficient_[3];
  int i = 0;
  const int BUFFER_SIZE = 5000;
  const float ERR_MARGIN = 0.15;
  std::thread t_phase_checker;

  void current_callback(const digital_twin_msgs::msg::SupplyInput::SharedPtr msg) {
    if (i >= BUFFER_SIZE) {
      i = 0;
      currents_buffer_ = currents_;
      can_calculate_ = true;
      currents_.clear();
      currents_.resize(3);
    }

    currents_.at(0).push_back(msg->currents.current1);
    currents_.at(1).push_back(msg->currents.current2);
    currents_.at(2).push_back(msg->currents.current3);
    i++;
  }

  void publish_warning() {
    warning_msg_.data = "Winding warning";
    warning_publisher_->publish(warning_msg_);
  }

  std::array<float, 3> calculate_RMS(std::vector<std::vector<float>> &buffer) {
    float mean[3];
    float square[3] = {0.0};
    std::array<float, 3> root = {0.0, 0.0, 0.0};
    /* squares */
    for (int j = 0; j < BUFFER_SIZE; j++) {
      square[0] += buffer[0][j] * buffer[0][j];
      square[1] += buffer[1][j] * buffer[1][j];
      square[2] += buffer[2][j] * buffer[2][j];
    }
    for (int j = 0; j < 3; j++) {
      mean[j] = (square[j] / BUFFER_SIZE);
    }
    for (int j = 0; j < 3; j++) {
      root[j] = std::sqrt(mean[j]);
    }
    return root;
  }

  void phase_checker() {
    bool problematic = false;

    while (true) {
      if (!not_terminated_) break;

      if (can_calculate_) {
        rms_currents_ = calculate_RMS(currents_buffer_);

        for (int j = 0; j < 3; j++) {
          error_coefficient_[j] = rms_currents_[j] / rms_currents_[0];
        }
        for (float j : error_coefficient_) {
          if (abs(error_coefficient_[0] - j) >= ERR_MARGIN) {
            problematic = true;
            break;
          }
        }

        if (problematic == true) {
          RCLCPP_WARN(this->get_logger(), "Potential malfunction in motor windings");
          publish_warning();
          problematic = false;
        }
        /* CLEANUP */
        currents_buffer_.clear();
        can_calculate_ = false;
      }
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WindingErrorChecker>());
  rclcpp::shutdown();
  return 0;
}
