#include <iostream>
#define _USE_MATH_DEFINES
#include <chrono>
#include <cmath>
#include <digital_twin_msgs/msg/float32_stamped.hpp>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class AngularConverter : public rclcpp::Node {
 public:
  AngularConverter() : Node("angular_converter") {
    rotation_publisher_ =
        this->create_publisher<digital_twin_msgs::msg::Float32Stamped>("actual_rpm", 10);
    feedback_receiver_ = this->create_subscription<digital_twin_msgs::msg::Float32Stamped>(
        "shaft_angular_velocity", 10,
        std::bind(&AngularConverter::velocityReceiver, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        100ms, std::bind(&AngularConverter::publishRotation, this));  // 100ms = 10 Hz
  }

  void publishRotation() {
    std::lock_guard<std::mutex> lock(mutex_);
    rpm_msg_.stamp = rclcpp::Node::now();
    rotation_publisher_->publish(rpm_msg_);
  }

 private:
  rclcpp::Subscription<digital_twin_msgs::msg::Float32Stamped>::SharedPtr feedback_receiver_;
  rclcpp::Publisher<digital_twin_msgs::msg::Float32Stamped>::SharedPtr rotation_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  digital_twin_msgs::msg::Float32Stamped rpm_msg_;
  std::mutex mutex_;

  void velocityReceiver(const digital_twin_msgs::msg::Float32Stamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    rpm_msg_.data = msg->data * 60 / (2 * M_PI);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AngularConverter>());
  rclcpp::shutdown();
  return 0;
}
