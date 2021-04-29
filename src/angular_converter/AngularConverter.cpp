#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <chrono>
#include <memory>
#include <functional>

using namespace std::chrono_literals;

class AngularConverter : public rclcpp::Node
{
public:
    AngularConverter() : Node("angular_converter")
    {
        rotation_publisher = this->create_publisher<std_msgs::msg::Float32>("actual_rpm", 10);
        feedback_receiver = this->create_subscription<std_msgs::msg::Float32>("shaft_angular_velocity", 10,
                            std::bind(&AngularConverter::velocityReceiver, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(100ms, std::bind(&AngularConverter::publishRotation, this)); // 100ms = 10 Hz
    }

    void publishRotation()
    {
        rotation_publisher->publish(rpm_msg);
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr feedback_receiver;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rotation_publisher;
    rclcpp::TimerBase::SharedPtr timer_;
    std_msgs::msg::Float32 rpm_msg;

    void velocityReceiver(const std_msgs::msg::Float32::SharedPtr msg)
    {
        rpm_msg.data = msg->data*60/(2* M_PI);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AngularConverter>());
    rclcpp::shutdown();
    return 0;
}
