#include "rclcpp/rclcpp.hpp"
#include <digital_twin_msgs/msg/float32_stamped.hpp>
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
        rotation_publisher = this->create_publisher<digital_twin_msgs::msg::Float32Stamped>("actual_rpm", 10);
        feedback_receiver = this->create_subscription<digital_twin_msgs::msg::Float32Stamped>("shaft_angular_velocity", 10,
                            std::bind(&AngularConverter::velocityReceiver, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(100ms, std::bind(&AngularConverter::publishRotation, this)); // 100ms = 10 Hz
    }

    void publishRotation()
    {   rpm_msg.stamp = rclcpp::Node::now();
        rotation_publisher->publish(rpm_msg);
    }

private:
    rclcpp::Subscription<digital_twin_msgs::msg::Float32Stamped>::SharedPtr feedback_receiver;
    rclcpp::Publisher<digital_twin_msgs::msg::Float32Stamped>::SharedPtr rotation_publisher;
    rclcpp::TimerBase::SharedPtr timer_;
    digital_twin_msgs::msg::Float32Stamped rpm_msg;

    void velocityReceiver(const digital_twin_msgs::msg::Float32Stamped::SharedPtr msg)
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
