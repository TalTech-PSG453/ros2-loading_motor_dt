#include "rclcpp/rclcpp.hpp"
#include <digital_twin_msgs/msg/float32_stamped.hpp>
#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <chrono>
#include <memory>
#include <functional>
#include "data_logger/data_logger.hpp"

using namespace std::chrono_literals;
using namespace DataLogger;

class AngularConverter : public rclcpp::Node
{
public:

    std::unique_ptr<PublisherLogger> p_rotation_pub;
    std::unique_ptr<SubscriptionLogger> p_angular_sub;

    AngularConverter() : Node("angular_converter")
    {
        rotation_publisher = this->create_publisher<digital_twin_msgs::msg::Float32Stamped>("actual_rpm", 10);
        feedback_receiver = this->create_subscription<digital_twin_msgs::msg::Float32Stamped>("shaft_angular_velocity", 10,
                            std::bind(&AngularConverter::velocityReceiver, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(100ms, std::bind(&AngularConverter::publishRotation, this)); // 100ms = 10 Hz
        p_angular_sub.reset(new SubscriptionLogger("/shaft_angular_velocity"));
        p_rotation_pub.reset(new PublisherLogger("/actual_rpm"));
        rpm_msg.id = 0;
    }

    void publishRotation()
    {
        rpm_msg.stamp = rclcpp::Node::now();
        rotation_publisher->publish(rpm_msg);
        rpm_msg.id += 1;
        p_rotation_pub->sent_counter += 1;
    }

private:
    rclcpp::Subscription<digital_twin_msgs::msg::Float32Stamped>::SharedPtr feedback_receiver;
    rclcpp::Publisher<digital_twin_msgs::msg::Float32Stamped>::SharedPtr rotation_publisher;
    rclcpp::TimerBase::SharedPtr timer_;
    digital_twin_msgs::msg::Float32Stamped rpm_msg;

    void velocityReceiver(const digital_twin_msgs::msg::Float32Stamped::SharedPtr msg)
    {
        rpm_msg.data = msg->data*60/(2* M_PI);
        if(msg->id == p_angular_sub->next_id)
        {
            p_angular_sub->recv_counter += 1;
            rclcpp::Duration diff = rclcpp::Node::now() - msg->stamp;
            p_angular_sub->time_diffs.push_back(diff.nanoseconds());
        }
        p_angular_sub->next_id = msg->id + 1;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto ptr = std::make_shared<AngularConverter>();
    rclcpp::spin(ptr);
    DataLogger::save_logged_data("angular_converter.csv");
    rclcpp::shutdown();
    return 0;
}
