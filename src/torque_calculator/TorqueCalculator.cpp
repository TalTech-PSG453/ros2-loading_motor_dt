//
// Created by sejego on 2/3/21.
//
#include <iostream>
#include <chrono>
#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include <digital_twin_msgs/msg/power.hpp>
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class TorqueCalculator : public rclcpp::Node
{
private:

    bool is_velocity_updated_ = false;
    bool is_power_updated_ = false;
    bool is_efficiency_updated = false;

    float electrical_power_ = 0;
    float efficiency_ = 0;
    float angular_velocity_= 0;
    float mechanical_torque_ = 0;
    float electrical_torque_ref_ = 0;

    float calculateElectricalTorqueRef()
    {
        return (electrical_power_ / angular_velocity_);
    }

    float calculateMechanicalTorque()
    {
        return (electrical_torque_ref_ * efficiency_);
    }

    /* Shared Pointers with ROS methods */
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr efficiencyReceiver;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angularVelocityReceiver;
    rclcpp::Subscription<digital_twin_msgs::msg::Power>::SharedPtr powerReceiver;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr electricalTorquePublisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mechanicalTorquePublisher;
    rclcpp::TimerBase::SharedPtr timer_;

    std_msgs::msg::Float32 electrical_torque_msg_;
    std_msgs::msg::Float32 mechanical_torque_msg_;


public:
    TorqueCalculator() : Node("torque_calculator")
    {
    /* Subscribers */
    powerReceiver = this->create_subscription<digital_twin_msgs::msg::Power>("motor_power/electrical_power", 10,
                            std::bind(&TorqueCalculator::powerListener, this, std::placeholders::_1));
    efficiencyReceiver = this->create_subscription<std_msgs::msg::Float32>("efficiency", 10,
                            std::bind(&TorqueCalculator::efficiencyListener, this, std::placeholders::_1));
    angularVelocityReceiver = this->create_subscription<std_msgs::msg::Float32>("shaft_angular_velocity", 10,
                            std::bind(&TorqueCalculator::angularVelocityListener, this, std::placeholders::_1));

    electrical_torque_msg_.data = 0;
    mechanical_torque_msg_.data = 0;

    /* Publishers */
    electricalTorquePublisher = this->create_publisher<std_msgs::msg::Float32>("electrical_torque_ref", 10);
    mechanicalTorquePublisher = this->create_publisher<std_msgs::msg::Float32>("mechanical_torque", 10);

    timer_ = this->create_wall_timer(100ms, std::bind(&TorqueCalculator::publishTorques, this)); // 100ms = 10 Hz
    }

    float getMechanicalTorque()
    {
        return mechanical_torque_;
    }

    float getElectricalTorqueRef()
    {
        return electrical_torque_ref_;
    }

    void publishTorques()
    {
        electrical_torque_msg_.data = getElectricalTorqueRef();
        mechanical_torque_msg_.data = getMechanicalTorque();
        electricalTorquePublisher->publish(electrical_torque_msg_);
        mechanicalTorquePublisher->publish(mechanical_torque_msg_);
    }

    void powerListener(const digital_twin_msgs::msg::Power::SharedPtr msg)
    {
        electrical_power_ = msg->total;
        is_power_updated_ = true;
        if(is_power_updated_ && is_velocity_updated_)
        {
            electrical_torque_ref_ = calculateElectricalTorqueRef();
            is_power_updated_ = false;
            is_velocity_updated_ = false;
        }
    }
    void efficiencyListener(const std_msgs::msg::Float32::SharedPtr msg)
    {
        efficiency_ = msg->data;
       // is_efficiency_updated_ = true;
        mechanical_torque_ = calculateMechanicalTorque();
    }
    void angularVelocityListener(const std_msgs::msg::Float32::SharedPtr msg)
    {
        angular_velocity_ = msg->data;
        is_velocity_updated_ = true;
        if(is_power_updated_ && is_velocity_updated_)
        {
            electrical_torque_ref_ = calculateElectricalTorqueRef();
            is_power_updated_ = false;
            is_velocity_updated_ = false;
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TorqueCalculator>());
    rclcpp::shutdown();
    return 0;
}