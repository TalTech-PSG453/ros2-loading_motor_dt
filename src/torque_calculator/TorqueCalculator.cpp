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
    //const int angularVelocity = 157; // rad/s or 1500 rpm  | for debugging
    bool velocityUpdated = false;
    bool powerUpdated = false;
    bool efficiencyUpdated = false;

    float electricalPower, efficiency, angularVelocity;
    float mechanicalTorque = 0;
    float electricalTorqueRef = 0;

    float calculateElectricalTorqueRef()
    {
        return (electricalPower / angularVelocity);
    }

    float calculateMechanicalTorque()
    {
        return (electricalTorqueRef * efficiency);
    }

    /* Shared Pointers with ROS methods */
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr efficiencyReceiver;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angularVelocityReceiver;
    rclcpp::Subscription<digital_twin_msgs::msg::Power>::SharedPtr powerReceiver;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr electricalTorquePublisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mechanicalTorquePublisher;
    rclcpp::TimerBase::SharedPtr timer_;

    std_msgs::msg::Float32 electrical_torque_msg;
    std_msgs::msg::Float32 mechanical_torque_msg;


public:
    TorqueCalculator() : Node("torque_calculator")
    {
    /* Subscribers */
    powerReceiver = this->create_subscription<digital_twin_msgs::msg::Power>("motor_power/electrical_power", 100,
                            std::bind(&TorqueCalculator::powerListener, this, std::placeholders::_1));
    efficiencyReceiver = this->create_subscription<std_msgs::msg::Float32>("efficiency", 100,
                            std::bind(&TorqueCalculator::efficiencyListener, this, std::placeholders::_1));
    angularVelocityReceiver = this->create_subscription<std_msgs::msg::Float32>("shaft_angular_velocity", 100,
                            std::bind(&TorqueCalculator::angularVelocityListener, this, std::placeholders::_1));

    electrical_torque_msg.data = 0;
    mechanical_torque_msg.data = 0;

    /* Publishers */
    electricalTorquePublisher = this->create_publisher<std_msgs::msg::Float32>("electrical_torque_ref", 10);
    mechanicalTorquePublisher = this->create_publisher<std_msgs::msg::Float32>("mechanical_torque", 10);

    timer_ = this->create_wall_timer(100ms, std::bind(&TorqueCalculator::publishTorques, this)); // 100ms = 10 Hz
    }

    float getMechanicalTorque()
    {
        return mechanicalTorque;
    }

    float getElectricalTorqueRef()
    {
        return electricalTorqueRef;
    }

    void publishTorques()
    {
        electrical_torque_msg.data = getElectricalTorqueRef();
        mechanical_torque_msg.data = getMechanicalTorque();
        electricalTorquePublisher->publish(electrical_torque_msg);
        mechanicalTorquePublisher->publish(mechanical_torque_msg);
    }

    void powerListener(const digital_twin_msgs::msg::Power::SharedPtr msg)
    {
        electricalPower = msg->total;
        powerUpdated = true;
        if(powerUpdated && velocityUpdated)
        {
            electricalTorqueRef = calculateElectricalTorqueRef();
            powerUpdated = false;
            velocityUpdated = false;
        }
    }
    void efficiencyListener(const std_msgs::msg::Float32::SharedPtr msg)
    {
        efficiency = msg->data;
       // efficiencyUpdated = true;
        mechanicalTorque = calculateMechanicalTorque();
    }
    void angularVelocityListener(const std_msgs::msg::Float32::SharedPtr msg)
    {
        angularVelocity = msg->data;
        velocityUpdated = true;
        if(powerUpdated && velocityUpdated)
        {
            electricalTorqueRef = calculateElectricalTorqueRef();
            powerUpdated = false;
            velocityUpdated = false;
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