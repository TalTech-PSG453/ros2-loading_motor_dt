//
// Created by sejego on 2/3/21.
//
#include <iostream>
#include <chrono>
#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include <digital_twin_msgs/msg/power.hpp>
#include <digital_twin_msgs/msg/float32_stamped.hpp>
#include <string>
#include <fstream>

#include "data_logger/data_logger.hpp"

using namespace std::chrono_literals;
using namespace DataLogger;

class TorqueCalculator : public rclcpp::Node
{
private:
    //const int angular_velocity_fake = 157; // rad/s or 1500 rpm  | for debugging
    bool is_velocity_updated_ = false;
    bool is_power_updated_ = false;
    bool is_efficiency_updated = false;

    float electrical_power_, efficiency_, angular_velocity_;
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
    rclcpp::Subscription<digital_twin_msgs::msg::Float32Stamped>::SharedPtr efficiencyReceiver;
    rclcpp::Subscription<digital_twin_msgs::msg::Float32Stamped>::SharedPtr angularVelocityReceiver;
    rclcpp::Subscription<digital_twin_msgs::msg::Power>::SharedPtr powerReceiver;
    rclcpp::Publisher<digital_twin_msgs::msg::Float32Stamped>::SharedPtr electricalTorquePublisher;
    rclcpp::Publisher<digital_twin_msgs::msg::Float32Stamped>::SharedPtr mechanicalTorquePublisher;
    rclcpp::TimerBase::SharedPtr timer_;

    digital_twin_msgs::msg::Float32Stamped electrical_torque_msg_;
    digital_twin_msgs::msg::Float32Stamped mechanical_torque_msg_;

public:

    std::unique_ptr<SubscriptionLogger> p_power_rec;
    std::unique_ptr<SubscriptionLogger> p_efficiency_rec;
    std::unique_ptr<SubscriptionLogger> p_angular_rec;
    std::unique_ptr<PublisherLogger> p_electrical_pub;
    std::unique_ptr<PublisherLogger> p_mechanical_pub;


    TorqueCalculator() : Node("torque_calculator")
    {
        
        /* Subscribers */
        powerReceiver = this->create_subscription<digital_twin_msgs::msg::Power>("motor_power/electrical_power", 100,
                                std::bind(&TorqueCalculator::powerListener, this, std::placeholders::_1));
        efficiencyReceiver = this->create_subscription<digital_twin_msgs::msg::Float32Stamped>("efficiency", 100,
                                std::bind(&TorqueCalculator::efficiencyListener, this, std::placeholders::_1));
        angularVelocityReceiver = this->create_subscription<digital_twin_msgs::msg::Float32Stamped>("shaft_angular_velocity", 100,
                                std::bind(&TorqueCalculator::angularVelocityListener, this, std::placeholders::_1));

        electrical_torque_msg_.data = 0;
        mechanical_torque_msg_.data = 0;

        /* Publishers */
        electricalTorquePublisher = this->create_publisher<digital_twin_msgs::msg::Float32Stamped>("electrical_torque_ref", 10);
        mechanicalTorquePublisher = this->create_publisher<digital_twin_msgs::msg::Float32Stamped>("mechanical_torque", 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&TorqueCalculator::publishTorques, this)); // 100ms = 10 Hz

        p_power_rec.reset(new SubscriptionLogger("/electrical_power"));
        p_efficiency_rec.reset(new SubscriptionLogger("/efficiency"));
        p_angular_rec.reset(new SubscriptionLogger("/shaft_angular_velocity"));
        p_electrical_pub.reset(new PublisherLogger("/electrical_torque"));
        p_mechanical_pub.reset(new PublisherLogger("/mechanical_torque"));
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
        electrical_torque_msg_.stamp = rclcpp::Node::now();
        mechanical_torque_msg_.stamp = rclcpp::Node::now();
        electricalTorquePublisher->publish(electrical_torque_msg_);
        p_electrical_pub->sent_counter += 0;
        mechanicalTorquePublisher->publish(mechanical_torque_msg_);
        p_mechanical_pub->sent_counter += 0;
    }

    void powerListener(const digital_twin_msgs::msg::Power::SharedPtr msg)
    {
        electrical_power_ = msg->total;
        p_power_rec->recv_counter += 1;
        rclcpp::Duration diff = rclcpp::Node::now() - msg->stamp;
        p_power_rec->time_diffs.push_back(diff.nanoseconds());
        is_power_updated_ = true;
        if(is_power_updated_ && is_velocity_updated_)
        {
            electrical_torque_ref_ = calculateElectricalTorqueRef();
            is_power_updated_ = false;
            is_velocity_updated_ = false;
        }
    }
    void efficiencyListener(const digital_twin_msgs::msg::Float32Stamped::SharedPtr msg)
    {
        efficiency_ = msg->data;
       // is_efficiency_updated_ = true;
        mechanical_torque_ = calculateMechanicalTorque();
    }
    void angularVelocityListener(const digital_twin_msgs::msg::Float32Stamped::SharedPtr msg)
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