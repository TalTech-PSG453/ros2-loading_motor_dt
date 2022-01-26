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
#include "std_msgs/msg/float32.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std::chrono_literals;
using namespace message_filters;

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
    Subscriber<digital_twin_msgs::msg::Float32Stamped> angularVelocityReceiver;
    Subscriber<digital_twin_msgs::msg::Power> powerReceiver;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr electricalTorquePublisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mechanicalTorquePublisher;

    // ApproximateTime Policy synchronizer definitions. This is required to approximately synchronize the incoming data from 2  different
    // topics for further calculations. In theory, should increase accuracy. More info below:
    // http://wiki.ros.org/message_filters

    typedef sync_policies::ApproximateTime<digital_twin_msgs::msg::Power, digital_twin_msgs::msg::Float32Stamped> ApproxSyncPolicy;
    std::shared_ptr<Synchronizer<ApproxSyncPolicy>> msg_sync_;
    rclcpp::TimerBase::SharedPtr timer_;

    std_msgs::msg::Float32 electrical_torque_msg_;
    std_msgs::msg::Float32 mechanical_torque_msg_;


public:
    TorqueCalculator() : Node("torque_calculator")
    {
        /* Subscribers */

        efficiencyReceiver = this->create_subscription<std_msgs::msg::Float32>("efficiency", 10,
                            std::bind(&TorqueCalculator::efficiencyListener, this, std::placeholders::_1));

        powerReceiver.subscribe(this, "motor_power/electrical_power");
        angularVelocityReceiver.subscribe(this, "actual_rpm"); // shaft_angular_velocity

        // The way synchronizer is implemented can be found below:
        // https://answers.ros.org/question/366440/ros-2-message_filters-timesynchronizer-minimal-example-does-not-reach-callback-function/

        msg_sync_ = std::make_shared<Synchronizer<ApproxSyncPolicy>>(ApproxSyncPolicy(100),powerReceiver, angularVelocityReceiver);
        msg_sync_->registerCallback(std::bind(&TorqueCalculator::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

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

        void efficiencyListener(const std_msgs::msg::Float32::SharedPtr msg)
        {
            efficiency_ = msg->data;
            mechanical_torque_ = calculateMechanicalTorque();
        }

        void syncCallback(
            const digital_twin_msgs::msg::Power::ConstSharedPtr& msg1,
            const digital_twin_msgs::msg::Float32Stamped::ConstSharedPtr& msg2)
        {
            /*
            RCLCPP_INFO(this->get_logger(),
                "I heard and synchronized the following timestamps: %u, %u",
                msg1->header.stamp.sec, msg2->header.stamp.sec);
            */
           electrical_power_ = msg1->total;
           angular_velocity_ = msg2->data;
           electrical_torque_ref_ = calculateElectricalTorqueRef();
        }
    };

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TorqueCalculator>());
    rclcpp::shutdown();
    return 0;
}