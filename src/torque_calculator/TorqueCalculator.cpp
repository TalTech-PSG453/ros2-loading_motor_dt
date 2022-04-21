//
// Created by sejego on 2/3/21.
//
#include <iostream>
#include <chrono>
#include <memory>
#include <functional>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include <digital_twin_msgs/msg/power.hpp>
#include <digital_twin_msgs/msg/float32_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std::chrono_literals;
using namespace message_filters;

class TorqueCalculator : public rclcpp::Node
{
    private:

        float electrical_power_ = 0;
        float efficiency_ = 0;
        float angular_velocity_= 0;
        float mechanical_torque_ = 0;
        float electrical_torque_ref_ = 0;
        
        std::mutex mutex_;

        float calculateElectricalTorqueRef()
        {
            return (electrical_power_ / angular_velocity_);
        }

        float calculateMechanicalTorque()
        {
            return (electrical_torque_ref_ * efficiency_);
        }

        /* Shared Pointers with ROS methods */
        Subscriber<digital_twin_msgs::msg::Float32Stamped> angularVelocityReceiver;
        Subscriber<digital_twin_msgs::msg::Power> powerReceiver;
        Subscriber<digital_twin_msgs::msg::Float32Stamped> efficiencyReceiver;
        rclcpp::Publisher<digital_twin_msgs::msg::Float32Stamped>::SharedPtr electricalTorquePublisher;
        rclcpp::Publisher<digital_twin_msgs::msg::Float32Stamped>::SharedPtr mechanicalTorquePublisher;

        // ApproximateTime Policy synchronizer definitions. This is required to approximately synchronize the incoming data from 2  different
        // topics for further calculations. In theory, should increase accuracy. More info below:
        // http://wiki.ros.org/message_filters

        typedef sync_policies::ApproximateTime<digital_twin_msgs::msg::Power, digital_twin_msgs::msg::Float32Stamped, digital_twin_msgs::msg::Float32Stamped> ApproxSyncPolicy;
        std::shared_ptr<Synchronizer<ApproxSyncPolicy>> msg_sync_;
        rclcpp::TimerBase::SharedPtr timer_;

        digital_twin_msgs::msg::Float32Stamped electrical_torque_msg_;
        digital_twin_msgs::msg::Float32Stamped mechanical_torque_msg_;


    public:
        TorqueCalculator() : Node("torque_calculator")
        {
            /* Subscribers */

            powerReceiver.subscribe(this, "motor_power/electrical_power");
            angularVelocityReceiver.subscribe(this, "actual_rpm"); // shaft_angular_velocity
            efficiencyReceiver.subscribe(this, "efficiency");

            // The way synchronizer is implemented can be found below:
            // https://answers.ros.org/question/366440/ros-2-message_filters-timesynchronizer-minimal-example-does-not-reach-callback-function/

            msg_sync_ = std::make_shared<Synchronizer<ApproxSyncPolicy>>(ApproxSyncPolicy(100),powerReceiver, angularVelocityReceiver, efficiencyReceiver);
            msg_sync_->registerCallback(std::bind(&TorqueCalculator::syncCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

            electrical_torque_msg_.data = 0.0;
            mechanical_torque_msg_.data = 0.0;

            /* Publishers */

            electricalTorquePublisher = this->create_publisher<digital_twin_msgs::msg::Float32Stamped>("electrical_torque_ref", 10);
            mechanicalTorquePublisher = this->create_publisher<digital_twin_msgs::msg::Float32Stamped>("mechanical_torque", 10);

            timer_ = this->create_wall_timer(100ms, std::bind(&TorqueCalculator::publishTorques, this)); // 100ms = 10 Hz
        }

            // In order to avoid racing conditions in callback and publisher, mutexes are used to ensure only
            // one method is reading/writing to variables at a time

            void publishTorques()
            {
                const std::lock_guard<std::mutex> lock(mutex_);
                electrical_torque_msg_.data = electrical_torque_ref_;
                mechanical_torque_msg_.data = mechanical_torque_;
                electrical_torque_msg_.stamp = rclcpp::Node::now();
                mechanical_torque_msg_.stamp = rclcpp::Node::now();
                electricalTorquePublisher->publish(electrical_torque_msg_);
                mechanicalTorquePublisher->publish(mechanical_torque_msg_);
            }

            void syncCallback(
                const digital_twin_msgs::msg::Power::ConstSharedPtr& msg1,
                const digital_twin_msgs::msg::Float32Stamped::ConstSharedPtr& msg2,
                const digital_twin_msgs::msg::Float32Stamped::ConstSharedPtr& msg3)
            {
                electrical_power_ = msg1->total;
                angular_velocity_ = msg2->data;
                efficiency_ = msg3->data;
                const std::lock_guard<std::mutex> lock(mutex_);
                electrical_torque_ref_ = calculateElectricalTorqueRef();
                mechanical_torque_ = calculateMechanicalTorque();
            }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TorqueCalculator>());
    rclcpp::shutdown();
    return 0;
}