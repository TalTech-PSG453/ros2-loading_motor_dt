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

using namespace std::chrono_literals;

class TorqueCalculator : public rclcpp::Node
{
private:
    //const int angular_velocity_fake = 157; // rad/s or 1500 rpm  | for debugging
    bool is_velocity_updated_ = false;
    bool is_power_updated_ = false;
    bool is_efficiency_updated = false;
    const std::string = 'torque_calculator';

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

    void init_loggers()
    {

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

    /* Initialize recorders */

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

    // Placeholder function
    /*void save_results()
    {
        std::ofstream RecordedFile;
        RecordedFile.open("~/dev_ws/src/loading_motor_dt/recorded_data/torque_calculator_node.cpp");
        RecordedFile << "topic,sent(#),received(#),time\n";
        RecordedFile << "shaft_angular_velocity," << '-,' << s_angular_counter << ',' << 

        
    }*/
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TorqueCalculator>());
    rclcpp::shutdown();
    return 0;
}