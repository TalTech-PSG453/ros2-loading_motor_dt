//
// Created by sejego on 10/19/20.
// Last modified by sejego on 26/03/21.
//
//

#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include <digital_twin_msgs/msg/current.hpp>
#include <digital_twin_msgs/msg/voltage.hpp>

#include "ParseDewetron.h"
#include "data_logger/data_logger.hpp"

using namespace std::chrono_literals;
using namespace DataLogger;
class InputCurrentVoltage : public rclcpp::Node
{

public:

    std::unique_ptr<PublisherLogger> p_current_pub;
    std::unique_ptr<PublisherLogger> p_voltage_pub;

    InputCurrentVoltage() : Node("data_processor")
    {
        /* initializing publishers/subscribers */
        current_publisher_ = this->create_publisher<digital_twin_msgs::msg::Current>("input_current", 10);
        voltage_publisher_ = this->create_publisher<digital_twin_msgs::msg::Voltage>("input_voltage", 10);
        timer_ = this->create_wall_timer(1ms, std::bind(&InputCurrentVoltage::publish_messages, this)); // 1ms = 1000 Hz

        /* initialize parameters */
        init_ros_params();
        this->run_forever_ = run_forever_param.as_bool();

        /* Run parser */
        dewetron = new ParseDewetron(filename_param.as_string(), num_of_cols_param.as_int()); // get from params
        processValues();

        p_current_pub.reset(new PublisherLogger("/input_current"));
        p_voltage_pub.reset(new PublisherLogger("/input_voltage"));
        inputCurrentValuesMsg.id = 0;    
        inputVoltageValuesMsg.id = 0;

    }

private:

    std::vector<std::vector<float>> array_of_processed_data_;
    ParseDewetron *dewetron;
    digital_twin_msgs::msg::Current inputCurrentValuesMsg;
    digital_twin_msgs::msg::Voltage inputVoltageValuesMsg;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<digital_twin_msgs::msg::Current>::SharedPtr current_publisher_;
    rclcpp::Publisher<digital_twin_msgs::msg::Voltage>::SharedPtr voltage_publisher_;

    /* params */

    rclcpp::Parameter num_of_cols_param;
    rclcpp::Parameter filename_param;
    rclcpp::Parameter run_forever_param;

    int arr_idx_ = 0;
    bool run_forever_;

    void processValues()
    {
        array_of_processed_data_ = dewetron->getOnlyValues();
    }

    void wrapToMsgArray(int index)
    {
        /* According to indexes in the file */
        inputCurrentValuesMsg.current1 = array_of_processed_data_[index][5];
        inputCurrentValuesMsg.current2 = array_of_processed_data_[index][4];
        inputCurrentValuesMsg.current3 = array_of_processed_data_[index][3];
        inputVoltageValuesMsg.voltage1 = array_of_processed_data_[index][0];
        inputVoltageValuesMsg.voltage2 = array_of_processed_data_[index][1];
        inputVoltageValuesMsg.voltage3 = array_of_processed_data_[index][2];
    }

    void publish_messages()
    {
        // If run_forever_ is activated, the index will be reset to 0
        if(run_forever_)
        {
            if (arr_idx_ >= dewetron->getNumberOfRows())
            {
                arr_idx_ = 0;
            }
        }

        wrapToMsgArray(arr_idx_);
        inputCurrentValuesMsg.stamp = rclcpp::Node::now();
        inputVoltageValuesMsg.stamp = rclcpp::Node::now();
        current_publisher_->publish(inputCurrentValuesMsg);
        p_current_pub->sent_counter += 1;
        inputCurrentValuesMsg.id += 1;
        voltage_publisher_->publish(inputVoltageValuesMsg);
        p_voltage_pub->sent_counter += 1;
        inputVoltageValuesMsg.id += 1;
        arr_idx_ += 1;
    }

    void init_ros_params()
    {
        this->declare_parameter("num_of_cols");
        this->declare_parameter("filename");
        this->declare_parameter("run_forever");

        num_of_cols_param = this->get_parameter("num_of_cols");
        filename_param = this->get_parameter("filename");
        run_forever_param = this->get_parameter("run_forever");
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto ptr = std::make_shared<InputCurrentVoltage>();
    rclcpp::spin(ptr);
    DataLogger::save_logged_data("data_processor.csv");
    rclcpp::shutdown();

    return 0;
}
