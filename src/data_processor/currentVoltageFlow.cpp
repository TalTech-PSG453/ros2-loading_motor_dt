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

using namespace std::chrono_literals;

class InputCurrentVoltage : public rclcpp::Node
{

public:

    InputCurrentVoltage() : Node("data_processor")
    {
        /* initializing publishers/subscribers */
        current_publisher = this->create_publisher<digital_twin_msgs::msg::Current>("input_current", 10);
        voltage_publisher = this->create_publisher<digital_twin_msgs::msg::Voltage>("input_voltage", 10);
        timer_ = this->create_wall_timer(1ms, std::bind(&InputCurrentVoltage::publish_messages, this)); // 1ms = 1000 Hz

        /* initialize parameters */
        init_ros_params();
        this->runForever = run_forever_param.as_bool();

        /* Run parser */
        dewetron = new ParseDewetron(filename_param.as_string(), num_of_cols_param.as_int()); // get from params
        processValues();
    }

private:

    std::vector<std::vector<float>> arrayOfProcessedData;
    ParseDewetron *dewetron;
    digital_twin_msgs::msg::Current inputCurrentValuesMsg;
    digital_twin_msgs::msg::Voltage inputVoltageValuesMsg;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<digital_twin_msgs::msg::Current>::SharedPtr current_publisher;
    rclcpp::Publisher<digital_twin_msgs::msg::Voltage>::SharedPtr voltage_publisher;

    /* params */

    rclcpp::Parameter num_of_cols_param;
    rclcpp::Parameter filename_param;
    rclcpp::Parameter run_forever_param;

    int arrIndex = 0;
    bool runForever;

    void processValues()
    {
        arrayOfProcessedData = dewetron->getOnlyValues();
    }

    void wrapToMsgArray(int index)
    {
        /* According to indexes in the file */
        inputCurrentValuesMsg.current1 = arrayOfProcessedData[index][5];
        inputCurrentValuesMsg.current2 = arrayOfProcessedData[index][4];
        inputCurrentValuesMsg.current3 = arrayOfProcessedData[index][3];
        inputVoltageValuesMsg.voltage1 = arrayOfProcessedData[index][0];
        inputVoltageValuesMsg.voltage2 = arrayOfProcessedData[index][1];
        inputVoltageValuesMsg.voltage3 = arrayOfProcessedData[index][2];
    }

    void publish_messages()
    {
        // If runForever is activated, the index will be reset to 0
        if(runForever)
        {
            if (arrIndex >= dewetron->getNumberOfRows())
            {
                arrIndex = 0;
            }
        }

        wrapToMsgArray(arrIndex);
        current_publisher->publish(inputCurrentValuesMsg);
        voltage_publisher->publish(inputVoltageValuesMsg);
        arrIndex += 1;
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
    rclcpp::spin(std::make_shared<InputCurrentVoltage>());
    rclcpp::shutdown();

    return 0;
}
