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
#include <digital_twin_msgs/msg/supply_input.hpp>
#include "ParseDewetron.h"

using namespace std::chrono_literals;

class InputCurrentVoltage : public rclcpp::Node
{

public:

    InputCurrentVoltage() : Node("data_processor")
    {
        /* initializing publishers/subscribers */
        supply_input_publisher_ = this->create_publisher<digital_twin_msgs::msg::SupplyInput>("supply_input", 10);
        timer_ = this->create_wall_timer(0.1ms, std::bind(&InputCurrentVoltage::publish_messages, this)); // 1ms = 1000 Hz

        /* initialize parameters */
        init_ros_params();
        this->run_forever_ = run_forever_param.as_bool();

        /* Run parser */
        dewetron = new ParseDewetron(filename_param.as_string(), num_of_cols_param.as_int()); // get from params
        processValues();
    }

private:

    std::vector<std::vector<float>> array_of_processed_data_;
    ParseDewetron *dewetron;
    digital_twin_msgs::msg::SupplyInput inputMsg;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<digital_twin_msgs::msg::SupplyInput>::SharedPtr supply_input_publisher_;

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
        inputMsg.currents.current1 = array_of_processed_data_[index][5];
        inputMsg.currents.current2 = array_of_processed_data_[index][4];
        inputMsg.currents.current3 = array_of_processed_data_[index][3];
        inputMsg.voltages.voltage1 = array_of_processed_data_[index][0];
        inputMsg.voltages.voltage2 = array_of_processed_data_[index][1];
        inputMsg.voltages.voltage3 = array_of_processed_data_[index][2];
        inputMsg.header.stamp = rclcpp::Node::now();
    }

    void publish_messages()
    {
        // If run_forever_ is activated, the index will be reset to 0
        if(run_forever_) {
            if (arr_idx_ >= dewetron->getNumberOfRows()) {
                arr_idx_ = 0;
            }
        }

        wrapToMsgArray(arr_idx_);
        supply_input_publisher_->publish(inputMsg);
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
    rclcpp::spin(std::make_shared<InputCurrentVoltage>());
    rclcpp::shutdown();

    return 0;
}
