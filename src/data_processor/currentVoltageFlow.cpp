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

class InputCurrentVoltage : public rclcpp::Node
{

public:

    InputCurrentVoltage() : Node("data_processor")
    {
        /* initialize parameters */
        init_ros_params();
        this->run_forever_ = run_forever_param.as_bool();

        /* initializing publishers/subscribers */
        supply_input_publisher_ = this->create_publisher<digital_twin_msgs::msg::SupplyInput>("supply_input", 10);

        auto interval_sec = std::chrono::duration<double, std::ratio<1>>(1.0 / this->pub_frequency_param.as_int());  // cast double Hz to duration in secs of type double
        auto interval_msec = std::chrono::duration_cast<std::chrono::milliseconds>(interval_sec);                     // cast duration of double to milliseconds
        
        timer_ = this->create_wall_timer(interval_msec, std::bind(&InputCurrentVoltage::publish_messages, this));

        /* Run parser */
        dewetron = std::make_unique<ParseDewetron>(filename_param.as_string(), num_of_cols_param.as_int()); // get number of columns in a file and filename from 
                                                                                                            // declared parameters.
        processValues();
    }

private:

    std::vector<std::vector<float>> array_of_processed_data_;
    std::unique_ptr<ParseDewetron> dewetron;
    digital_twin_msgs::msg::SupplyInput inputMsg;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<digital_twin_msgs::msg::SupplyInput>::SharedPtr supply_input_publisher_;

    /* params */

    rclcpp::Parameter num_of_cols_param;
    rclcpp::Parameter filename_param;
    rclcpp::Parameter run_forever_param;
    rclcpp::Parameter pub_frequency_param;

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
        inputMsg.stamp = rclcpp::Node::now();
    }

    void publish_messages()
    {
        /* If run_forever_ is activated, the index will be reset to 0  when end of file is reached*/
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
        this->declare_parameter("frequency", 500);

        try{
            num_of_cols_param = this->get_parameter("num_of_cols");
            filename_param = this->get_parameter("filename");
            run_forever_param = this->get_parameter("run_forever");
            pub_frequency_param = this->get_parameter("frequency");
        }
        catch(const rclcpp::exceptions::ParameterNotDeclaredException& e){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to declare parameters for data_processor. Perhaps you did not define the namespace correctly?");
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Terminating...");
            exit(1);
        }
    }

};
int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<InputCurrentVoltage>());
    rclcpp::shutdown();

    return 0;
}
