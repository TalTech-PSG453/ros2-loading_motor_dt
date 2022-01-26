#include "rclcpp/rclcpp.hpp"
#include <stdlib.h>
#include <chrono>
#include <memory>
#include <functional>
#include <digital_twin_msgs/msg/supply_input.hpp>
#include <digital_twin_msgs/msg/power.hpp>

using namespace std::chrono_literals;

 struct powerValues
{
    float phase[3] = {0.0};
    float total = 0;
};

class PowerCalculator : public rclcpp::Node
{
private:

    const float COS_PHI = 0.84; // https://fortop.co.uk/knowledge/white-papers/cos-phi-compensation-cosinus/
    const int SIZE_A = 5000;

    float input_current_[3];
    float input_voltage_[3];
    std::vector<std::vector<float>> currents_;
    std::vector<std::vector<float>> voltages_;
    std::vector<std::vector<float>> current_buffer_;
    std::vector<std::vector<float>> voltage_buffer_;
    int count = 0;
    float rms_currents_[3] = {0.0};
    float rms_voltages_[3] = {0.0};
    bool input_ready_ = false;
    powerValues reactive_;
    powerValues electrical_;

    /* Declaration of publisher and subscriber shared pointers */

    rclcpp::Subscription<digital_twin_msgs::msg::SupplyInput>::SharedPtr InputSubscriber;
    rclcpp::Publisher<digital_twin_msgs::msg::Power>::SharedPtr PowerElectricalPublisher;
    rclcpp::Publisher<digital_twin_msgs::msg::Power>::SharedPtr PowerReactivePublisher;
    rclcpp::TimerBase::SharedPtr timer_electrical_;
    rclcpp::TimerBase::SharedPtr timer_reactive_;
    digital_twin_msgs::msg::Power powerReactMsg;
    digital_twin_msgs::msg::Power powerElMsg;

    void calculateRMS()
    {
        float mean_[2][3] ={0.0};
        float square_[2][3] = {0.0};

        /* square_ */
        for (int j = 0; j < SIZE_A; j++){

            square_[0][0] += current_buffer_[0][j] * current_buffer_[0][j];
            square_[0][1] += current_buffer_[1][j] * current_buffer_[1][j];
            square_[0][2] += current_buffer_[2][j] * current_buffer_[2][j];

            square_[1][0] += voltage_buffer_[0][j] * voltage_buffer_[0][j];
            square_[1][1] += voltage_buffer_[1][j] * voltage_buffer_[1][j];
            square_[1][2] += voltage_buffer_[2][j] * voltage_buffer_[2][j];
        }
        for (int j = 0; j < 3; j++){

            mean_[0][j] = (square_[0][j] / SIZE_A);
            mean_[1][j] = (square_[1][j] / SIZE_A);
        }
        for (int j = 0; j < 3; j++){

            rms_currents_[j] = std::sqrt(mean_[0][j]);
            rms_voltages_[j] = std::sqrt(mean_[1][j]);
        }
        for(int i=0;i<2;i++){

            for(int j=0; j<3; j++){
                square_[i][j] = 0;
            }
        }
    }

public:

    PowerCalculator() : Node("power_calculator")
    {
        currents_.resize(3);
        voltages_.resize(3);

        PowerReactivePublisher = this->create_publisher<digital_twin_msgs::msg::Power>("motor_power/reactive_power", 100);
        PowerElectricalPublisher = this->create_publisher<digital_twin_msgs::msg::Power>("motor_power/electrical_power", 100);
        InputSubscriber = this->create_subscription<digital_twin_msgs::msg::SupplyInput>("supply_input", 100, 
                                                    std::bind(&PowerCalculator::inputCallback, this, std::placeholders::_1));

        timer_electrical_ = this->create_wall_timer(100ms, std::bind(&PowerCalculator::publishElectricalPower, this)); // Here frequency maybe faulty - check it
        timer_reactive_ = this->create_wall_timer(100ms, std::bind(&PowerCalculator::publishReactivePower, this));
    }

    void inputCallback(const digital_twin_msgs::msg::SupplyInput::SharedPtr msg)
    {
        input_current_[0] = msg->currents.current1;
        input_current_[1] = msg->currents.current2;
        input_current_[2] = msg->currents.current3;

        input_voltage_[0] = msg->voltages.voltage1;
        input_voltage_[1] = msg->voltages.voltage2;
        input_voltage_[2] = msg->voltages.voltage3;

        if(count >= SIZE_A ){
            current_buffer_ = currents_;
            voltage_buffer_ = voltages_;
            input_ready_ = true;
            voltages_.clear();
            currents_.clear();
            voltages_.resize(3);
            currents_.resize(3);
            count = 0;
        }

        voltages_.at(0).push_back(msg->voltages.voltage1);
        voltages_.at(1).push_back(msg->voltages.voltage2);
        voltages_.at(2).push_back(msg->voltages.voltage3);
        currents_.at(0).push_back(msg->currents.current1);
        currents_.at(1).push_back(msg->currents.current2);
        currents_.at(2).push_back(msg->currents.current3);

        count++;
    }

    void calculatePowerReactive()
    {
        for(int i=0;i<3;i++){
            reactive_.phase[i] = (abs(input_current_[i]*input_voltage_[i]))/3;
        }
        reactive_.total = reactive_.phase[0]+reactive_.phase[1]+reactive_.phase[2];
    }

    void calculatePowerElectrical()
    {
        calculateRMS();

        for(int i = 0; i < 3; i++){
            electrical_.phase[i] = rms_currents_[i] * rms_voltages_[i] * COS_PHI;
        }
        electrical_.total = electrical_.phase[0] + electrical_.phase[1] + electrical_.phase[2];
    }

    void publishElectricalPower()
    {
        if(input_ready_){
            calculatePowerElectrical();
            clearBuffers();
            input_ready_ = false;
        }
            powerElMsg.phase1 = electrical_.phase[0];
            powerElMsg.phase2 = electrical_.phase[1];
            powerElMsg.phase3 = electrical_.phase[2];
            powerElMsg.total = electrical_.total;
            powerElMsg.header.stamp = rclcpp::Node::now();
            PowerElectricalPublisher->publish(powerElMsg);
    }

    void publishReactivePower()
    {
        calculatePowerReactive();
        powerReactMsg.phase1 = reactive_.phase[0];
        powerReactMsg.phase2 = reactive_.phase[1];
        powerReactMsg.phase3 = reactive_.phase[2];
        powerReactMsg.total = reactive_.total;
        powerReactMsg.header.stamp = rclcpp::Node::now();
        PowerReactivePublisher->publish(powerReactMsg);
    }

    void clearBuffers()
    {
        current_buffer_.clear();
        voltage_buffer_.clear();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PowerCalculator>());
    rclcpp::shutdown();
    return 0;
}