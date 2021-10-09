#include "rclcpp/rclcpp.hpp"
#include <stdlib.h>
#include <chrono>
#include <memory>
#include <functional>
#include <digital_twin_msgs/msg/current.hpp>
#include <digital_twin_msgs/msg/voltage.hpp>
#include <digital_twin_msgs/msg/power.hpp>

#include "data_logger/data_logger.hpp"

using namespace std::chrono_literals;
using namespace DataLogger;

 struct powerValues
{
    float phase[3] = {0.0};
    float total = 0;
};

class PowerCalculator : public rclcpp::Node
{
private:
    //bool canCalculate = false;
    float input_current_[3];
    float input_voltage_[3];
    float mean_[2][3] ={0.0};
    float square_[2][3] = {0.0};
    std::vector<std::vector<float>> currents_;
    std::vector<std::vector<float>> voltages_;
    std::vector<std::vector<float>> current_buffer_;
    std::vector<std::vector<float>> voltage_buffer_;
    const float COS_PHI = 0.84;
    const int SIZE_A = 5000;
    /* counters for calculating RMS values */
    int c  = 0;
    int v = 0;
    float rms_currents_[3] = {0.0};
    float rms_voltages_[3] = {0.0};
    bool voltage_ready_ = false;
    bool current_ready_ = false;
    powerValues reactive_;
    powerValues electrical_;

    /* Declaration of publisher and subscriber shared pointers */

    rclcpp::Subscription<digital_twin_msgs::msg::Voltage>::SharedPtr voltageSubscriber;
    rclcpp::Subscription<digital_twin_msgs::msg::Current>::SharedPtr currentSubscriber;
    rclcpp::Publisher<digital_twin_msgs::msg::Power>::SharedPtr PowerElectricalPublisher;
    rclcpp::Publisher<digital_twin_msgs::msg::Power>::SharedPtr PowerReactivePublisher;
    rclcpp::TimerBase::SharedPtr timer_electrical_;
    rclcpp::TimerBase::SharedPtr timer_reactive_;
    digital_twin_msgs::msg::Power powerReactMsg;
    digital_twin_msgs::msg::Power powerElMsg;

    void calculateRMS()
    {
        /* square_ */
        for (int j = 0; j < SIZE_A; j++)
        {
            square_[0][0] += current_buffer_[0][j] * current_buffer_[0][j];
            square_[0][1] += current_buffer_[1][j] * current_buffer_[1][j];
            square_[0][2] += current_buffer_[2][j] * current_buffer_[2][j];

            square_[1][0] += voltage_buffer_[0][j] * voltage_buffer_[0][j];
            square_[1][1] += voltage_buffer_[1][j] * voltage_buffer_[1][j];
            square_[1][2] += voltage_buffer_[2][j] * voltage_buffer_[2][j];
        }
        for (int j = 0; j < 3; j++)
        {
            mean_[0][j] = (square_[0][j] / SIZE_A);
            mean_[1][j] = (square_[1][j] / SIZE_A);
        }
        for (int j = 0; j < 3; j++)
        {
            rms_currents_[j] = std::sqrt(mean_[0][j]);
            rms_voltages_[j] = std::sqrt(mean_[1][j]);
        }
        for(int i=0;i<2;i++)
        {
            for(int j=0; j<3; j++)
            {
                square_[i][j] = 0;
            }
        }
    }

public:

    std::unique_ptr<PublisherLogger> p_electrical_power_pub;
    std::unique_ptr<PublisherLogger> p_reactive_power_pub;

    std::unique_ptr<SubscriptionLogger> p_current_sub;
    std::unique_ptr<SubscriptionLogger> p_voltage_sub;

    PowerCalculator() : Node("power_calculator")
    {

        currents_.resize(3);
        voltages_.resize(3);

        PowerReactivePublisher = this->create_publisher<digital_twin_msgs::msg::Power>("motor_power/reactive_power", 100);
        PowerElectricalPublisher = this->create_publisher<digital_twin_msgs::msg::Power>("motor_power/electrical_power", 100);
        voltageSubscriber = this->create_subscription<digital_twin_msgs::msg::Voltage>("input_voltage", 100, 
                                                    std::bind(&PowerCalculator::voltageCallback, this, std::placeholders::_1));
        currentSubscriber = this->create_subscription<digital_twin_msgs::msg::Current>("input_current", 100,
                                                    std::bind(&PowerCalculator::currentCallback, this, std::placeholders::_1));

        timer_electrical_ = this->create_wall_timer(100ms, std::bind(&PowerCalculator::publishElectricalPower, this)); // Here frequency maybe faulty - check it
        timer_reactive_ = this->create_wall_timer(100ms, std::bind(&PowerCalculator::publishReactivePower, this));

        p_electrical_power_pub.reset(new PublisherLogger("/motor_power/electrical_power"));
        p_reactive_power_pub.reset(new PublisherLogger("/motor_power/reactive_power"));
        p_current_sub.reset(new SubscriptionLogger("/input_current"));
        p_voltage_sub.reset(new SubscriptionLogger("/input_voltage"));

        powerReactMsg.id = 0;
        powerElMsg.id = 0;
    }

    void currentCallback(const digital_twin_msgs::msg::Current::SharedPtr msg)
    {
        input_current_[0] = msg->current1;
        input_current_[1] = msg->current2;
        input_current_[2] = msg->current3;
        if(msg->id == p_current_sub->next_id)
        {
            p_current_sub->recv_counter += 1;
            rclcpp::Duration diff = rclcpp::Node::now() - msg->stamp;
            p_current_sub->time_diffs.push_back(diff.nanoseconds());
        }
        p_current_sub->next_id = msg->id + 1;

        if(c >= SIZE_A )
        {
            current_buffer_ = currents_;
            current_ready_ = true;
            currents_.clear();
            currents_.resize(3);
            c = 0;
        }

        currents_.at(0).push_back(msg->current1);
        currents_.at(1).push_back(msg->current2);
        currents_.at(2).push_back(msg->current3);

        c++;
    }
    void voltageCallback(const digital_twin_msgs::msg::Voltage::SharedPtr msg)
    {
        input_voltage_[0] = msg->voltage1;
        input_voltage_[1] = msg->voltage2;
        input_voltage_[2] = msg->voltage3;
        if(msg->id == p_voltage_sub->next_id)
        {   p_voltage_sub->recv_counter += 1;
            rclcpp::Duration diff = rclcpp::Node::now() - msg->stamp;
            p_voltage_sub->time_diffs.push_back(diff.nanoseconds());
        }
        p_voltage_sub->next_id = msg->id + 1;
        if(v >= SIZE_A )
        {
            voltage_buffer_ = voltages_;
            voltage_ready_ = true;
            voltages_.clear();
            voltages_.resize(3);
            v = 0;
        }

        voltages_.at(0).push_back(msg->voltage1);
        voltages_.at(1).push_back(msg->voltage2);
        voltages_.at(2).push_back(msg->voltage3);

        v++;
    }
    void calculatePowerReactive()
    {
        for(int i=0;i<3;i++)
        {
            reactive_.phase[i] = (abs(input_current_[i]*input_voltage_[i]))/3;
        }
        reactive_.total = reactive_.phase[0]+reactive_.phase[1]+reactive_.phase[2];
        //return totalPower;
    }
    bool calculatePowerElectrical()
    {
        /* BOOL to confirm calculation and not send bullcrap?*/
        if(current_ready_ && voltage_ready_)
        {
            calculateRMS();
            electrical_.phase[0] = rms_currents_[0] * rms_voltages_[0] * COS_PHI;
            electrical_.phase[1] = rms_currents_[1] * rms_voltages_[1] * COS_PHI;
            electrical_.phase[2] = rms_currents_[2] * rms_voltages_[2] * COS_PHI;
            electrical_.total = electrical_.phase[0] + electrical_.phase[1] + electrical_.phase[2];
            return true;
        }
        // Is this really needed?
        else
        {
            return false;
        }
    }

    void publishElectricalPower()
    {
        if(calculatePowerElectrical())
        {
            powerElMsg.phase1 = electrical_.phase[0];
            powerElMsg.phase2 = electrical_.phase[1];
            powerElMsg.phase3 = electrical_.phase[2];
            powerElMsg.total = electrical_.total;
            clearBuffers();
            powerElMsg.stamp = rclcpp::Node::now();
            PowerElectricalPublisher->publish(powerElMsg);
            powerElMsg.id += 1;
            p_electrical_power_pub->sent_counter +=1;
            setCurrentReady(false);
            setVoltageReady(false);
           // setCanCalculate(false);
        }
    }

    void publishReactivePower()
    {
        calculatePowerReactive();
        powerReactMsg.phase1 = reactive_.phase[0];
        powerReactMsg.phase2 = reactive_.phase[1];
        powerReactMsg.phase3 = reactive_.phase[2];
        powerReactMsg.total = reactive_.total;
        powerReactMsg.stamp = rclcpp::Node::now();
        PowerReactivePublisher->publish(powerReactMsg);
        powerReactMsg.id += 1;
        p_reactive_power_pub->sent_counter += 1;
    }

    void clearBuffers()
    {
        current_buffer_.clear();
        voltage_buffer_.clear();
    }
    void setCurrentReady(bool val)
    {
        current_ready_ = val;
    }
    void setVoltageReady(bool val)
    {
        voltage_ready_ = val;
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto ptr = std::make_shared<PowerCalculator>();
    rclcpp::spin(ptr);
    DataLogger::save_logged_data("power_calculator.csv");
    rclcpp::shutdown();
    return 0;
}