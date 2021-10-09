/*
 * twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack
 *
 * This code is based on a similar node from the differential_drive package
 * http://wiki.ros.org/differential_drive
 * https://github.com/jfstepha/differential-drive
 */

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <iostream>
#include <fstream>
#include <ctime>
#include <chrono>
#include <memory>
#include <functional>
#include <digital_twin_msgs/msg/float32_stamped.hpp>
#include "data_logger/data_logger.hpp"

using namespace std;
using namespace std::chrono_literals;
using namespace DataLogger;
int ticksSinceTarget;

int closestItemIDX(vector<float> array, float item)
{
    float delta=-1;
    int idx=-1;

    //cout<<array.size()<<endl;

    for (int i = 0;i<array.size();i++)
    {
        float it=array.at(i);
        if(i==0)
        {
            delta=abs(it-item);

            idx=i;
        }
        else if(delta>abs(it-item)) {

            delta = abs(it - item);
            idx=i;
        }

        //cout<<i<<endl;
    }
    return idx;
}
vector<float> rpm;
vector<vector<float>> torque;
vector<vector<float>> rmsEfficiency;
struct state{
    float rpm;
    float torque;
}current_state;
int rate;
int timeoutTicks;

class EfficiencyMapProcessor : public rclcpp::Node{
    public:
    
    string line;
    std::unique_ptr<PublisherLogger> p_efficiency_pub;
    std::unique_ptr<SubscriptionLogger> p_torque_sub;
    std::unique_ptr<SubscriptionLogger> p_rpm_sub;

    EfficiencyMapProcessor() : Node("efficiency_map")
    {
        /* Initalizing Parameters */
        this->declare_parameter("filename");
        rclcpp::Parameter filename_param = this->get_parameter("filename");

        processFile(filename_param.as_string());

        // create publishers/subscribers
        TorqueReceiver = this->create_subscription<digital_twin_msgs::msg::Float32Stamped>("torque", 10,
                            std::bind(&EfficiencyMapProcessor::getTorque, this, placeholders::_1));

        RPMReceiver = this->create_subscription<digital_twin_msgs::msg::Float32Stamped>("actual_rpm", 10,
                            std::bind(&EfficiencyMapProcessor::getRPM, this, placeholders::_1));

        EfficiencyControl = this->create_publisher<digital_twin_msgs::msg::Float32Stamped>("efficiency", 10);
        
        timer_ = this->create_wall_timer(100ms, std::bind(&EfficiencyMapProcessor::publishEfficiency, this)); // 100ms = 10 Hz
        
        p_efficiency_pub.reset(new PublisherLogger("/efficiency"));
        p_torque_sub.reset(new SubscriptionLogger("/torque"));
        p_rpm_sub.reset(new SubscriptionLogger("/actual_rpm"));

        efficiency_value.id = 0;
    }

    float getEfficiency(float c_rpm, float c_torque)
    {
        int idx_rpm = closestItemIDX(rpm,c_rpm);
        vector<float> torque_column;
        for(auto item : torque)
        {
            torque_column.push_back(item.at(idx_rpm));
        }
        int idx_torque=closestItemIDX(torque_column,c_torque);
        return rmsEfficiency.at(idx_torque).at(idx_rpm);
    }

    void publishEfficiency()
    {
        efficiency_value.data = (getEfficiency(current_state.rpm,current_state.torque)/100); // *current_state.torque
        efficiency_value.stamp = rclcpp::Node::now();
        EfficiencyControl->publish(efficiency_value);
        efficiency_value.id += 1;
        p_efficiency_pub->sent_counter += 1;
    }

    private:

    // creating shared pointers for publishers/subscribers and messages
    rclcpp::Subscription<digital_twin_msgs::msg::Float32Stamped>::SharedPtr TorqueReceiver;
    rclcpp::Subscription<digital_twin_msgs::msg::Float32Stamped>::SharedPtr RPMReceiver;
    rclcpp::Publisher<digital_twin_msgs::msg::Float32Stamped>::SharedPtr EfficiencyControl;
    rclcpp::TimerBase::SharedPtr timer_;
    digital_twin_msgs::msg::Float32Stamped efficiency_value;

    void getTorque(const digital_twin_msgs::msg::Float32Stamped::SharedPtr msg)
    {
        ticksSinceTarget = 0;
        current_state.torque=msg->data;
        if(msg->id == p_torque_sub->next_id)
        {
            p_torque_sub->recv_counter += 1;
            rclcpp::Duration diff = rclcpp::Node::now() - msg->stamp;
            p_torque_sub->time_diffs.push_back(diff.nanoseconds());
        }
        p_torque_sub->next_id = msg->id + 1;
    }

    void getRPM(const digital_twin_msgs::msg::Float32Stamped::SharedPtr msg)
    {
        ticksSinceTarget = 0;
        current_state.rpm=msg->data;
        if(msg->id == p_torque_sub->next_id)
        {
            p_rpm_sub->recv_counter += 1;
            rclcpp::Duration diff = rclcpp::Node::now() - msg->stamp;
            p_rpm_sub->time_diffs.push_back(diff.nanoseconds());
        }
        p_rpm_sub->next_id = msg->id + 1;
    }

    void processFile(std::string filename)
    {
        std::ifstream file(filename);
        if(!file.is_open())
        {
            throw std::runtime_error("Could not open file");
        }
        int line_c=0;
        int word_c=0;

        while (std::getline(file, line))
        {
            word_c=0;
            string word = "";
            vector<float> temp_tor;
            vector<float> temp_eff;
            for (auto x : line)
            {
                if (x == ',')
                {
                    if(line_c==0&&word_c%2==0)
                    {
                        //cout<<word<<endl;
                        rpm.push_back(stof(word));
                    } else if(line_c==0&&word_c%2!=0)
                    {

                    }
                    else
                    {
                        if(word_c%2==0)
                        {
                            temp_tor.push_back(stof(word));
                        }
                        else
                        {
                            temp_eff.push_back(stof(word));
                        }
                    }
                    word_c++;
                    word = "";
                }
                else
                {
                    word = word + x;
                }
            }
            if(line_c==0&&word_c%2==0)
            {
                rpm.push_back(stof(word));
            } else if(line_c==0&&word_c%2!=0)
            {
            }
            else
            {
                if(word_c%2==0)
                {
                    temp_tor.push_back(stof(word));
                }
                else
                {
                    temp_eff.push_back(stof(word));
                }
                torque.push_back(temp_tor);
                rmsEfficiency.push_back(temp_eff);
                word_c++;
            }
            line_c++;
        }
        cout<<torque.size()<<endl;
        cout<<torque.size()<<endl;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc,argv);
    auto ptr = std::make_shared<EfficiencyMapProcessor>();
    rclcpp::spin(ptr);
    DataLogger::save_logged_data("efficiency_map.csv");
    rclcpp::shutdown();

    return 0;
}
