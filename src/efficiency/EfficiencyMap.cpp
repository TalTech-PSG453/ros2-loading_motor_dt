/*
 * twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack
 *
 * This code is based on a similar node from the differential_drive package
 * http://wiki.ros.org/differential_drive
 * https://github.com/jfstepha/differential-drive
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <string>
#include <iostream>
#include <fstream>
#include <ctime>
#include <chrono>
#include <memory>
#include <functional>

using namespace std;
using namespace std::chrono_literals;
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
    EfficiencyMapProcessor() : Node("efficiency_map")
    {
        /* Initalizing Parameters */
        this->declare_parameter("filename");
        rclcpp::Parameter filename_param = this->get_parameter("filename");

        processFile(filename_param.as_string());

        // create publishers/subscribers
        TorqueReceiver = this->create_subscription<std_msgs::msg::Float32>("torque", 10,
                            std::bind(&EfficiencyMapProcessor::getTorque, this, placeholders::_1));

        RPMReceiver = this->create_subscription<std_msgs::msg::Float32>("actual_rpm", 10,
                            std::bind(&EfficiencyMapProcessor::getRPM, this, placeholders::_1));

        EfficiencyControl = this->create_publisher<std_msgs::msg::Float32>("efficiency", 10);
        
        timer_ = this->create_wall_timer(100ms, std::bind(&EfficiencyMapProcessor::publishEfficiency, this)); // 100ms = 10 Hz
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
        EfficiencyControl->publish(efficiency_value);
    }

private:

    // creating shared pointers for publishers/subscribers and messages
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr TorqueReceiver;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr RPMReceiver;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr EfficiencyControl;
    rclcpp::TimerBase::SharedPtr timer_;
    std_msgs::msg::Float32 efficiency_value;

    void getTorque(const std_msgs::msg::Float32::SharedPtr msg)
    {
        ticksSinceTarget = 0;
        current_state.torque=msg->data;
    }

    void getRPM(const std_msgs::msg::Float32::SharedPtr msg)
    {
        ticksSinceTarget = 0;
        current_state.rpm=msg->data;
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
    rclcpp::spin(std::make_shared<EfficiencyMapProcessor>());
    rclcpp::shutdown();

    return 0;
}
