#include <iostream>
#include <thread>
#include <numeric>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <digital_twin_msgs/msg/current.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <array>

using namespace std::chrono_literals;

class WindingErrorChecker : public rclcpp::Node
{
public:

    std::vector<std::vector<float>> currents_;
    std::vector<std::vector<float>> currents_buffer_;
    bool can_calculate_ = false;
    std_msgs::msg::String warning_msg_;

    WindingErrorChecker() : Node("windings_checker")
    {
        std::cout << "Init happened\n";
        currents_.resize(3);
        can_calculate_ = false;
        t_phase_checker = std::thread(&WindingErrorChecker::phaseChecker, this);

        currents_listener_ = this->create_subscription<digital_twin_msgs::msg::Current>("tb_lm/input_current", 10, 
                            std::bind(&WindingErrorChecker::currentCallback, this, std::placeholders::_1));

        warning_publisher_ = this->create_publisher<std_msgs::msg::String>("diagnostics/warnings/windings", 10);
    }
    ~WindingErrorChecker()
    {
        not_terminated_ = false;
        t_phase_checker.join();
        currents_.clear();
        currents_buffer_.clear();
    }

private:

    rclcpp::Subscription<digital_twin_msgs::msg::Current>::SharedPtr currents_listener_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr warning_publisher_;

    bool not_terminated_ = true;
    std::array<float,3> rms_currents_ = {0.0, 0.0, 0.0};
    float error_coefficient_[3];
    int i = 0;
    const int SIZE_A = 5000;
    std::thread t_phase_checker;

    void currentCallback(const digital_twin_msgs::msg::Current::SharedPtr msg)
    {                
        if(i >= SIZE_A )
        {
            i = 0;
            currents_buffer_ = currents_;
            can_calculate_ = true;
            currents_.clear();
            currents_.resize(3);
        }

        currents_.at(0).push_back(msg->current1);
        currents_.at(1).push_back(msg->current2);
        currents_.at(2).push_back(msg->current3);
        i++;
    }

    void publishWarning()
    {
        warning_msg_.data = "Winding warning";
        warning_publisher_->publish(warning_msg_);
    }

    std::array<float,3> calculateRMS(std::vector<std::vector<float>> &buffer)
    {
        float mean[3];
        float square[3] = {0.0};
        std::array<float, 3> root = {0.0, 0.0, 0.0};
        /* squares */
        for(int j = 0; j<SIZE_A ; j++)
        {
            square[0] += buffer[0][j]*buffer[0][j];
            square[1] += buffer[1][j]*buffer[1][j];
            square[2] += buffer[2][j]*buffer[2][j];
        }
        for(int j = 0; j<3;j++)
        {
            mean[j] = (square[j] / SIZE_A);
        }
        for(int j = 0; j<3; j++)
        {
            root[j] = std::sqrt(mean[j]);
        }
        return root;
    }

    void phaseChecker()
    {
        bool problematic = false;

        while(true)
        {
            if(!not_terminated_) break;

            if(can_calculate_)
            {
                rms_currents_ = calculateRMS(currents_buffer_);

                for(int j = 0; j<3;j++)
                {
                    error_coefficient_[j] = rms_currents_[j]/rms_currents_[0];
                }
                for(float j : error_coefficient_)
                {
                    if( abs(error_coefficient_[0] - j) >= 0.15 ) 
                    {
                        problematic = true;
                        break;
                    }
                }
                
                if(problematic == true)
                {
                    RCLCPP_WARN(this->get_logger(), "Potential malfunction in motor windings");
                    publishWarning();
                    problematic = false;
                }

                /* CLEANUP */
                currents_buffer_.clear();
                can_calculate_ = false;
            }
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WindingErrorChecker>());
    rclcpp::shutdown();
    return 0;
}
