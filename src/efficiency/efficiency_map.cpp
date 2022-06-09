/*
 * twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack
 *
 * This code is based on a similar node from the differential_drive package
 * http://wiki.ros.org/differential_drive
 * https://github.com/jfstepha/differential-drive
 */

#include <chrono>
#include <ctime>
#include <digital_twin_msgs/msg/float32_stamped.hpp>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std;
using namespace std::chrono_literals;
int ticksSinceTarget;

int closestItemIDX(vector<float> array, float item) {
  float delta = -1;
  int idx = -1;

  for (size_t i = 0; i < array.size(); i++) {
    float it = array.at(i);

    if (i == 0) {
      delta = abs(it - item);

      idx = i;
    } else if (delta > abs(it - item)) {
      delta = abs(it - item);
      idx = i;
    }

    // cout<<i<<endl;
  }
  return idx;
}
vector<float> rpm;
vector<vector<float>> torque;
vector<vector<float>> rmsEfficiency;
struct state {
  float rpm;
  float torque;
} current_state;

int rate;
int timeoutTicks;

class EfficiencyMapProcessor : public rclcpp::Node {
 public:
  string line;
  EfficiencyMapProcessor() : Node("efficiency_map") {
    /* Initalizing Parameters */
    this->declare_parameter("filename");

    try {
      filename_param = this->get_parameter("filename");
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException& e) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Failed to declare parameters for efficiency map. Perhaps you did not define "
                   "the namespace correctly?");
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Terminating...");
      exit(1);
    }

    processFile(filename_param.as_string());

    // create publishers/subscribers
    TorqueReceiver = this->create_subscription<digital_twin_msgs::msg::Float32Stamped>(
        "torque", 10, std::bind(&EfficiencyMapProcessor::getTorque, this, placeholders::_1));

    /* Shaft angular velocity of a motor in RPM */
    RPMReceiver = this->create_subscription<digital_twin_msgs::msg::Float32Stamped>(
        "angular_velocity", 10, std::bind(&EfficiencyMapProcessor::getRPM, this, placeholders::_1));

    EfficiencyControl =
        this->create_publisher<digital_twin_msgs::msg::Float32Stamped>("efficiency", 10);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&EfficiencyMapProcessor::publishEfficiency, this));  // 100ms = 10 Hz
  }

  float getEfficiency(float c_rpm, float c_torque) {
    int idx_rpm = closestItemIDX(rpm, c_rpm);
    vector<float> torque_column;

    for (auto item : torque) {
      torque_column.push_back(item.at(idx_rpm));
    }
    int idx_torque = closestItemIDX(torque_column, c_torque);
    return rmsEfficiency.at(idx_torque).at(idx_rpm);
  }

  void publishEfficiency() {
    efficiency_value.data =
        (getEfficiency(current_state.rpm, current_state.torque) / 100);  // *current_state.torque
    efficiency_value.header.stamp = rclcpp::Node::now();
    EfficiencyControl->publish(efficiency_value);
  }

 private:
  rclcpp::Parameter filename_param;

  // creating shared pointers for publishers/subscribers and messages
  rclcpp::Subscription<digital_twin_msgs::msg::Float32Stamped>::SharedPtr TorqueReceiver;
  rclcpp::Subscription<digital_twin_msgs::msg::Float32Stamped>::SharedPtr RPMReceiver;
  rclcpp::Publisher<digital_twin_msgs::msg::Float32Stamped>::SharedPtr EfficiencyControl;
  rclcpp::TimerBase::SharedPtr timer_;
  digital_twin_msgs::msg::Float32Stamped efficiency_value;

  void getTorque(const digital_twin_msgs::msg::Float32Stamped::SharedPtr msg) {
    ticksSinceTarget = 0;
    current_state.torque = msg->data;
  }

  void getRPM(const digital_twin_msgs::msg::Float32Stamped::SharedPtr msg) {
    ticksSinceTarget = 0;
    current_state.rpm = msg->data;
  }

  void processFile(std::string filename) {
    std::ifstream file(filename);
    if (!file.is_open()) throw std::runtime_error("Could not open file");

    int line_c = 0;
    int word_c = 0;

    while (std::getline(file, line)) {
      word_c = 0;
      string word = "";
      vector<float> temp_tor;
      vector<float> temp_eff;
      for (auto x : line) {
        if (x == ',') {
          if (line_c == 0 && word_c % 2 == 0) {
            // cout<<word<<endl;
            rpm.push_back(stof(word));
          } else if (line_c == 0 && word_c % 2 != 0) {
          } else {
            if (word_c % 2 == 0) {
              temp_tor.push_back(stof(word));
            } else {
              temp_eff.push_back(stof(word));
            }
          }
          word_c++;
          word = "";
        } else {
          word = word + x;
        }
      }
      if (line_c == 0 && word_c % 2 == 0) {
        rpm.push_back(stof(word));
      } else if (line_c == 0 && word_c % 2 != 0) {
      } else {
        if (word_c % 2 == 0) {
          temp_tor.push_back(stof(word));
        } else {
          temp_eff.push_back(stof(word));
        }
        torque.push_back(temp_tor);
        rmsEfficiency.push_back(temp_eff);
        word_c++;
      }
      line_c++;
    }
    cout << torque.size() << endl;
    cout << torque.size() << endl;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EfficiencyMapProcessor>());
  rclcpp::shutdown();

  return 0;
}
