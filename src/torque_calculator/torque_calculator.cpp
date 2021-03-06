#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <chrono>
#include <digital_twin_msgs/msg/float32_stamped.hpp>
#include <digital_twin_msgs/msg/power.hpp>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace message_filters;

/* Node that calculates electrical and mechanical torque of a shaft
 *  based on efficiency, power of the motor and angular velocity at a moment of time
 *  It will work only when these three topics are being published onto, otherwise
 *  Node cannot calculate the output torques. Calculation for mechanical torque
 *  is based on electrical torque reference and efficiency of the motor.
 *  Since efficiency, power and angular velocity are coming from 3 different sources,
 *  Approximate Time Synchronizer is used that compares timestamps in the Headers of msgs
 *  publishing and torque calculation are happening asynchronously. Locks are avoided by use of
 *  a mutex_.
 */

class TorqueCalculator : public rclcpp::Node {
 private:
  float electrical_power_ = 0;
  float efficiency_ = 0;
  float angular_velocity_ = 0;
  float mechanical_torque_ = 0;
  float electrical_torque_ref_ = 0;
  const float k_rads_in_rpm = 0.104719755;

  std::mutex mutex_;

  float calculate_electrical_torque_ref() { return (electrical_power_ / angular_velocity_); }
  float calculate_mechanical_torque() { return (electrical_torque_ref_ * efficiency_); }

  // Shared Pointers with ROS methods
  Subscriber<digital_twin_msgs::msg::Float32Stamped> angularVelocityReceiver;
  Subscriber<digital_twin_msgs::msg::Power> powerReceiver;
  Subscriber<digital_twin_msgs::msg::Float32Stamped> efficiencyReceiver;
  rclcpp::Publisher<digital_twin_msgs::msg::Float32Stamped>::SharedPtr electricalTorquePublisher;
  rclcpp::Publisher<digital_twin_msgs::msg::Float32Stamped>::SharedPtr mechanicalTorquePublisher;

  // ApproximateTime Policy synchronizer definitions. This is required to approximately synchronize
  // the incoming data from 2  different topics for further calculations. In theory, should increase
  // accuracy. More info below: http://wiki.ros.org/message_filters

  typedef sync_policies::ApproximateTime<digital_twin_msgs::msg::Power,
                                         digital_twin_msgs::msg::Float32Stamped,
                                         digital_twin_msgs::msg::Float32Stamped>
      ApproxSyncPolicy;
  std::shared_ptr<Synchronizer<ApproxSyncPolicy>> msg_sync_;
  rclcpp::TimerBase::SharedPtr timer_;

  digital_twin_msgs::msg::Float32Stamped electrical_torque_msg_;
  digital_twin_msgs::msg::Float32Stamped mechanical_torque_msg_;

 public:
  TorqueCalculator() : Node("torque_calculator") {
    // Subscribers
    powerReceiver.subscribe(this, "motor_power/electrical_power");
    angularVelocityReceiver.subscribe(
        this, "angular_velocity");  // shaft angular velocity of a motor in RPM
    efficiencyReceiver.subscribe(this, "efficiency");

    // The way synchronizer is implemented can be found below:
    // https://answers.ros.org/question/366440/ros-2-message_filters-timesynchronizer-minimal-example-does-not-reach-callback-function/

    msg_sync_ = std::make_shared<Synchronizer<ApproxSyncPolicy>>(
        ApproxSyncPolicy(100), powerReceiver, angularVelocityReceiver, efficiencyReceiver);
    msg_sync_->registerCallback(std::bind(&TorqueCalculator::sync_callback, this,
                                          std::placeholders::_1, std::placeholders::_2,
                                          std::placeholders::_3));

    electrical_torque_msg_.data = 0.0;
    mechanical_torque_msg_.data = 0.0;

    /* Publishers */

    electricalTorquePublisher =
        this->create_publisher<digital_twin_msgs::msg::Float32Stamped>("electrical_torque_ref", 10);
    mechanicalTorquePublisher =
        this->create_publisher<digital_twin_msgs::msg::Float32Stamped>("mechanical_torque", 10);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&TorqueCalculator::publish_torques, this));  // 100ms = 10 Hz
  }

  // In order to avoid racing conditions in callback and publisher, mutexes are used to ensure only
  // one method is reading/writing to variables at a time

  void publish_torques() {
    const std::lock_guard<std::mutex> lock(mutex_);
    electrical_torque_msg_.data = electrical_torque_ref_;
    mechanical_torque_msg_.data = mechanical_torque_;
    electrical_torque_msg_.header.stamp = rclcpp::Node::now();
    mechanical_torque_msg_.header.stamp = rclcpp::Node::now();
    electricalTorquePublisher->publish(electrical_torque_msg_);
    mechanicalTorquePublisher->publish(mechanical_torque_msg_);
  }

  float convert_to_rads(float rpm_value) { return rpm_value * k_rads_in_rpm; }

  void sync_callback(const digital_twin_msgs::msg::Power::ConstSharedPtr& msg1,
                     const digital_twin_msgs::msg::Float32Stamped::ConstSharedPtr& msg2,
                     const digital_twin_msgs::msg::Float32Stamped::ConstSharedPtr& msg3) {
    electrical_power_ = msg1->total;
    angular_velocity_ = msg2->data;
    efficiency_ = convert_to_rads(msg3->data);
    const std::lock_guard<std::mutex> lock(mutex_);
    electrical_torque_ref_ = calculate_electrical_torque_ref();
    mechanical_torque_ = calculate_mechanical_torque();
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TorqueCalculator>());
  rclcpp::shutdown();
  return 0;
}