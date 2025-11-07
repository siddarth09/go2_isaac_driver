#ifndef GO2_ISAAC_DRIVER_SIT_CONTROL_HPP_
#define GO2_ISAAC_DRIVER_SIT_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "go2_isaac_driver/pd_controller.hpp"
#include "go2_isaac_driver/srv/compute_ik.hpp"

using namespace std::chrono_literals;

class Go2SitController : public rclcpp::Node
{
public:
  Go2SitController();

private:
  // ---- ROS Interfaces ----
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
  rclcpp::Client<go2_isaac_driver::srv::ComputeIK>::SharedPtr client_ik_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- Control Components ----
  PDController pd_;
  std::vector<double> q_curr_;
  std::vector<double> qd_curr_;
  double roll_{0.0};
  double pitch_{0.0};
  bool have_imu_{false};
  bool have_js_{false};

  // ---- Callbacks ----
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void controlLoop();
};

#endif  // GO2_ISAAC_DRIVER_SIT_CONTROL_HPP_
