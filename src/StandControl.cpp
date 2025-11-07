#include "go2_isaac_driver/stand.hpp"

Go2StandController::Go2StandController()
: Node("go2_stand_controller"), pd_(12, 25.0, 0.8)
{
  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu", 10, std::bind(&Go2StandController::imuCallback, this, std::placeholders::_1));

  sub_joint_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, std::bind(&Go2StandController::jointCallback, this, std::placeholders::_1));

  pub_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/go2_position_controller/commands", 10);

  client_ik_ = create_client<go2_isaac_driver::srv::ComputeIK>("/go2/compute_ik");

  timer_ = create_wall_timer(10ms, std::bind(&Go2StandController::controlLoop, this));

  RCLCPP_INFO(get_logger(), "✅ Go2 Stand Controller initialized");
}

// ---------------------------------------------------------
void Go2StandController::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  if (q.length2() < 1e-12) return;
  q.normalize();

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  roll_  = roll;
  pitch_ = pitch;
  have_imu_ = true;
}

// ---------------------------------------------------------
void Go2StandController::jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  q_curr_ = msg->position;
  qd_curr_ = msg->velocity;
  have_js_ = true;
}

// ---------------------------------------------------------
void Go2StandController::controlLoop()
{
  if (!have_imu_ || !have_js_ || q_curr_.size() != 12)
    return;

  if (!client_ik_->wait_for_service(0s))
    return;

  // Prepare IK request
  auto req = std::make_shared<go2_isaac_driver::srv::ComputeIK::Request>();
  req->body_pose = {0.0, 0.0, 0.25, -roll_, -pitch_, 0.0};  // slight body leveling

  // Feet: FR, FL, RR, RL (flattened)
  req->foot_positions_world = {
    0.25, -0.10, -0.30,   // FR
    0.25,  0.10, -0.30,   // FL
   -0.25, -0.10, -0.30,   // RR
   -0.25,  0.10, -0.30    // RL
  };

  auto future = client_ik_->async_send_request(req);
  if (rclcpp::spin_until_future_complete(get_node_base_interface(), future, 50ms)
      != rclcpp::FutureReturnCode::SUCCESS)
    return;

  auto res = future.get();
  if (!res->success || res->joint_angles.size() != 12) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
                         "IK failed: %s", res->message.c_str());
    return;
  }

  // PD control
  const double dt = 0.01;  // 100 Hz
  std::vector<double> q_des(res->joint_angles.begin(), res->joint_angles.end());
   std::vector<double> cmd = pd_.computeCommand(q_des, q_curr_, qd_curr_, dt);


  std_msgs::msg::Float64MultiArray out;
  out.data = cmd;
  pub_cmd_->publish(out);
}

// ---------------------------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2StandController>());
  rclcpp::shutdown();
  return 0;
}
