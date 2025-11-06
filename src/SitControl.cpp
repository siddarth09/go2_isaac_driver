#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

#include "go2_isaac_driver/srv/compute_ik.hpp"
#include "go2_isaac_driver/pd_controller.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class Go2SitController : public rclcpp::Node
{
public:
    Go2SitController()
    : Node("go2_sit_controller"),
      pd_(12, 25.0, 0.8)
    {
        // ---- ROS Interfaces ----
        sub_imu_   = create_subscription<geometry_msgs::msg::Imu>(
            "/imu", 10, std::bind(&Go2SitController::imuCallback, this, _1));
        sub_joint_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&Go2SitController::jointCallback, this, _1));
        pub_cmd_   = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/go2_position_controller/commands", 10);
        client_ik_ = create_client<go2_isaac_driver::srv::ComputeIK>("/go2/compute_ik");

        // ---- Control timer ----
        timer_ = create_wall_timer(10ms, std::bind(&Go2SitController::controlLoop, this));

        RCLCPP_INFO(get_logger(), "✅ Go2 Sit Controller initialized");
    }

private:
    // ROS Interfaces
    rclcpp::Subscription<geometry_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
    rclcpp::Client<go2_isaac_driver::srv::ComputeIK>::SharedPtr client_ik_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Control components
    PDController pd_;
    std::vector<double> q_curr_, qd_curr_;
    double roll_ = 0.0, pitch_ = 0.0;

    // ---------------------------------------------------------
    void imuCallback(const geometry_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                          msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll_, pitch_, std::ignore);
    }

    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        q_curr_ = msg->position;
        qd_curr_ = msg->velocity;
    }

    // ---------------------------------------------------------
    void controlLoop()
    {
        if (q_curr_.size() != 12 || !client_ik_->wait_for_service(0s))
            return;

        // Adjust body pose slightly for balance in sit mode
        double body_roll = -0.5 * roll_;
        double body_pitch = -0.5 * pitch_;

        auto req = std::make_shared<go2_isaac_driver::srv::ComputeIK::Request>();
        req->body_pose.position.x = 0.0;
        req->body_pose.position.y = 0.0;
        req->body_pose.position.z = 0.18;  // lower sit height
        req->body_pose.orientation = tf2::toMsg(
            tf2::Quaternion(tf2::Vector3(1,0,0), body_roll) *
            tf2::Quaternion(tf2::Vector3(0,1,0), body_pitch));

        // Sit foot positions (feet closer and higher)
        req->foot_positions = {
            { 0.15,  0.10, -0.20},
            { 0.15, -0.10, -0.20},
            {-0.15,  0.10, -0.20},
            {-0.15, -0.10, -0.20}
        };

        auto future = client_ik_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(get_node_base_interface(), future, 20ms)
            != rclcpp::FutureReturnCode::SUCCESS)
            return;

        auto res = future.get();
        std::vector<double> q_des = res->joint_angles;
        if (q_des.size() != 12) return;

        // ---- PD control ----
        double dt = 0.01;  // 100 Hz
        std::vector<double> cmd = pd_.computeCommand(q_des, q_curr_, qd_curr_, dt);

        // ---- Publish ----
        std_msgs::msg::Float64MultiArray out;
        out.data = cmd;
        pub_cmd_->publish(out);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Go2SitController>());
    rclcpp::shutdown();
    return 0;
}
