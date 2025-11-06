#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "go2_isaac_driver/inversekinematics.hpp"
#include "go2_isaac_driver/srv/compute_ik.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class Go2IKService : public rclcpp::Node {
public:
  Go2IKService()
      : Node("go2_ik_service"),
        ik_(declare_parameter("body_length", 0.559),
            declare_parameter("body_width", 0.120),
            declare_parameter("l1", 0.0),
            declare_parameter("l2", 0.1425),
            declare_parameter("l3", 0.426),
            declare_parameter("l4", 0.345)) {

    srv_ = create_service<go2_isaac_driver::srv::ComputeIK>(
        "/go2/compute_ik",
        std::bind(&Go2IKService::handle_request, this, _1, _2));

    RCLCPP_INFO(get_logger(),
                "Go2 IK Service ready (body %.3f×%.3f, legs %.3f %.3f %.3f %.3f)",
                get_parameter("body_length").as_double(),
                get_parameter("body_width").as_double(),
                get_parameter("l1").as_double(),
                get_parameter("l2").as_double(),
                get_parameter("l3").as_double(),
                get_parameter("l4").as_double());
  }

private:
  void handle_request(
      const std::shared_ptr<go2_isaac_driver::srv::ComputeIK::Request> request,
      std::shared_ptr<go2_isaac_driver::srv::ComputeIK::Response> response) {

    try {
      if (request->body_pose.size() != 6) {
        response->success = false;
        response->message = "body_pose must have 6 elements";
        return;
      }
      if (request->foot_positions_world.size() != 12) {
        response->success = false;
        response->message = "foot_positions_world must have 12 elements";
        return;
      }

      std::array<double, 12> legs{};
      for (size_t i = 0; i < 12; ++i)
        legs[i] = request->foot_positions_world[i];

      const auto &bp = request->body_pose;
      auto result = ik_.compute(legs, bp[0], bp[1], bp[2],
                                bp[3], bp[4], bp[5]);

      if (result.size() != 12) {
        response->success = false;
        response->message = "Invalid IK result size";
        return;
      }

      for (size_t i = 0; i < 12 && i < result.size(); ++i){
        response->joint_angles[i] = result[i];
      }

      response->success = true;
      response->message = "ok";
    } catch (const std::exception &e) {
      response->success = false;
      response->message = e.what();
      RCLCPP_ERROR(get_logger(), "IK Exception: %s", e.what());
    }
  }

  InverseKinematics ik_;
  rclcpp::Service<go2_isaac_driver::srv::ComputeIK>::SharedPtr srv_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2IKService>());
  rclcpp::shutdown();
  return 0;
}
