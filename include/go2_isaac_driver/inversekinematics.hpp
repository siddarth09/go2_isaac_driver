#pragma once
#include <array>
#include <cmath>
#include <vector>

class InverseKinematics {
public:
  InverseKinematics(double body_length, double body_width,
                    double l1, double l2, double l3, double l4)
      : bodyLength_(body_length),
        bodyWidth_(body_width),
        l1_(l1), l2_(l2), l3_(l3), l4_(l4) {}

  // main API
  std::vector<double> compute(const std::array<double, 12> &leg_positions,
                              double dx, double dy, double dz,
                              double roll, double pitch, double yaw) {
    // leg_positions is 4x3 flattened order: FR, FL, RR, RL
    // Output: 12 joint angles (3 per leg)
    std::vector<double> angles;
    angles.reserve(12);

    std::array<std::array<double, 3>, 4> pos{};
    for (size_t i = 0; i < 4; ++i) {
      pos[i] = {leg_positions[3 * i + 0],
                leg_positions[3 * i + 1],
                leg_positions[3 * i + 2]};
    }

    int cnt = 0;
    for (size_t i = 0; i < 4; ++i) {
      double x = pos[i][0];
      double y = pos[i][1];
      double z = pos[i][2];

      double F = std::sqrt(x * x + y * y - l2_ * l2_);
      double G = F - l1_;
      double H = std::sqrt(G * G + z * z);

      double theta1 =
          -std::atan2(y, x) - std::atan2(F, l2_ * std::pow(-1, static_cast<int>(i)));

      double D = (H * H - l3_ * l3_ - l4_ * l4_) / (2.0 * l3_ * l4_);
      if (D < -1.0)
        D = -1.0;
      else if (D > 1.0)
        D = 1.0;

      double theta4 = std::acos(-D) - M_PI;
      double theta3 = std::atan2(z, G) -
                      std::atan2(l4_ * std::sin(theta4),
                                 l3_ + l4_ * std::cos(theta4));

      if (cnt == 0 || cnt == 2)
        theta1 = -theta1;

      angles.push_back(theta1);
      angles.push_back(theta3);
      angles.push_back(theta4);
      cnt++;
    }
    return angles;
  }

private:
  double bodyLength_;
  double bodyWidth_;
  double l1_, l2_, l3_, l4_;
};
