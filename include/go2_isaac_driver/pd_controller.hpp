#pragma once
#include <vector>
#include <cmath>

class PDController
{
public:
    PDController(size_t num_joints = 12, double kp = 20.0, double kd = 0.5)
        : num_joints_(num_joints)
    {
        Kp_.assign(num_joints_, kp);
        Kd_.assign(num_joints_, kd);
        prev_error_.assign(num_joints_, 0.0);
    }

    void setGains(double kp, double kd)
    {
        std::fill(Kp_.begin(), Kp_.end(), kp);
        std::fill(Kd_.begin(), Kd_.end(), kd);
    }

    void setGains(const std::vector<double>& kp, const std::vector<double>& kd)
    {
        Kp_ = kp;
        Kd_ = kd;
    }

    std::vector<double> computeCommand(
        const std::vector<double>& q_des,
        const std::vector<double>& q_curr,
        const std::vector<double>& qd_curr,
        double dt)
    {
        std::vector<double> cmd(num_joints_, 0.0);
        if (q_des.size() != num_joints_ || q_curr.size() != num_joints_)
            return cmd;

        for (size_t i = 0; i < num_joints_; ++i)
        {
            double error = q_des[i] - q_curr[i];
            double d_error = (error - prev_error_[i]) / std::max(dt, 1e-6);
            cmd[i] = Kp_[i] * error - Kd_[i] * qd_curr[i];  // derivative term on velocity
            prev_error_[i] = error;
        }
        return cmd;
    }

private:
    size_t num_joints_;
    std::vector<double> Kp_, Kd_, prev_error_;
};
