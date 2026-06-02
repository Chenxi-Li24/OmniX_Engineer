// swerve_kinematics.cpp
#include "swerve_kinematics.h"
#include "mathutil.h"
#include <cmath>

namespace algo {
namespace {
constexpr float kRad2Deg = 180.0f / 3.14159265358979323846f;

inline float wrap_angle_deg(float deg) {
    return wrap_deg(deg);
}

inline float delta_angle_deg(float target_deg, float curr_deg) {
    return wrap_angle_deg(target_deg - curr_deg);
}
} // namespace

void solve_swerve_4wheel(float vx, float vy, float wz,
                         const float wheel_pos_x[4],
                         const float wheel_pos_y[4],
                         const float curr_theta_deg[4],
                         const uint8_t last_anti[4],
                         const SwerveConfig& cfg,
                         SwerveModuleCmd cmd_out[4])
{
    for (int i = 0; i < 4; ++i) {
        const float vxi = vx - wz * wheel_pos_y[i];
        const float vyi = vy + wz * wheel_pos_x[i];
        const float speed = ::hypotf(vxi, vyi);

        SwerveModuleCmd cmd{};
        cmd.anti = last_anti[i];

        if (speed < cfg.speed_eps) {
            cmd.theta_set_deg = wrap_angle_deg(curr_theta_deg[i]);
            cmd.wheel_mps = 0.f;
            cmd_out[i] = cmd;
            continue;
        }

        const float ang_a = wrap_angle_deg(::atan2f(vyi, vxi) * kRad2Deg);
        const float ang_b = wrap_angle_deg(ang_a - 180.f);
        const float delta_a = ::fabsf(delta_angle_deg(ang_a, curr_theta_deg[i]));
        const float delta_b = ::fabsf(delta_angle_deg(ang_b, curr_theta_deg[i]));

        if (cmd.anti == 0) {
            if ((delta_b + cfg.anti_hyst_deg) < delta_a) {
                cmd.anti = 1;
            }
        } else {
            if ((delta_a + cfg.anti_hyst_deg) < delta_b) {
                cmd.anti = 0;
            }
        }

        if (cmd.anti) {
            cmd.theta_set_deg = ang_b;
            cmd.wheel_mps = -speed;
        } else {
            cmd.theta_set_deg = ang_a;
            cmd.wheel_mps = speed;
        }

        cmd_out[i] = cmd;
    }
}

} // namespace algo
