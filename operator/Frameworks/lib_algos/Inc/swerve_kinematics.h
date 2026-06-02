// swerve_kinematics.h
#ifndef H723VG_V2_FREERTOS_SWERVE_KINEMATICS_H
#define H723VG_V2_FREERTOS_SWERVE_KINEMATICS_H

#pragma once
#include <cstdint>

namespace algo {

struct SwerveModuleCmd {
    float theta_set_deg;
    float wheel_mps;
    uint8_t anti;
};

struct SwerveConfig {
    float anti_hyst_deg;
    float speed_eps;
};

void solve_swerve_4wheel(float vx, float vy, float wz,
                         const float wheel_pos_x[4],
                         const float wheel_pos_y[4],
                         const float curr_theta_deg[4],
                         const uint8_t last_anti[4],
                         const SwerveConfig& cfg,
                         SwerveModuleCmd cmd_out[4]);

} // namespace algo

#endif // H723VG_V2_FREERTOS_SWERVE_KINEMATICS_H
