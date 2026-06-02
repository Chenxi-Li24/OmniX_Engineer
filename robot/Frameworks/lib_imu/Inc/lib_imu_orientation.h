//
// Created by sirin on 2025/12/1.
//

#ifndef H723VG_V2_FREERTOS_LIB_IMU_ORIENTATION_H
#define H723VG_V2_FREERTOS_LIB_IMU_ORIENTATION_H
/*
 * IMU orientation adapter
 *
 * This module defines a 3x3 rotation matrix that transforms
 * vectors from sensor frame (S) to body frame (B):
 *
 *      v_B = R_SB * v_S
 *
 * Assumptions:
 *  - Only 90-degree rotations and axis flips are used.
 *  - Matrix elements are in {-1, 0, +1}.
 *
 * Coordinate conventions:
 *
 *  Sensor frame (S):
 *      Defined by the IMU datasheet (e.g. BMI088).
 *
 *  Body frame (B) (robot frame):
 *      +X: forward
 *      +Y: left
 *      +Z: up
 *
 * You should:
 *  - Decide what your body frame is (above is a recommendation).
 *  - Determine how the sensor frame is mounted relative to body frame.
 *  - Select or define a suitable R_SB in lib_imu_orientation.c.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct
    {
        int8_t m[3][3];  /* row-major: vB = R * vS */
    } imu_rot3_t;

    /**
     * @brief Apply orientation: v_B = R_SB * v_S
     *
     * @param R_SB  Pointer to rotation matrix (sensor -> body).
     * @param vS    Input vector in sensor frame.
     * @param vB    Output vector in body frame.
     */
    static inline void imu_apply_orientation(const imu_rot3_t *R_SB,
                                             const float vS[3],
                                             float vB[3])
    {
        /* row-major: vB = R * vS */
        vB[0] = (float)R_SB->m[0][0] * vS[0]
              + (float)R_SB->m[0][1] * vS[1]
              + (float)R_SB->m[0][2] * vS[2];

        vB[1] = (float)R_SB->m[1][0] * vS[0]
              + (float)R_SB->m[1][1] * vS[1]
              + (float)R_SB->m[1][2] * vS[2];

        vB[2] = (float)R_SB->m[2][0] * vS[0]
              + (float)R_SB->m[2][1] * vS[1]
              + (float)R_SB->m[2][2] * vS[2];
    }

    /**
     * @brief Global orientation matrix from sensor frame to body frame.
     *
     * This is defined in lib_imu_orientation.c depending on
     * IMU_ORIENTATION_SEL or your custom configuration.
     */
    extern const imu_rot3_t g_imu_R_SB;

#ifdef __cplusplus
}
#endif

#endif //H723VG_V2_FREERTOS_LIB_IMU_ORIENTATION_H