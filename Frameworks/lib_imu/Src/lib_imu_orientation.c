//
// Created by sirin on 2025/12/1.
//
#include "lib_imu_orientation.h"

//请在lib_imu的CMakeLists里选择安装方向 0为PCB上原始标记方向 1为XT30朝上在左方向

/*
 * IMU orientation presets
 *
 * All presets define a rotation matrix R_SB such that:
 *
 *      v_B = R_SB * v_S
 *
 * where:
 *      S: sensor frame (IMU datasheet axes)
 *      B: body frame (robot frame)
 *
 * Body frame convention (recommended):
 *      +X_B: forward
 *      +Y_B: left
 *      +Z_B: up
 *
 * Note: All matrices are row-major:
 *      [ Bx ]   [ r00 r01 r02 ] [ Sx ]
 *      [ By ] = [ r10 r11 r12 ] [ Sy ]
 *      [ Bz ]   [ r20 r21 r22 ] [ Sz ]
 *
 * Each row must contain exactly one of {-1, +1}, other elements 0.
 *
 * Select orientation via IMU_ORIENTATION_SEL macro (e.g. in CMake):
 *      add_compile_definitions(IMU_ORIENTATION_SEL=1)
 *
 * Or define IMU_ORIENTATION_SEL in some global config header.
 */

#ifndef IMU_ORIENTATION_SEL
/* Default: identity (sensor frame == body frame) */
#define IMU_ORIENTATION_SEL 0
#endif

/*
 * Helper macro to define an imu_rot3_t in a compact form.
 */
#define IMU_ROT3( r00, r01, r02, \
                  r10, r11, r12, \
                  r20, r21, r22 ) \
    { .m = { { (int8_t)(r00), (int8_t)(r01), (int8_t)(r02) }, \
             { (int8_t)(r10), (int8_t)(r11), (int8_t)(r12) }, \
             { (int8_t)(r20), (int8_t)(r21), (int8_t)(r22) } } }


/* ================================
 *  Preset 0: Identity (B == S)
 * ================================
 *
 *  Bx = +Sx
 *  By = +Sy
 *  Bz = +Sz
 */
#if (IMU_ORIENTATION_SEL == 0)

const imu_rot3_t g_imu_R_SB = IMU_ROT3(
    +1,  0,  0,   /* Bx */
     0, +1,  0,   /* By */
     0,  0, +1    /* Bz */
);


/* ========================================
 *  Preset 1: IMU flipped around X axis
 * ========================================
 *
 *  Mounting:
 *      Sensor is rotated 180 deg around its X axis.
 *
 *  Mapping:
 *      Bx = +Sx
 *      By = -Sy
 *      Bz = -Sz
 */
#elif (IMU_ORIENTATION_SEL == 1)

const imu_rot3_t g_imu_R_SB = IMU_ROT3(
    +1,  0,  0,   /* Bx = +Sx */
     0, -1,  0,   /* By = -Sy */
     0,  0, -1    /* Bz = -Sz */
);


/* ========================================
 *  Preset 2: IMU rotated +90 deg around Z
 * ========================================
 *
 *  Mounting:
 *      Sensor is rotated +90 deg around its Z axis
 *      (right-hand rule).
 *
 *  Mapping example:
 *      Bx = +Sy
 *      By = -Sx
 *      Bz =  +Sz
 */
#elif (IMU_ORIENTATION_SEL == 2)

const imu_rot3_t g_imu_R_SB = IMU_ROT3(
     0, +1,  0,   /* Bx = +Sy */
    -1,  0,  0,   /* By = -Sx */
     0,  0, +1    /* Bz = +Sz */
);


/* ========================================
 *  Preset 3: IMU rotated -90 deg around Z
 * ========================================
 *
 *  Mapping example:
 *      Bx = -Sy
 *      By = +Sx
 *      Bz =  +Sz
 */
#elif (IMU_ORIENTATION_SEL == 3)

const imu_rot3_t g_imu_R_SB = IMU_ROT3(
     0, -1,  0,   /* Bx = -Sy */
    +1,  0,  0,   /* By = +Sx */
     0,  0, +1    /* Bz = +Sz */
);


/* ========================================
 *  Preset 4: IMU rotated +90 deg around Y
 * ========================================
 *
 *  Example mapping:
 *      Bx = +Sz
 *      By = +Sy
 *      Bz = -Sx
 */
#elif (IMU_ORIENTATION_SEL == 4)

const imu_rot3_t g_imu_R_SB = IMU_ROT3(
     0,  0, +1,   /* Bx = +Sz */
     0, +1,  0,   /* By = +Sy */
    -1,  0,  0    /* Bz = -Sx */
);


/* ========================================
 *  Preset 5: IMU rotated -90 deg around Y
 * ========================================
 *
 *  Example mapping:
 *      Bx = -Sz
 *      By = +Sy
 *      Bz = +Sx
 */
#elif (IMU_ORIENTATION_SEL == 5)

const imu_rot3_t g_imu_R_SB = IMU_ROT3(
     0,  0, -1,   /* Bx = -Sz */
     0, +1,  0,   /* By = +Sy */
    +1,  0,  0    /* Bz = +Sx */
);


/* ========================================
 *  Custom / fallback
 * ========================================
 *
 *  If none of the presets fits your mounting,
 *  you can define IMU_ORIENTATION_SEL to a
 *  custom value and implement the matrix here,
 *  or simply replace this whole #elif block
 *  with your own R_SB.
 */

#elif (IMU_ORIENTATION_SEL == 6)
// XT30 Upwards and Forwards
const imu_rot3_t g_imu_R_SB = IMU_ROT3(
     0, -1,  0,   /* Bx = -Sy */
    -1,  0,  0,   /* By = -Sx */
     0,  0, -1    /* Bz = -Sz */
);
#else

#warning "IMU_ORIENTATION_SEL has no matching preset, using identity as fallback. Please define your own R_SB."

const imu_rot3_t g_imu_R_SB = IMU_ROT3(
    +1,  0,  0,
     0, +1,  0,
     0,  0, +1
);

#endif /* IMU_ORIENTATION_SEL */

#undef IMU_ROT3