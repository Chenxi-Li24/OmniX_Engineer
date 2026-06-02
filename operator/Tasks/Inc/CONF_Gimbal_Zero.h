#ifndef H723VG_V2_FREERTOS_CONF_GIMBAL_ZERO_H
#define H723VG_V2_FREERTOS_CONF_GIMBAL_ZERO_H
#pragma once

#include <cstdint>

#include "../../Frameworks/lib_adp_dm_4310/Inc/lib_adp_dm_4310.h"
#include "../../Frameworks/lib_adp_lk_mg8016e/Inc/lib_adp_lk_mg8016e.h"

// Canonical gimbal absolute-zero macros.
// J1-J9 are the authority for startup home / pause targets / zero references.

#define GIMBAL_LK_J1_HAS_ABS_ZERO (1)
#define GIMBAL_LK_J1_ABS_ZERO_RAW (32685u)
#define GIMBAL_LK_J1_ABS_ZERO_DEG (LK8016E_EncoderRawToDeg((uint16_t)GIMBAL_LK_J1_ABS_ZERO_RAW))

#define GIMBAL_LK_J2_HAS_ABS_ZERO (1)
#define GIMBAL_LK_J2_ABS_ZERO_RAW (32631u)
#define GIMBAL_LK_J2_ABS_ZERO_DEG (LK8016E_EncoderRawToDeg((uint16_t)GIMBAL_LK_J2_ABS_ZERO_RAW))

#define GIMBAL_LK_J3_HAS_ABS_ZERO (1)
#define GIMBAL_LK_J3_ABS_ZERO_RAW (16507u)
#define GIMBAL_LK_J3_ABS_ZERO_DEG (LK8016E_EncoderRawToDeg((uint16_t)GIMBAL_LK_J3_ABS_ZERO_RAW))

#define GIMBAL_DM_J4_HAS_ABS_ZERO (1)
#define GIMBAL_DM_J4_ABS_ZERO_RAW (48977u)
#define GIMBAL_DM_J4_ABS_ZERO_RAD (DM4310_PosRawToRad((uint16_t)GIMBAL_DM_J4_ABS_ZERO_RAW))

#define GIMBAL_DM_J5_HAS_ABS_ZERO (1)
#define GIMBAL_DM_J5_ABS_ZERO_RAW (33585u)
#define GIMBAL_DM_J5_ABS_ZERO_RAD (DM4310_PosRawToRad((uint16_t)GIMBAL_DM_J5_ABS_ZERO_RAW))

#define GIMBAL_DM_J6_HAS_ABS_ZERO (1)
#define GIMBAL_DM_J6_ABS_ZERO_RAW (62893u)
#define GIMBAL_DM_J6_ABS_ZERO_RAD (DM4310_PosRawToRad((uint16_t)GIMBAL_DM_J6_ABS_ZERO_RAW))

#define GIMBAL_DM_J7_HAS_ABS_ZERO (1)
#define GIMBAL_DM_J7_ABS_ZERO_RAW (60961u)
#define GIMBAL_DM_J7_ABS_ZERO_RAD (DM4310_PosRawToRad((uint16_t)GIMBAL_DM_J7_ABS_ZERO_RAW))

#define GIMBAL_DM_J8_HAS_ABS_ZERO (1)
#define GIMBAL_DM_J8_ABS_ZERO_RAW (6926u)
#define GIMBAL_DM_J8_ABS_ZERO_RAD (DM4310_PosRawToRad((uint16_t)GIMBAL_DM_J8_ABS_ZERO_RAW))

#define GIMBAL_DM_J9_HAS_ABS_ZERO (1)
#define GIMBAL_DM_J9_ABS_ZERO_RAW (57165u)
#define GIMBAL_DM_J9_ABS_ZERO_RAD (DM4310_PosRawToRad((uint16_t)GIMBAL_DM_J9_ABS_ZERO_RAW))

#endif // H723VG_V2_FREERTOS_CONF_GIMBAL_ZERO_H