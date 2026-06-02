#ifndef OMNIX_SERVO_JOINT_ANGLE_PROTOCOL_H
#define OMNIX_SERVO_JOINT_ANGLE_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OMNIX_SERVO_JOINT_COUNT               7u
#define OMNIX_JOINT_RAW_COUNT                 OMNIX_SERVO_JOINT_COUNT
#define OMNIX_SERVO_JOINT_PROTOCOL_VERSION    2u
#define OMNIX_CUSTOM_CONTROLLER_PROTOCOL_VERSION OMNIX_SERVO_JOINT_PROTOCOL_VERSION
#define OMNIX_SERVO_JOINT_RX_TIMEOUT_MS       200u
#define OMNIX_CUSTOM_CONTROLLER_RX_TIMEOUT_MS OMNIX_SERVO_JOINT_RX_TIMEOUT_MS
#define OMNIX_REFEREE_CUSTOM_DATA_LEN         30u
#define OMNIX_ROBOT_JOINT_FEEDBACK_VERSION    2u
#define OMNIX_CONTROLLER_SERVO_SPAN_CODE      1000.0f
#define OMNIX_CONTROLLER_SERVO_SPAN_DEG       240.0f
#define OMNIX_CONTROLLER_SERVO_DEG_PER_CODE \
    (OMNIX_CONTROLLER_SERVO_SPAN_DEG / OMNIX_CONTROLLER_SERVO_SPAN_CODE)
#define OMNIX_CONTROLLER_RAW_DEGENERATE_SPAN_WARN_RAW 20u

#define OMNIX_CTRL_J1_RAW_MIN  1u
#define OMNIX_CTRL_J1_RAW_ZERO 493u
#define OMNIX_CTRL_J1_RAW_MAX  1000u

#define OMNIX_CTRL_J2_RAW_MIN  260u
#define OMNIX_CTRL_J2_RAW_ZERO 717u
#define OMNIX_CTRL_J2_RAW_MAX  717u

#define OMNIX_CTRL_J3_RAW_MIN  215u
#define OMNIX_CTRL_J3_RAW_ZERO 674u
#define OMNIX_CTRL_J3_RAW_MAX  674u

#define OMNIX_CTRL_J4_RAW_MIN  85u
#define OMNIX_CTRL_J4_RAW_ZERO 468u
#define OMNIX_CTRL_J4_RAW_MAX  838u

#define OMNIX_CTRL_J5_RAW_MIN  37u
#define OMNIX_CTRL_J5_RAW_ZERO 533u
#define OMNIX_CTRL_J5_RAW_MAX  1082u

#define OMNIX_CTRL_J6_RAW_MIN  1u
#define OMNIX_CTRL_J6_RAW_ZERO 251u
#define OMNIX_CTRL_J6_RAW_MAX  622u

#define OMNIX_CTRL_J7_RAW_MIN  149u
#define OMNIX_CTRL_J7_RAW_ZERO 286u
#define OMNIX_CTRL_J7_RAW_MAX  520u

#define OMNIX_DIAG_FLAG_REMOTE_BUZZER_REQ                  ((uint8_t)(1u << 0))
#define OMNIX_DIAG_FLAG_MAPPING_RESET_REQ                  ((uint8_t)(1u << 1))
#define OMNIX_DIAG_FLAG_MAPPING_RESET_DONE                 ((uint8_t)(1u << 2))
#define OMNIX_JOINT_FEEDBACK_FLAG_EXTERNAL_CONTROL_ENABLED ((uint8_t)(1u << 0))
#define OMNIX_JOINT_FEEDBACK_FLAG_PAUSE_ACTIVE             ((uint8_t)(1u << 1))
#define OMNIX_JOINT_FEEDBACK_FLAG_ZERO_FORCE               ((uint8_t)(1u << 2))
#define OMNIX_JOINT_FEEDBACK_FLAG_GIMBAL_ALL_ONLINE        ((uint8_t)(1u << 3))

#if defined(__GNUC__)
#define OMNIX_PACKED __attribute__((packed))
#else
#define OMNIX_PACKED
#endif

typedef struct {
    uint16_t raw_min;
    uint16_t raw_zero;
    uint16_t raw_max;
} omnix_controller_joint_raw_calib_t;

typedef struct OMNIX_PACKED {
    uint8_t version;
    uint8_t flags;
    uint16_t seq;
    uint8_t valid_mask;
    union {
        uint16_t raw_u16[OMNIX_SERVO_JOINT_COUNT];
        int16_t angle_cdeg[OMNIX_SERVO_JOINT_COUNT];
    };
    uint8_t diag_flags;
    uint8_t reserved[10];
} omnix_custom_controller_raw_payload_t;

#define OMNIX_CUSTOM_CONTROLLER_RAW_PAYLOAD_SIZE ((uint16_t)sizeof(omnix_custom_controller_raw_payload_t))
#define OMNIX_SERVO_JOINT_PAYLOAD_SIZE OMNIX_CUSTOM_CONTROLLER_RAW_PAYLOAD_SIZE
typedef omnix_custom_controller_raw_payload_t omnix_servo_joint_angle_payload_t;

typedef struct OMNIX_PACKED {
    uint8_t version;
    uint8_t flags;
    uint16_t seq;
    uint8_t valid_mask;
    uint8_t online_mask;
    union {
        uint16_t raw_u16[OMNIX_SERVO_JOINT_COUNT];
        int16_t angle_cdeg[OMNIX_SERVO_JOINT_COUNT];
    };
    uint8_t pause_phase;
    uint8_t diag_flags;
    uint8_t reserved[8];
} omnix_robot_joint_feedback_payload_t;

#define OMNIX_ROBOT_JOINT_FEEDBACK_PAYLOAD_SIZE ((uint16_t)sizeof(omnix_robot_joint_feedback_payload_t))

static const omnix_controller_joint_raw_calib_t OMNIX_CONTROLLER_JOINT_RAW_CALIB[OMNIX_SERVO_JOINT_COUNT] = {
    {OMNIX_CTRL_J1_RAW_MIN, OMNIX_CTRL_J1_RAW_ZERO, OMNIX_CTRL_J1_RAW_MAX},
    {OMNIX_CTRL_J2_RAW_MIN, OMNIX_CTRL_J2_RAW_ZERO, OMNIX_CTRL_J2_RAW_MAX},
    {OMNIX_CTRL_J3_RAW_MIN, OMNIX_CTRL_J3_RAW_ZERO, OMNIX_CTRL_J3_RAW_MAX},
    {OMNIX_CTRL_J4_RAW_MIN, OMNIX_CTRL_J4_RAW_ZERO, OMNIX_CTRL_J4_RAW_MAX},
    {OMNIX_CTRL_J5_RAW_MIN, OMNIX_CTRL_J5_RAW_ZERO, OMNIX_CTRL_J5_RAW_MAX},
    {OMNIX_CTRL_J6_RAW_MIN, OMNIX_CTRL_J6_RAW_ZERO, OMNIX_CTRL_J6_RAW_MAX},
    {OMNIX_CTRL_J7_RAW_MIN, OMNIX_CTRL_J7_RAW_ZERO, OMNIX_CTRL_J7_RAW_MAX},
};

static inline int32_t Omnix_RoundToI32(float value)
{
    return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static inline const omnix_controller_joint_raw_calib_t* Omnix_ControllerJointRawCalib(uint8_t joint_id)
{
    if (joint_id < 1u || joint_id > OMNIX_SERVO_JOINT_COUNT) {
        return 0;
    }
    return &OMNIX_CONTROLLER_JOINT_RAW_CALIB[joint_id - 1u];
}

static inline uint16_t Omnix_ControllerClampRaw(uint16_t raw,
                                                const omnix_controller_joint_raw_calib_t* calib)
{
    if (calib == 0) {
        return raw;
    }
    if (raw <= calib->raw_min) {
        return calib->raw_min;
    }
    if (raw >= calib->raw_max) {
        return calib->raw_max;
    }
    return raw;
}

static inline int32_t Omnix_ControllerNegativeSpanRaw(const omnix_controller_joint_raw_calib_t* calib)
{
    if (calib == 0 || calib->raw_zero <= calib->raw_min) {
        return 0;
    }
    return (int32_t)calib->raw_zero - (int32_t)calib->raw_min;
}

static inline int32_t Omnix_ControllerPositiveSpanRaw(const omnix_controller_joint_raw_calib_t* calib)
{
    if (calib == 0 || calib->raw_max <= calib->raw_zero) {
        return 0;
    }
    return (int32_t)calib->raw_max - (int32_t)calib->raw_zero;
}

static inline float Omnix_ControllerRawToRelDeg(uint16_t raw,
                                                const omnix_controller_joint_raw_calib_t* calib)
{
    if (calib == 0) {
        return 0.0f;
    }
    return ((float)((int32_t)raw - (int32_t)calib->raw_zero)) *
           OMNIX_CONTROLLER_SERVO_DEG_PER_CODE;
}

static inline uint16_t Omnix_ControllerRelDegToRaw(float rel_deg,
                                                   const omnix_controller_joint_raw_calib_t* calib)
{
    int32_t raw_value = 0;

    if (calib == 0) {
        return 0u;
    }

    raw_value = (int32_t)calib->raw_zero;
    if (rel_deg <= 0.0f) {
        const float neg_deg =
            ((float)Omnix_ControllerNegativeSpanRaw(calib)) * OMNIX_CONTROLLER_SERVO_DEG_PER_CODE;
        if (neg_deg <= 0.0f) {
            return calib->raw_zero;
        }
        {
            float alpha = (-rel_deg) / neg_deg;
            if (alpha > 1.0f) {
                alpha = 1.0f;
            }
            raw_value = (int32_t)calib->raw_zero -
                        Omnix_RoundToI32(alpha * (float)Omnix_ControllerNegativeSpanRaw(calib));
        }
    } else {
        const float pos_deg =
            ((float)Omnix_ControllerPositiveSpanRaw(calib)) * OMNIX_CONTROLLER_SERVO_DEG_PER_CODE;
        if (pos_deg <= 0.0f) {
            return calib->raw_zero;
        }
        {
            float alpha = rel_deg / pos_deg;
            if (alpha > 1.0f) {
                alpha = 1.0f;
            }
            raw_value = (int32_t)calib->raw_zero +
                        Omnix_RoundToI32(alpha * (float)Omnix_ControllerPositiveSpanRaw(calib));
        }
    }

    if (raw_value <= (int32_t)calib->raw_min) {
        return calib->raw_min;
    }
    if (raw_value >= (int32_t)calib->raw_max) {
        return calib->raw_max;
    }
    return (uint16_t)raw_value;
}

static inline bool Omnix_ServoJointValid(uint8_t valid_mask, uint8_t joint_id)
{
    if (joint_id < 1u || joint_id > OMNIX_SERVO_JOINT_COUNT) {
        return false;
    }
    return (valid_mask & (uint8_t)(1u << (joint_id - 1u))) != 0u;
}

static inline bool Omnix_JointRawValid(uint8_t valid_mask, uint8_t joint_id)
{
    return Omnix_ServoJointValid(valid_mask, joint_id);
}

static inline float Omnix_CdegToDeg(int16_t angle_cdeg)
{
    return ((float)angle_cdeg) * 0.01f;
}

static inline float Omnix_CdegToRad(int16_t angle_cdeg)
{
    return Omnix_CdegToDeg(angle_cdeg) * 0.01745329251994329577f;
}

static inline int16_t Omnix_DegToCdeg(float angle_deg)
{
    float scaled = angle_deg * 100.0f;
    int32_t angle_cdeg = Omnix_RoundToI32(scaled);

    if (angle_cdeg > 32767) {
        angle_cdeg = 32767;
    } else if (angle_cdeg < -32768) {
        angle_cdeg = -32768;
    }

    return (int16_t)angle_cdeg;
}

static inline int16_t Omnix_RadToCdeg(float angle_rad)
{
    return Omnix_DegToCdeg(angle_rad * 57.2957795130823208768f);
}

#ifdef __cplusplus
}
#endif

#endif
