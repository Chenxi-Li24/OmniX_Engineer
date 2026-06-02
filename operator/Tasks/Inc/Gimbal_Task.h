#ifndef H723VG_V2_FREERTOS_GIMBAL_TASK_H
#define H723VG_V2_FREERTOS_GIMBAL_TASK_H

#include <cstddef>
#include <stdint.h>


    typedef struct
    {
        float yaw_relative;  // 相对底盘的 yaw（rad）= s.pos_rad_total - zero_ofst_;//赋值电机角度状态
        //float yaw_absolute;   // IMU / 世界坐标 yaw
        float yaw_omega;
        //float pitch;

        // Jaw force-feedback helper fields (raw torque from DM4310 feedback)
        float   jaw_relative;     // rad (relative)
        float   jaw_omega;        // rad/s
        int16_t jaw_torque_raw;   // DM4310 feedback torque (raw)
        uint8_t jaw_online;       // 1=online, 0=offline
    } gimbal_feedback_t;
    /* 只读接口 */

    extern gimbal_feedback_t gimbal_feedback;

    inline const gimbal_feedback_t* Gimbal_GetFeedback(void)
    {
        return &gimbal_feedback;
    }


#endif //H723VG_V2_FREERTOS_GIMBAL_TASK_H
