//
// Created by sirin on 2025/10/5.
//
// Frameworks/lib_algos/Src/pid.cpp
#include "pid.h"

namespace algo {

static inline float clampf_local(float x, float lo, float hi) {
    if (x < lo) x = lo;
    if (x > hi) x = hi;
    return x;
}

void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3],
              float max_out, float max_iout)
{
    if (!pid) return;
    pid->mode     = mode;
    pid->Kp       = PID[0];
    pid->Ki       = PID[1];
    pid->Kd       = PID[2];
    pid->max_out  = max_out;
    pid->max_iout = max_iout;

    pid->set = 0.0f;
    pid->fdb = 0.0f;

    pid->out  = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;

    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
}

float PID_calc(pid_type_def *pid, float ref, float set)
{
    if (!pid) return 0.0f;

    // 保存本次输入/设定，便于调试观测
    pid->fdb = ref;
    pid->set = set;

    // 误差移位：e2 <- e1 <- e0
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = (set - ref);

    if (pid->mode == PID_DELTA) {
        // 增量式：out += Kp*(e0-e1) + Ki*e0 + Kd*(e0 - 2e1 + e2)
        float delta =
            pid->Kp * (pid->error[0] - pid->error[1]) +
            pid->Ki * (pid->error[0]) +
            pid->Kd * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);

        pid->out += delta;
        pid->out  = clampf_local(pid->out, -pid->max_out, pid->max_out);

        // 分项输出仅供调试
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout += pid->Ki * pid->error[0];
        pid->Iout  = clampf_local(pid->Iout, -pid->max_iout, pid->max_iout);
        pid->Dout = pid->Kd * (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);

        return pid->out;
    } else {
        // 位置式：out = P + I + D
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Iout  = clampf_local(pid->Iout, -pid->max_iout, pid->max_iout);

        // D 基于误差的一阶差分，带简单缓存
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = pid->error[0] - pid->error[1];
        pid->Dout    = pid->Kd * pid->Dbuf[0];

        pid->out = pid->Pout + pid->Iout + pid->Dout;
        pid->out = clampf_local(pid->out, -pid->max_out, pid->max_out);
        return pid->out;
    }
}

void PID_clear(pid_type_def *pid)
{
    if (!pid) return;
    pid->set = 0.0f;
    pid->fdb = 0.0f;

    pid->out  = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;

    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
}

} // namespace algo