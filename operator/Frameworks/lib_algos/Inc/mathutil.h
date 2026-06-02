//
// mathutil.h —— unified math utilities (C + C++)
//

#ifndef H723VG_V2_FREERTOS_MATHUTIL_H
#define H723VG_V2_FREERTOS_MATHUTIL_H

#pragma once

#include <stdint.h>

#ifdef __cplusplus
#include <cmath>
#else
#include <math.h>
#endif

// ====================== 全局 C 接口（C / C++ 都能用） ======================

// 基础小工具：clamp / wrap / 编码器映射 / 死区 / 遥控归一化
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline float wrap_pi(float a) {
    while (a >  3.1415926535f) a -= 6.283185307f;
    while (a < -3.1415926535f) a += 6.283185307f;
    return a;
}

static inline float wrap_deg(float deg) {
    return wrap_pi(deg * 3.14159265358979323846f / 180.0f) * 180.0f / 3.14159265358979323846f;
}

static inline float rad_from_u16_8192(uint16_t enc) {
    return (6.283185307f) * ( (float)enc / 8192.0f );
}

static inline uint16_t u16_wrap_8192(int32_t x) {
    int32_t y = x % 8192;
    if (y < 0) y += 8192;
    return (uint16_t)y;
}

static inline float deadzone(float x, float dz) {
#ifdef __cplusplus
    return (std::fabs(x) < dz) ? 0.0f : x;
#else
    return (fabsf(x) < dz) ? 0.0f : x;
#endif
}

static inline float norm660_to_unit(int16_t ch, int16_t dead) {
    float x = (float)ch;
#ifdef __cplusplus
    if (std::fabs(x) < (float)dead) return 0.0f;
#else
    if (fabsf(x) < (float)dead) return 0.0f;
#endif
    float r = x / 660.0f;
    return clampf(r, -1.0f, 1.0f);
}

// ---------- ramp / filter 结构体 ----------

typedef struct __attribute__((packed))
{
    float input;        // 输入数据
    float out;          // 输出数据
    float min_value;    // 限幅最小值
    float max_value;    // 限幅最大值
    float frame_period; // 时间间隔
} ramp_function_source_t;

typedef struct __attribute__((packed))
{
    float input;        // 输入数据
    float out;          // 滤波输出
    float num[1];       // 滤波参数
    float frame_period; // 时间间隔
} first_order_filter_type_t;

// ---------- OLS 结构体 ----------

typedef struct
{
    uint16_t Order;
    uint32_t Count;

    float *x;
    float *y;

    float k;
    float b;

    float StandardDeviation;

    float t[4];
} Ordinary_Least_Squares_t;

// ---------- 函数声明（全局 C 接口） ----------

// 快速平方根倒数
float invSqrt(float num);

// 斜坡初始化 / 计算
void  ramp_init(ramp_function_source_t *r, float frame_period, float max, float min);
void  ramp_calc(ramp_function_source_t *r, float input);

// 绝对限幅（指针版本）
void  abs_limit(float *num, float Limit);

// 符号
float sign(float value);

// 浮点死区（名字沿用之前 algo 版本）
float float_deadline(float Value, float minValue, float maxValue);
// 兼容老 RM 习惯：float_deadband -> 调用 float_deadline
float float_deadband(float Value, float minValue, float maxValue);

// int16 死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);

// 限幅
float   float_constrain(float Value, float minValue, float maxValue);
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);

// 循环限幅
float loop_float_constrain(float Input, float minValue, float maxValue);

// 角度 ° 限幅 [-180, 180]
float theta_format(float Ang);

// 牛顿迭代平方根（老接口）
float Sqrt(float x);

// 浮点四舍五入
int float_rounding(float raw);

// OLS 系列
void  OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order);
void  OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS);
float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS);

// ---------- 宏（从 user_lib 搬过来） ----------

#ifndef PI
#define PI 3.14159265354f
#endif

#define VAL_LIMIT(val, min, max) \
    do                           \
    {                            \
        if ((val) <= (min))      \
        {                        \
            (val) = (min);       \
        }                        \
        else if ((val) >= (max)) \
        {                        \
            (val) = (max);       \
        }                        \
    } while (0)

#define ANGLE_LIMIT_360(val, angle)     \
    do                                  \
    {                                   \
        (val) = (angle) - (int)(angle); \
        (val) += (int)(angle) % 360;    \
    } while (0)

#define ANGLE_LIMIT_360_TO_180(val) \
    do                              \
    {                               \
        if ((val) > 180)            \
            (val) -= 360;           \
    } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

// 弧度格式化为 [-PI, PI]
#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

// ====================== C++ 下保留原 algo:: 接口 ======================

#ifdef __cplusplus
namespace algo {

    // 把全局的这些 helper 带进 namespace，保持你原来 algo:: 的用法
    using ::clampf;
    using ::wrap_pi;
    using ::wrap_deg;
    using ::rad_from_u16_8192;
    using ::u16_wrap_8192;
    using ::deadzone;
    using ::norm660_to_unit;

    using ::ramp_function_source_t;
    using ::first_order_filter_type_t;
    using ::Ordinary_Least_Squares_t;

    // 这些用 inline 包一层，调用全局 C 实现
    inline float invSqrt(float num)                                { return ::invSqrt(num); }
    inline void  ramp_init(ramp_function_source_t *r, float fp,
                           float max, float min)                   { ::ramp_init(r, fp, max, min); }
    inline void  ramp_calc(ramp_function_source_t *r, float input) { ::ramp_calc(r, input); }
    inline void  abs_limit(float *num, float Limit)                { ::abs_limit(num, Limit); }
    inline float sign(float value)                                 { return ::sign(value); }
    inline float float_deadline(float v, float mn, float mx)       { return ::float_deadline(v, mn, mx); }
    inline float float_deadband(float v, float mn, float mx)       { return ::float_deadband(v, mn, mx); }
    inline int16_t int16_deadline(int16_t v, int16_t mn, int16_t mx){ return ::int16_deadline(v, mn, mx); }
    inline float   float_constrain(float v, float mn, float mx)    { return ::float_constrain(v, mn, mx); }
    inline int16_t int16_constrain(int16_t v, int16_t mn, int16_t mx)
                                                                   { return ::int16_constrain(v, mn, mx); }
    inline float loop_float_constrain(float v, float mn, float mx) { return ::loop_float_constrain(v, mn, mx); }
    inline float theta_format(float Ang)                           { return ::theta_format(Ang); }

    inline float Sqrt(float x)                                     { return ::Sqrt(x); }
    inline int   float_rounding(float raw)                         { return ::float_rounding(raw); }

    inline void  OLS_Init(Ordinary_Least_Squares_t *o, uint16_t n) { ::OLS_Init(o, n); }
    inline void  OLS_Update(Ordinary_Least_Squares_t *o, float dx, float y)
                                                                   { ::OLS_Update(o, dx, y); }
    inline float OLS_Derivative(Ordinary_Least_Squares_t *o, float dx, float y)
                                                                   { return ::OLS_Derivative(o, dx, y); }
    inline float OLS_Smooth(Ordinary_Least_Squares_t *o, float dx, float y)
                                                                   { return ::OLS_Smooth(o, dx, y); }
    inline float Get_OLS_Derivative(Ordinary_Least_Squares_t *o)   { return ::Get_OLS_Derivative(o); }
    inline float Get_OLS_Smooth(Ordinary_Least_Squares_t *o)       { return ::Get_OLS_Smooth(o); }

} // namespace algo
#endif // __cplusplus

#endif // H723VG_V2_FREERTOS_MATHUTIL_H