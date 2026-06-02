//
// mathutil.cpp —— implementation of math utilities
//

#include "mathutil.h"
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
#include <cmath>
using std::fabs;
#else
#include <math.h>
#endif

// ================== 基础实现（全局 C 符号） ==================

// 快速平方根倒数（Quake 风格）
float invSqrt(float num)
{
    float halfnum = 0.5f * num;
    float y = num;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

// 斜坡初始化
void ramp_init(ramp_function_source_t *r, float frame_period, float max, float min)
{
    if (!r) return;
    r->frame_period = frame_period;
    r->max_value    = max;
    r->min_value    = min;
    r->input        = 0.0f;
    r->out          = 0.0f;
}

// 斜坡计算：out += input * dt，并限幅
void ramp_calc(ramp_function_source_t *r, float input)
{
    if (!r) return;
    r->input = input;
    r->out  += r->input * r->frame_period;
    if (r->out > r->max_value)
    {
        r->out = r->max_value;
    }
    else if (r->out < r->min_value)
    {
        r->out = r->min_value;
    }
}

// 绝对值限幅（指针版）
void abs_limit(float *num, float Limit)
{
    if (!num) return;
    if (*num >  Limit) *num =  Limit;
    if (*num < -Limit) *num = -Limit;
}

// 符号函数
float sign(float value)
{
    return (value >= 0.0f) ? 1.0f : -1.0f;
}

// 浮点死区（algo 版本命名）
float float_deadline(float Value, float minValue, float maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

// RM 老接口名：float_deadband —— 调用 float_deadline
float float_deadband(float Value, float minValue, float maxValue)
{
    return float_deadline(Value, minValue, maxValue);
}

// int16 死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

// 浮点限幅
float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

// int16 限幅
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

// 循环限幅
float loop_float_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    float len = maxValue - minValue;

    if (Input > maxValue)
    {
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        while (Input < minValue)
        {
            Input += len;
        }
    }

    return Input;
}

// 角度 ° 限幅 [-180, 180]
float theta_format(float Ang)
{
    return loop_float_constrain(Ang, -180.0f, 180.0f);
}

// 牛顿迭代平方根
float Sqrt(float x)
{
    float y;
    float delta;
    float maxError;

    if (x <= 0.0f)
    {
        return 0.0f;
    }

    // 初始估计
    y = x * 0.5f;

    // 迭代逼近
    maxError = x * 0.001f;

    do
    {
        delta = (y * y) - x;
        y -= delta / (2.0f * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}

// 浮点四舍五入
int float_rounding(float raw)
{
    int   integer = (int)raw;
    float decimal = raw - (float)integer;
    if (decimal > 0.5f)
        integer++;
    return integer;
}

// ================== OLS 实现 ==================

void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order)
{
    if (!OLS || order == 0)
        return;

    OLS->Order = order;
    OLS->Count = 0;
    OLS->x = (float *)malloc(sizeof(float) * order);
    OLS->y = (float *)malloc(sizeof(float) * order);
    OLS->k = 0.0f;
    OLS->b = 0.0f;
    OLS->StandardDeviation = 0.0f;

    if (OLS->x)
        memset(OLS->x, 0, sizeof(float) * order);
    if (OLS->y)
        memset(OLS->y, 0, sizeof(float) * order);
    memset(OLS->t, 0, sizeof(float) * 4);
}

void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    if (!OLS || !OLS->x || !OLS->y || OLS->Order == 0)
        return;

    float temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset(OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    float denom = (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
    if (denom == 0.0f)
        return;

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / denom;
    OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / denom;

    OLS->StandardDeviation = 0.0f;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
#ifdef __cplusplus
        OLS->StandardDeviation += std::fabs(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
#else
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
#endif
    }
    OLS->StandardDeviation /= (float)OLS->Order;
}

float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    if (!OLS || !OLS->x || !OLS->y || OLS->Order == 0)
        return 0.0f;

    float temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset(OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    float denom = (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
    if (denom == 0.0f)
        return 0.0f;

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / denom;

    OLS->StandardDeviation = 0.0f;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
#ifdef __cplusplus
        OLS->StandardDeviation += std::fabs(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
#else
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
#endif
    }
    OLS->StandardDeviation /= (float)OLS->Order;

    return OLS->k;
}

float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
    if (!OLS || !OLS->x || !OLS->y || OLS->Order == 0)
        return 0.0f;

    float temp = OLS->x[1];
    for (uint16_t i = 0; i < OLS->Order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
    OLS->y[OLS->Order - 1] = y;

    if (OLS->Count < OLS->Order)
    {
        OLS->Count++;
    }

    memset(OLS->t, 0, sizeof(float) * 4);
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i];
        OLS->t[1] += OLS->x[i];
        OLS->t[2] += OLS->x[i] * OLS->y[i];
        OLS->t[3] += OLS->y[i];
    }

    float denom = (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
    if (denom == 0.0f)
        return 0.0f;

    OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / denom;
    OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / denom;

    OLS->StandardDeviation = 0.0f;
    for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
    {
#ifdef __cplusplus
        OLS->StandardDeviation += std::fabs(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
#else
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
#endif
    }
    OLS->StandardDeviation /= (float)OLS->Order;

    return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}

float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS)
{
    if (!OLS) return 0.0f;
    return OLS->k;
}

float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS)
{
    if (!OLS || !OLS->x) return 0.0f;
    return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}