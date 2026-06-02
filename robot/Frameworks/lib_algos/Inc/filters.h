//
// Created by sirin on 2025/10/5.
//

#ifndef H723VG_V2_FREERTOS_FILTERS_H
#define H723VG_V2_FREERTOS_FILTERS_H

#pragma once
#include <cmath>
#include "mathutil.h"

namespace algo {

    // 一阶惯性：y += alpha*(x-y)
    struct FirstOrder {
        float alpha{0.1f};
        float y{0.f};
        void reset(float v=0.f){ y=v; }
        float step(float x){ y += alpha*(x-y); return y; }
    };

    // 斜率限幅器（每秒最大变化）
    struct SlewRate {
        float rate{1.0f}; // unit/s
        float y{0.f};
        float step(float target, float dt){
            float max_step = rate * dt;
            float d = target - y;
            if (d >  max_step) d =  max_step;
            if (d < -max_step) d = -max_step;
            y += d;
            return y;
        }
    };

    //一阶滤波初始化
    extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);
    //一阶滤波计算
    extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);

} // namespace algo

#endif //H723VG_V2_FREERTOS_FILTERS_H