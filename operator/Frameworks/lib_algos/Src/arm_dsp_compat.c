//
// Created by sirin on 2025/12/5.
//
// Applications/Src/arm_dsp_compat.c
#include "arm_math.h"
#include <math.h>

// Simple wrapper: map arm_* APIs to standard libm.
// This avoids pulling full CMSIS-DSP sources for now.

float arm_sin_f32(float x)
{
    return sinf(x);
}

float arm_cos_f32(float x)
{
    return cosf(x);
}