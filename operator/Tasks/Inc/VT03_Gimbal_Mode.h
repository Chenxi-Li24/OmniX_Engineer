#ifndef H723VG_V2_FREERTOS_VT03_GIMBAL_MODE_H
#define H723VG_V2_FREERTOS_VT03_GIMBAL_MODE_H
#pragma once

#include "RC_Control_Mode.h"

RcControlMode VT03_Gimbal_ResolveControlMode(const RC_State& rc, const RC_Status& st);

#endif
