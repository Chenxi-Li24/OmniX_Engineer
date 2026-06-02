//
// lib_adp_omximu.cpp
//
#include "lib_adp_omximu.h"
#include <cstring>

namespace {
    static inline uint16_t le_u16(const uint8_t* p)
    {
        return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) |
                                     (static_cast<uint16_t>(p[1]) << 8));
    }

    static inline int16_t le_i16(const uint8_t* p)
    {
        return static_cast<int16_t>(le_u16(p));
    }

    static float half_to_float(uint16_t h)
    {
        const uint32_t sign = (static_cast<uint32_t>(h & 0x8000u)) << 16;
        const uint32_t exp  = (h >> 10) & 0x1Fu;
        uint32_t mant       = h & 0x03FFu;
        uint32_t f          = 0u;

        if (exp == 0u) {
            if (mant == 0u) {
                f = sign;
            } else {
                int32_t exp_norm = -14;
                while ((mant & 0x0400u) == 0u) {
                    mant <<= 1;
                    exp_norm--;
                }
                mant &= 0x03FFu;
                const uint32_t f_exp  = static_cast<uint32_t>(exp_norm + 127) << 23;
                const uint32_t f_mant = mant << 13;
                f = sign | f_exp | f_mant;
            }
        } else if (exp == 0x1Fu) {
            const uint32_t f_exp  = 0xFFu << 23;
            const uint32_t f_mant = mant ? (mant << 13) : 0u;
            f = sign | f_exp | f_mant;
        } else {
            const uint32_t f_exp  = (exp + (127u - 15u)) << 23;
            const uint32_t f_mant = mant << 13;
            f = sign | f_exp | f_mant;
        }

        float out;
        std::memcpy(&out, &f, sizeof(out));
        return out;
    }
}

extern "C" {

void OmxImu_Init(OmxImu* imu, uint16_t quat_id)
{
    if (!imu) {
        return;
    }

    std::memset(imu, 0, sizeof(*imu));

    if (quat_id == 0u) {
        quat_id = OMXIMU_QUAT_STD_ID;
    }

    imu->quat_id = quat_id;
    imu->euler_id = static_cast<uint16_t>(quat_id + 1u);
    imu->gyro_id = static_cast<uint16_t>(quat_id + 2u);
    imu->accel_id = static_cast<uint16_t>(quat_id + 3u);
    imu->offline_ms = 100u;
    imu->state.is_online = false;
}

void OmxImu_SetOfflineTimeout(OmxImu* imu, uint32_t ms)
{
    if (!imu) {
        return;
    }
    imu->offline_ms = ms;
}

uint16_t OmxImu_QuatId(const OmxImu* imu)
{
    return imu ? imu->quat_id : OMXIMU_QUAT_STD_ID;
}

uint16_t OmxImu_EulerId(const OmxImu* imu)
{
    return imu ? imu->euler_id : OMXIMU_EULER_STD_ID;
}

uint16_t OmxImu_GyroId(const OmxImu* imu)
{
    return imu ? imu->gyro_id : OMXIMU_GYRO_STD_ID;
}

uint16_t OmxImu_AccelId(const OmxImu* imu)
{
    return imu ? imu->accel_id : OMXIMU_ACCEL_STD_ID;
}

void OmxImu_OnRxFeedback(OmxImu* imu, const CanFrame* f, uint32_t now_ms)
{
    if (!imu || !f) {
        return;
    }
    if (f->is_ext || f->dlc < 8u) {
        return;
    }

    const uint16_t id = static_cast<uint16_t>(f->id);
    const uint8_t* d = f->data;

    if (id != imu->quat_id && id != imu->euler_id &&
        id != imu->gyro_id && id != imu->accel_id) {
        return;
    }

    __atomic_fetch_add(&imu->state_seq, 1u, __ATOMIC_ACQ_REL);

    OmxImuState s = imu->state;

    if (id == imu->quat_id) {
        s.quat[0] = half_to_float(le_u16(&d[0]));
        s.quat[1] = half_to_float(le_u16(&d[2]));
        s.quat[2] = half_to_float(le_u16(&d[4]));
        s.quat[3] = half_to_float(le_u16(&d[6]));
    } else if (id == imu->euler_id) {
        s.euler[0] = half_to_float(le_u16(&d[0]));
        s.euler[1] = half_to_float(le_u16(&d[2]));
        s.euler[2] = half_to_float(le_u16(&d[4]));
        s.euler_seq = d[6];
    } else if (id == imu->gyro_id) {
        const int16_t gx = le_i16(&d[0]);
        const int16_t gy = le_i16(&d[2]);
        const int16_t gz = le_i16(&d[4]);
        s.gyro[0] = static_cast<float>(gx) * 0.001f;
        s.gyro[1] = static_cast<float>(gy) * 0.001f;
        s.gyro[2] = static_cast<float>(gz) * 0.001f;
        s.gyro_seq = d[6];
    } else if (id == imu->accel_id) {
        const int16_t ax = le_i16(&d[0]);
        const int16_t ay = le_i16(&d[2]);
        const int16_t az = le_i16(&d[4]);
        s.accel[0] = static_cast<float>(ax) * 0.01f;
        s.accel[1] = static_cast<float>(ay) * 0.01f;
        s.accel[2] = static_cast<float>(az) * 0.01f;
        s.accel_seq = d[6];
    }

    s.last_rx_ms = now_ms;
    s.is_online = true;
    s.rx_count++;
    imu->state = s;

    __atomic_fetch_add(&imu->state_seq, 1u, __ATOMIC_RELEASE);
}

void OmxImu_Tick(OmxImu* imu, uint32_t now_ms)
{
    if (!imu) {
        return;
    }

    const uint32_t last = __atomic_load_n(&imu->state.last_rx_ms, __ATOMIC_RELAXED);
    const bool was_online = __atomic_load_n(&imu->state.is_online, __ATOMIC_RELAXED);

    if (was_online && (now_ms - last > imu->offline_ms)) {
        __atomic_fetch_add(&imu->state_seq, 1u, __ATOMIC_ACQ_REL);
        imu->state.is_online = false;
        __atomic_fetch_add(&imu->state_seq, 1u, __ATOMIC_RELEASE);
    }
}

bool OmxImu_Snapshot(const OmxImu* imu, OmxImuState* out)
{
    if (!imu || !out) {
        return false;
    }

    for (int attempt = 0; attempt < 3; ++attempt) {
        const uint32_t s1 = __atomic_load_n(&imu->state_seq, __ATOMIC_ACQUIRE);
        if (s1 & 1u) {
            continue;
        }

        OmxImuState tmp = imu->state;

        const uint32_t s2 = __atomic_load_n(&imu->state_seq, __ATOMIC_ACQUIRE);
        if (s1 == s2) {
            *out = tmp;
            return true;
        }
    }

    return false;
}

} // extern "C"
