// === lib_adp_navision.cpp ===

#include "lib_adp_navision.h"
#include <cstddef>
#include <cstring>

static inline int16_t le16(const uint8_t* p) {
    return static_cast<int16_t>(static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8)));
}

Navi::Navi(uint16_t rx_id)
    : rx_id_(rx_id)
{}

void Navi::onRxFeedback(const CanFrame& f, uint32_t now_ms) {
    if (f.is_ext || f.dlc < 8 || f.id != rx_id_) return;

    __atomic_fetch_add(&state_seq_, 1u, __ATOMIC_ACQ_REL);

    NaviState s = state_;
    s.vx = le16(&f.data[0]);
    s.vy = le16(&f.data[2]);
    s.omega_z = le16(&f.data[4]);
    s.reserved0 = f.data[6];
    s.reserved1 = f.data[7];
    s.is_navi_online = (f.data[6] == 0xFFu) && (f.data[7] == 0xFFu);
    s.last_rx_ms = now_ms;
    s.rx_count++;

    state_ = s;

    __atomic_fetch_add(&state_seq_, 1u, __ATOMIC_RELEASE);
}

bool Navi::snapshot(NaviState& out) const {
    for (int attempt = 0; attempt < 3; ++attempt) {
        uint32_t s1 = __atomic_load_n(&state_seq_, __ATOMIC_ACQUIRE);
        if (s1 & 1u) continue;

        NaviState tmp;
        std::memcpy(&tmp, &state_, sizeof(tmp));

        uint32_t s2 = __atomic_load_n(&state_seq_, __ATOMIC_ACQUIRE);
        if (s1 == s2) {
            out = tmp;
            return true;
        }
    }
    return false;
}

Vision::Vision(uint16_t b1_id)
    : b1_id_(b1_id)
{
    std::memset(raw_, 0, sizeof(raw_));
    for (uint8_t i = 0; i < VISION_ITEM_COUNT; ++i) {
        raw_seq_[i] = 0;
        last_seq_[i] = 0;
    }
    std::memset(&state_, 0, sizeof(state_));
}

VisionItem* Vision::item_by_index(VisionState* s, uint8_t idx)
{
    if (!s || idx >= VISION_ITEM_COUNT) return nullptr;

    static const size_t offsets[VISION_ITEM_COUNT] = {
        offsetof(VisionState, B1),
        offsetof(VisionState, B2),
        offsetof(VisionState, B3),
        offsetof(VisionState, B4),
        offsetof(VisionState, B5),
        offsetof(VisionState, BOut),
        offsetof(VisionState, BSen),
        offsetof(VisionState, R1),
        offsetof(VisionState, R2),
        offsetof(VisionState, R3),
        offsetof(VisionState, R4),
        offsetof(VisionState, R5),
        offsetof(VisionState, ROut),
        offsetof(VisionState, RSen),
    };

    return reinterpret_cast<VisionItem*>(reinterpret_cast<uint8_t*>(s) + offsets[idx]);
}

const VisionItem* Vision::item_by_index(const VisionState* s, uint8_t idx)
{
    return item_by_index(const_cast<VisionState*>(s), idx);
}

bool Vision::raw_snapshot(uint8_t idx, VisionRaw& out, uint32_t& seq) const
{
    for (int attempt = 0; attempt < 3; ++attempt) {
        uint32_t s1 = __atomic_load_n(&raw_seq_[idx], __ATOMIC_ACQUIRE);
        if (s1 & 1u) continue;

        VisionRaw tmp = raw_[idx];

        uint32_t s2 = __atomic_load_n(&raw_seq_[idx], __ATOMIC_ACQUIRE);
        if (s1 == s2) {
            out = tmp;
            seq = s2;
            return true;
        }
    }
    return false;
}

void Vision::onRxFeedback(const CanFrame& f, uint32_t now_ms)
{
    (void)now_ms;
    if (f.is_ext || f.dlc < 8) return;
    if (f.data[4] || f.data[5] || f.data[6]) return;

    const uint16_t base = b1_id_;
    if (f.id < base || f.id >= (uint16_t)(base + VISION_ITEM_COUNT)) return;
    const uint8_t idx = static_cast<uint8_t>(f.id - base);

    __atomic_fetch_add(&raw_seq_[idx], 1u, __ATOMIC_ACQ_REL);

    VisionRaw r = raw_[idx];
    r.dx = le16(&f.data[0]);
    r.dy = le16(&f.data[2]);
    r.fire = (f.data[7] == 0xFFu) ? 1u : 0u;
    raw_[idx] = r;

    __atomic_fetch_add(&raw_seq_[idx], 1u, __ATOMIC_RELEASE);
}

void Vision::update()
{
    __atomic_fetch_add(&state_seq_, 1u, __ATOMIC_ACQ_REL);

    for (uint8_t idx = 0; idx < VISION_ITEM_COUNT; ++idx) {
        VisionItem* item = item_by_index(&state_, idx);
        if (!item) continue;

        VisionRaw raw{};
        uint32_t seq = 0;
        if (raw_snapshot(idx, raw, seq) && seq != last_seq_[idx]) {
            item->dx = raw.dx;
            item->dy = raw.dy;
            item->fire = raw.fire;
            item->is_appear = 1u;
            last_seq_[idx] = seq;
        } else {
            item->dx = 0;
            item->dy = 0;
            item->fire = 0;
            item->is_appear = 0u;
        }
    }

    __atomic_fetch_add(&state_seq_, 1u, __ATOMIC_RELEASE);
}

bool Vision::snapshot(VisionState& out) const
{
    for (int attempt = 0; attempt < 3; ++attempt) {
        uint32_t s1 = __atomic_load_n(&state_seq_, __ATOMIC_ACQUIRE);
        if (s1 & 1u) continue;

        VisionState tmp;
        std::memcpy(&tmp, &state_, sizeof(tmp));

        uint32_t s2 = __atomic_load_n(&state_seq_, __ATOMIC_ACQUIRE);
        if (s1 == s2) {
            out = tmp;
            return true;
        }
    }
    return false;
}
