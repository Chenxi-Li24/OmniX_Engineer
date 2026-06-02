// === lib_adp_navision.h ===
// Navi + Vision CAN adapters:
// - Navi default RX ID: NAVI_DEFAULT_STD_ID (0x401)
// - Navi payload (8B, little-endian):
//   [0:1] VX int16, [2:3] VY int16, [4:5] OmegaZ int16, [6:7] reserved (0xFF)
// - Vision base ID: VISION_B1_STD_ID (0x300)
// - Vision payload (8B, little-endian):
//   [0:1] dX int16, [2:3] dY int16, [4:6] reserved 0x00, [7] FIRE (0xFF -> true)

#ifndef H723VG_V2_FREERTOS_LIB_ADP_NAVISION_H
#define H723VG_V2_FREERTOS_LIB_ADP_NAVISION_H

#pragma once
#include <cstdint>
#include <cstddef>

#ifndef ADP_CANFRAME_DEFINED
#define ADP_CANFRAME_DEFINED
struct CanFrame {
    uint32_t id;
    uint8_t  dlc;
    bool     is_ext;
    uint8_t  data[8];
};
#endif

#ifndef NAVI_DEFAULT_STD_ID
#define NAVI_DEFAULT_STD_ID 0x401u
#endif

#ifndef NAVISION_DEFAULT_STD_ID
#define NAVISION_DEFAULT_STD_ID NAVI_DEFAULT_STD_ID
#endif

struct NaviState {
    int16_t vx = 0;
    int16_t vy = 0;
    int16_t omega_z = 0;
    uint8_t reserved0 = 0;
    uint8_t reserved1 = 0;

    uint32_t last_rx_ms = 0;
    bool     is_navi_online = false;
    uint32_t rx_count = 0;
};

class Navi {
public:
    explicit Navi(uint16_t rx_id = NAVI_DEFAULT_STD_ID);

    uint16_t rxId() const { return rx_id_; }
    void set_rx_id(uint16_t rx_id) { rx_id_ = rx_id; }

    void onRxFeedback(const CanFrame& f, uint32_t now_ms);

    bool snapshot(NaviState& out) const;
    NaviState state() const noexcept {
        NaviState s{};
        (void)snapshot(s);
        return s;
    }

private:
    uint16_t rx_id_;
    mutable volatile uint32_t state_seq_ = 0;
    NaviState state_{};
};

using Navision = Navi;
using NavisionState = NaviState;

#ifndef VISION_B1_STD_ID
#define VISION_B1_STD_ID 0x300u
#endif
#define VISION_B2_STD_ID   (VISION_B1_STD_ID + 1u)
#define VISION_B3_STD_ID   (VISION_B1_STD_ID + 2u)
#define VISION_B4_STD_ID   (VISION_B1_STD_ID + 3u)
#define VISION_B5_STD_ID   (VISION_B1_STD_ID + 4u)
#define VISION_BOUT_STD_ID (VISION_B1_STD_ID + 5u)
#define VISION_BSEN_STD_ID (VISION_B1_STD_ID + 6u)
#define VISION_R1_STD_ID   (VISION_B1_STD_ID + 7u)
#define VISION_R2_STD_ID   (VISION_B1_STD_ID + 8u)
#define VISION_R3_STD_ID   (VISION_B1_STD_ID + 9u)
#define VISION_R4_STD_ID   (VISION_B1_STD_ID + 10u)
#define VISION_R5_STD_ID   (VISION_B1_STD_ID + 11u)
#define VISION_ROUT_STD_ID (VISION_B1_STD_ID + 12u)
#define VISION_RSEN_STD_ID (VISION_B1_STD_ID + 13u)

enum { VISION_ITEM_COUNT = 14u };

struct VisionItem {
    int16_t dx = 0;
    int16_t dy = 0;
    uint8_t fire = 0;
    uint8_t is_appear = 0;
};

struct VisionState {
    VisionItem B1;
    VisionItem B2;
    VisionItem B3;
    VisionItem B4;
    VisionItem B5;
    VisionItem BOut;
    VisionItem BSen;
    VisionItem R1;
    VisionItem R2;
    VisionItem R3;
    VisionItem R4;
    VisionItem R5;
    VisionItem ROut;
    VisionItem RSen;
};

class Vision {
public:
    explicit Vision(uint16_t b1_id = VISION_B1_STD_ID);

    uint16_t base_id() const { return b1_id_; }
    void set_base_id(uint16_t b1_id) { b1_id_ = b1_id; }

    void onRxFeedback(const CanFrame& f, uint32_t now_ms);
    void update();

    bool snapshot(VisionState& out) const;
    VisionState state() const noexcept {
        VisionState s{};
        (void)snapshot(s);
        return s;
    }

private:
    struct VisionRaw {
        int16_t dx = 0;
        int16_t dy = 0;
        uint8_t fire = 0;
    };

    static VisionItem* item_by_index(VisionState* s, uint8_t idx);
    static const VisionItem* item_by_index(const VisionState* s, uint8_t idx);
    bool raw_snapshot(uint8_t idx, VisionRaw& out, uint32_t& seq) const;

    uint16_t b1_id_;
    VisionRaw raw_[VISION_ITEM_COUNT];
    mutable volatile uint32_t raw_seq_[VISION_ITEM_COUNT];
    uint32_t last_seq_[VISION_ITEM_COUNT];
    mutable volatile uint32_t state_seq_ = 0;
    VisionState state_{};
};

#endif // H723VG_V2_FREERTOS_LIB_ADP_NAVISION_H
