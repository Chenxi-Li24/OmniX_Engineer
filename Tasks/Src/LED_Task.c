// Tasks/Src/LED_Task.c
#include "bsp_ws2812.h"
#include "bsp_buzzer.h"
#include "LED_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_srn_log.h"
#include "SerialServo_Task.h"

// 统一遥控管理器（lib_remote_control）
#include "lib_remote_control.h"

extern TaskHandle_t LED_TaskHandle;

static RC_State s_rc;     // 放静态/全局，减少栈占用
static int led_tick = 0;
static volatile int led_calib_blink = 0;  // 剩余闪烁tick数
static volatile int led_calib_phase = 0;  // 内部相位

void LED_NotifyCalibStart(void) {
    // 设定闪烁时长：比如 40 tick（和本任务 vTaskDelay(1) 对应，大约 40ms*40=~1.6s）
    led_calib_blink = 40;
    led_calib_phase = 0;
}


void Start_LED_Task(void const * argument) {
    (void)argument;

    for (;;) {
        RC_GetSnapshot(&s_rc);
        RC_Status st = RC_GetStatus();
        serial_servo_boot_state_t boot_state = g_servo_boot_state;
        uint8_t online_count = g_servo_online_count;
        uint8_t offline_count = (online_count >= SERVO_COUNT) ? 0u : (uint8_t)(SERVO_COUNT - online_count);

        if (boot_state == SERVO_BOOT_WAIT_ONLINE) {
            if (offline_count == 0u) {
                WS2812_SetPixel(0, 0, 255, 0);
            } else {
                uint32_t period_ms = 1000u / offline_count;
                if (period_ms < 100u) {
                    period_ms = 100u;
                }
                if ((led_tick % (int)period_ms) < (int)(period_ms / 2u)) {
                    WS2812_SetPixel(0, 255, 0, 0);
                } else {
                    WS2812_SetPixel(0, 0, 0, 0);
                }
            }
        } else if (boot_state == SERVO_BOOT_RESTORING || boot_state == SERVO_BOOT_HOLDING) {
            uint32_t period_ms = 300u;
            if ((led_tick % (int)period_ms) < (int)(period_ms / 2u)) {
                WS2812_SetPixel(0, 0, 255, 0);
            } else {
                WS2812_SetPixel(0, 0, 0, 0);
            }
        } else {
            if (offline_count == 0u) {
                WS2812_SetPixel(0, 0, 255, 0);     // all online -> green
            } else if (offline_count >= SERVO_COUNT) {
                WS2812_SetPixel(0, 255, 0, 0);     // all offline -> red
            } else {
                uint32_t period_ms = 1000u / offline_count;
                if (period_ms < 100u) {
                    period_ms = 100u;
                }
                if ((led_tick % (int)period_ms) < (int)(period_ms / 2u)) {
                    WS2812_SetPixel(0, 255, 0, 0); // partial offline -> red blink
                } else {
                    WS2812_SetPixel(0, 0, 0, 0);
                }
            }
        }

        /* --- 改造后的“索引1”来源指示灯 --- */
        const int dr16_on = (st.dr16_online != 0);
        const int vt03_on = (st.vt03_online != 0);

        if (dr16_on && vt03_on) {
            WS2812_SetPixel(1, 255, 0, 255);   // 紫：两者都在线
        } else if (dr16_on) {
            WS2812_SetPixel(1, 0, 255, 0);     // 绿：仅 DR16 在线
        } else if (vt03_on) {
            WS2812_SetPixel(1, 0, 0, 255);     // 蓝：仅 VT03 在线
        } else {
            WS2812_SetPixel(1, 255, 0, 0);     // 红：都不在线
        }

        /* --- 原来的拨杆颜色（索引2），仍根据统一快照的 rc.s[0] --- */
        if (switch_is_up(s_rc.rc.s[0])) {
            WS2812_SetPixel(2, 0, 0, 255);   // 上=蓝
        } else if (switch_is_mid(s_rc.rc.s[0])) {
            WS2812_SetPixel(2, 255, 0, 0);   // 中=红
        } else {
            WS2812_SetPixel(2, 0, 255, 0);   // 下=绿
        }

        // --- 进入校准时的提示灯：占用像素索引 3（可换） ---
        if (led_calib_blink > 0) {
            // 简单两色快闪：蓝-白交替
            if ((led_calib_phase & 1) == 0) {
                WS2812_SetPixel(3, 0, 0, 255);     // 蓝
            } else {
                WS2812_SetPixel(3, 255, 255, 255); // 白
            }
            led_calib_phase++;
            led_calib_blink--;
        } else {
            // 提示结束后熄灭该像素，不影响其它指示灯
            WS2812_SetPixel(3, 0, 0, 0);
        }

        WS2812_Commit_StartAsync(NULL);
        led_tick++;
        vTaskDelay(1);
    }
}
