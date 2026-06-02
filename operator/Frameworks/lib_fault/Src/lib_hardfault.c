// SPDX-License-Identifier: MIT
// lib_hardfault.c — Rich HardFault diagnostics for STM32H7 using usart1_critical

#include "lib_hardfault.h"
#include "usart1_critical.h"   // 你的最小串口模块（TX=PB14/AF4已在工程中覆盖）
#include <stdbool.h>

/* --------- 小工具 --------- */
__attribute__((always_inline)) static inline uint32_t rd_msp(void){ return __get_MSP(); }
__attribute__((always_inline)) static inline uint32_t rd_psp(void){ return __get_PSP(); }
__attribute__((always_inline)) static inline uint32_t rd_ctl(void){ return __get_CONTROL(); }
__attribute__((always_inline)) static inline uint32_t rd_pri(void){ return __get_PRIMASK(); }
__attribute__((always_inline)) static inline uint32_t rd_bpr(void){ return __get_BASEPRI(); }
__attribute__((always_inline)) static inline uint32_t rd_ipsr(void){ return __get_IPSR(); }

static inline void put(const char*s){ usart1_critical_write_str(s); }
static inline void put_hex32(uint32_t v){ usart1_critical_write_hex_u32(v); }

static bool is_sram(uint32_t a){
    return (a>=0x20000000u && a<0x20600000u) || (a>=0x24000000u && a<0x24080000u);
}
static bool is_flash(uint32_t a){
    return (a>=0x08000000u && a<0x0A000000u);
}

static void libhf_dump_words(const char* tag, const uint32_t* addr, uint32_t words){
    put(tag); put(" @0x"); put_hex32((uint32_t)addr); put(": ");
    if(!is_sram((uint32_t)addr) && !is_flash((uint32_t)addr)){ put("(invalid)\r\n"); return; }
    for(uint32_t i=0;i<words;i++){ put_hex32(addr[i]); put(" "); }
    put("\r\n");
}

static void decode_faults(void){
    uint32_t cfsr = SCB->CFSR, hfsr = SCB->HFSR, dfsr = SCB->DFSR;
    if (cfsr){
        put("CFSR: ");
        if (cfsr & (1u<<0))  put("IACCVIOL ");
        if (cfsr & (1u<<1))  put("DACCVIOL ");
        if (cfsr & (1u<<7))  put("MMARVALID ");
        if (cfsr & (1u<<8))  put("IBUSERR ");
        if (cfsr & (1u<<9))  put("PRECISERR ");
        if (cfsr & (1u<<10)) put("IMPRECISERR ");
        if (cfsr & (1u<<11)) put("UNSTKERR ");
        if (cfsr & (1u<<12)) put("STKERR ");
        if (cfsr & (1u<<15)) put("BFARVALID ");
        if (cfsr & (1u<<16)) put("UNDEFINSTR ");
        if (cfsr & (1u<<17)) put("INVSTATE ");
        if (cfsr & (1u<<18)) put("INVPC ");
        if (cfsr & (1u<<19)) put("NOCP ");
        if (cfsr & (1u<<24)) put("UNALIGNED ");
        if (cfsr & (1u<<25)) put("DIVBYZERO ");
        put("\r\n");
    }
    if (hfsr){
        put("HFSR:");
        if (hfsr & SCB_HFSR_VECTTBL_Msk) put(" VECTTBL");
        if (hfsr & SCB_HFSR_FORCED_Msk)   put(" FORCED");
        if (hfsr & SCB_HFSR_DEBUGEVT_Msk) put(" DEBUG");
        put("\r\n");
    }
    if (dfsr){
        put("DFSR:");
        if (dfsr & SCB_DFSR_HALTED_Msk)   put(" HALTED");
        if (dfsr & SCB_DFSR_BKPT_Msk)     put(" BKPT");
        if (dfsr & SCB_DFSR_DWTTRAP_Msk)  put(" DWT");
        if (dfsr & SCB_DFSR_VCATCH_Msk)   put(" VCATCH");
        if (dfsr & SCB_DFSR_EXTERNAL_Msk) put(" EXTERNAL");
        put("\r\n");
    }
}

/* --------- （可选）FreeRTOS 任务信息 --------- */
#if defined(INCLUDE_uxTaskGetStackHighWaterMark) && (INCLUDE_uxTaskGetStackHighWaterMark==1)
#include "FreeRTOS.h"
#include "task.h"
static void dump_freertos(void){
    TaskHandle_t h = xTaskGetCurrentTaskHandle();
    const char* name = pcTaskGetName(h);
    put("[RTOS] Task="); put(name?name:"<null>");
    put(" Handle=0x"); put_hex32((uint32_t)h); put("\r\n");
#if (configUSE_TRACE_FACILITY==1)
    TaskStatus_t ts; vTaskGetInfo(h,&ts,pdTRUE,eInvalid);
    put("[RTOS] StackBase=0x"); put_hex32((uint32_t)ts.pxStackBase);
    put(" HighWater(words)="); put_hex32((uint32_t)ts.usStackHighWaterMark); put("\r\n");
#endif
}
#else
static void dump_freertos(void){ /* no-op */ }
#endif

/* --------- 对外 API --------- */
void lib_hf_init(void){
    // 非必需；库在 handle 中会自初始化串口
    usart1_critical_init(0u, LIB_HF_USART_BAUD);
}

static volatile uint32_t s_guard = 0;

void lib_hf_handle(uint32_t *sp, uint32_t exc_ret)
{
    if (s_guard++){ for(volatile int i=0;i<100000;i++) __NOP(); NVIC_SystemReset(); }

    __disable_irq();
    // 初始化紧急串口（已在头文件定义 USART1C_KERNEL_HZ_OVERRIDE=25MHz）
    usart1_critical_init(25000000u, LIB_HF_USART_BAUD);

    put("\r\n[HF] === OMNIX H723VG DevBoard 2 HardFault ===\r\n");

    // 核心寄存器与状态
    uint32_t msp=rd_msp(), psp=rd_psp(), ctl=rd_ctl(), pri=rd_pri(), bpr=rd_bpr(), ipsr=rd_ipsr();
    put("MSP=0x"); put_hex32(msp);
    put(" PSP=0x"); put_hex32(psp);
    put(" CONTROL=0x"); put_hex32(ctl);
    put(" PRIMASK=0x"); put_hex32(pri);
    put(" BASEPRI=0x"); put_hex32(bpr);
    put(" IPSR="); put_hex32(ipsr);
    put("\r\n");

    // Fault 标志寄存器
    put("CFSR=0x"); put_hex32(SCB->CFSR);
    put(" HFSR=0x"); put_hex32(SCB->HFSR);
    put(" DFSR=0x"); put_hex32(SCB->DFSR);
    put(" AFSR=0x"); put_hex32(SCB->AFSR);
    put("\r\n");
    put("MMFAR=0x"); put_hex32(SCB->MMFAR);
    put(" BFAR=0x"); put_hex32(SCB->BFAR);
    put("\r\n");
    decode_faults();

    // 解包“已压栈”的寄存器：r0,r1,r2,r3,r12,lr,pc,xpsr
    if (sp && is_sram((uint32_t)sp)){
        put("[STACK] r0=");  put_hex32(sp[0]);
        put(" r1=");         put_hex32(sp[1]);
        put(" r2=");         put_hex32(sp[2]);
        put(" r3=");         put_hex32(sp[3]); put("\r\n");

        put("[STACK] r12="); put_hex32(sp[4]);
        put(" lr=");         put_hex32(sp[5]);
        put(" pc=");         put_hex32(sp[6]);
        put(" xpsr=");       put_hex32(sp[7]); put("\r\n");

        libhf_dump_words("SP dump", sp, 16);              // 栈顶周围
        uint32_t pc = sp[6] & ~1u;
        if (is_flash(pc)) libhf_dump_words("PC nearby", (const uint32_t*)pc, 8);
    } else {
        put("[STACK] invalid SP\r\n");
    }

    // EXC_RETURN & 使用的栈
    put("EXC_RETURN=0x"); put_hex32(exc_ret);
    put("  (SP used = "); put((exc_ret & 4) ? "PSP)\r\n" : "MSP)\r\n");

    // FreeRTOS 信息（若启用）
    dump_freertos();
#if LIB_HF_AUTO_RESET
    put("[HF] System will reset.\r\n");
#else
    put("[HF] System halt.\r\n");
#endif
    usart1_critical_flush();

#if LIB_HF_AUTO_RESET
    NVIC_SystemReset();
#else
    for(;;){ __NOP(); }
#endif
}