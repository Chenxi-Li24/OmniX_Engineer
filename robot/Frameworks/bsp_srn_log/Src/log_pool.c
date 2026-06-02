// Log_Pool.c
#include "cmsis_os2.h"
#include "bsp_srn_log.h"
#include "FreeRTOS.h"
#include "task.h"

osMemoryPoolId_t Log_PoolHandle;  // ★ 唯一“定义”（非 extern）
static const osMemoryPoolAttr_t s_LogPoolAttr = { .name = "Log_Pool" };

/* 你的日志块大小（对齐后）大概率是 296 字节 */
_Static_assert(sizeof(srn_log_msg_t) >= 280 && sizeof(srn_log_msg_t) <= 512,
               "srn_log_msg_t size looks unexpected; update pool block size if you hardcode it somewhere.");

/* 可选：估个安全的块数 */
#ifndef LOG_POOL_COUNT
#define LOG_POOL_COUNT 64
#endif

/* 只在 osKernelInitialize() 之后 & osKernelStart() 之前调用一次 */
void Log_Pool_Create(void)
{
    // 内核必须已初始化
    configASSERT(osKernelGetState() == osKernelReady);

    // ★ 强制从 heap 动态分配（第三参传 NULL）
    Log_PoolHandle = osMemoryPoolNew(LOG_POOL_COUNT, sizeof(srn_log_msg_t), NULL);

    if (Log_PoolHandle == NULL) {
            // 打印 heap，定位是否不够
    }
    configASSERT(Log_PoolHandle != NULL);
}