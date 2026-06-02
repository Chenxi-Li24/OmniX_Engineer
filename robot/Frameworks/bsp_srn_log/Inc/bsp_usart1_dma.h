#pragma once

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

 /**
  * 初始化 USART1 的 DMA TX 后端：
  * - 注册 HAL 回调（TxCplt/Error）
  * - 创建发送 StreamBuffer
  * 需在 MX_DMA_Init() 与 MX_USART1_UART_Init() 之后调用一次。
  * 依赖：stm32h7xx_hal_conf.h -> USE_HAL_UART_REGISTER_CALLBACKS 置 1
  */
 void   USART1_DMA_TX_Init(void);

 /**
  * 注册“服务任务”（负责在任务态续发 DMA）。
  * 一般在 Log_Task 启动时：USART1_DMA_RegisterServiceTask(xTaskGetCurrentTaskHandle());
  */
 void   USART1_DMA_RegisterServiceTask(TaskHandle_t task);

 /**
  * 在任务上下文里尝试“派发一块”到 DMA（若空闲且队列有数据）。
  * 可在 Log_Task 循环中或收到通知后调用多次以抽干队列。
  */
 void   USART1_DMA_ServiceOnce(void);

 /** 线程安全写入：把数据放入发送队列，并通知服务任务续发 */
 size_t USART1_Write(const void* data, size_t len, TickType_t timeout);

 /** 写 C 字符串（不含 '\0'） */
 size_t USART1_WriteStr(const char* s, TickType_t timeout);

 /** putchar 兼容（返回写入字符或 -1） */
 int    USART1_PutChar(int ch, TickType_t timeout);

#ifdef __cplusplus
}
#endif