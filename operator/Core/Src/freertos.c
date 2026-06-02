/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart1_critical.h"
#include "bsp_ws2812.h"
#include "log_pool.h"
#include "_NTFDCAN_Tasks.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for LED_Task */
osThreadId_t LED_TaskHandle;
const osThreadAttr_t LED_Task_attributes = {
  .name = "LED_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for RC_Task */
osThreadId_t RC_TaskHandle;
const osThreadAttr_t RC_Task_attributes = {
  .name = "RC_Task",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for Log_Task */
osThreadId_t Log_TaskHandle;
const osThreadAttr_t Log_Task_attributes = {
  .name = "Log_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Chassis_Task */
osThreadId_t Chassis_TaskHandle;
const osThreadAttr_t Chassis_Task_attributes = {
  .name = "Chassis_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Gimbal_Task */
osThreadId_t Gimbal_TaskHandle;
const osThreadAttr_t Gimbal_Task_attributes = {
  .name = "Gimbal_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Shoot_Task */
osThreadId_t Shoot_TaskHandle;
const osThreadAttr_t Shoot_Task_attributes = {
  .name = "Shoot_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Referee_Task */
osThreadId_t Referee_TaskHandle;
const osThreadAttr_t Referee_Task_attributes = {
  .name = "Referee_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Gimbal_behavior */
osThreadId_t Gimbal_behaviorHandle;
const osThreadAttr_t Gimbal_behavior_attributes = {
  .name = "Gimbal_behavior",
  // Pause/mapping path contains multiple control branches and diagnostics.
  // Keep a larger stack margin to avoid overflow under heavy logging.
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for IMU_Task */
osThreadId_t IMU_TaskHandle;
const osThreadAttr_t IMU_Task_attributes = {
  .name = "IMU_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh5,
};
/* Definitions for Log_Queue */
osMessageQueueId_t Log_QueueHandle;
const osMessageQueueAttr_t Log_Queue_attributes = {
  .name = "Log_Queue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start_LED_Task(void *argument);
void Start_RC_Task(void *argument);
void Start_Log_Task(void *argument);
void Start_Chassis_Task(void *argument);
void Start_Gimbal_Task(void *argument);
void Start_Shoot_Task(void *argument);
void Start_Referee_Task(void *argument);
void Start_Gimbal_behave(void *argument);
void Start_IMU_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
  __disable_irq();
  usart1_critical_init(25000000u, 115200u);

  usart1_critical_write_str("\r\n[KERNEL] === FreeRTOS stack overflow detected ===\r\n");
  usart1_critical_write_str("[KERNEL] Task name: ");
  if (pcTaskName) {
    usart1_critical_write_str(pcTaskName);
  } else {
    usart1_critical_write_str("<null>");
  }
  usart1_critical_write_str("\r\n");
  usart1_critical_write_str("Handle=0x");
  usart1_critical_write_hex_u32((uint32_t)xTask);
  usart1_critical_write_str("\r\n");
  // 3) CPU寄存器与异常状态快照（有助定位）
    uint32_t msp     = __get_MSP();
    uint32_t psp     = __get_PSP();
    uint32_t control = __get_CONTROL();
    uint32_t primask = __get_PRIMASK();
    uint32_t basepri = __get_BASEPRI();
    uint32_t ipsr    = __get_IPSR();   // 当前异常号（0=线程态）

    usart1_critical_write_str("MSP=0x");  usart1_critical_write_hex_u32(msp);
    usart1_critical_write_str("  PSP=0x"); usart1_critical_write_hex_u32(psp);
    usart1_critical_write_str("  CONTROL=0x"); usart1_critical_write_hex_u32(control);
    usart1_critical_write_str("  PRIMASK=0x"); usart1_critical_write_hex_u32(primask);
    usart1_critical_write_str("  BASEPRI=0x"); usart1_critical_write_hex_u32(basepri);
    usart1_critical_write_str("  IPSR="); usart1_critical_write_hex_u32(ipsr);
    usart1_critical_write_str("\r\n");

    // SCB 故障寄存器（如果溢出引发了后续fault，这些很关键）
    usart1_critical_write_str("CFSR=0x"); usart1_critical_write_hex_u32(SCB->CFSR);
    usart1_critical_write_str("  HFSR=0x"); usart1_critical_write_hex_u32(SCB->HFSR);
    usart1_critical_write_str("  DFSR=0x"); usart1_critical_write_hex_u32(SCB->DFSR);
    usart1_critical_write_str("  AFSR=0x"); usart1_critical_write_hex_u32(SCB->AFSR);
    usart1_critical_write_str("\r\n");

    // 4) 尝试拿到任务栈信息（在溢出后不保证完全可靠，但通常仍可用）
#if (INCLUDE_uxTaskGetStackHighWaterMark == 1)
    UBaseType_t hwm = uxTaskGetStackHighWaterMark(xTask); // 单位：stack words
    usart1_critical_write_str("uxTaskGetStackHighWaterMark(words)=");
    usart1_critical_write_hex_u32((uint32_t)hwm);
    usart1_critical_write_str("\r\n");
#endif

#if (configUSE_TRACE_FACILITY == 1)
    TaskStatus_t ts;
    vTaskGetInfo(xTask, &ts, pdTRUE, eInvalid);
    usart1_critical_write_str("StackBase=0x");
    usart1_critical_write_hex_u32((uint32_t)ts.pxStackBase);
    usart1_critical_write_str("  StackHighWater(words)=");
    usart1_critical_write_hex_u32((uint32_t)ts.usStackHighWaterMark);
    usart1_critical_write_str("\r\n");
#endif

    // 5) 适度 dump 当前 PSP 附近的栈内容（64B~128B 足够；不要过多以免再次问题）
    //    hook 在任务上下文触发，通常 PSP 指向该任务的栈
    if (psp >= 0x20000000u) {  // 粗略校验位于SRAM区（按需调整你的内存映射）
        dump_words("SP dump", (void *)psp, 16); // 16 words = 64 bytes
    }

    // 6) 结束语 & 处理策略
    usart1_critical_write_str("=== SYSTEM HALT ===\r\n");
    usart1_critical_flush();

    // 选择其一：
    //NVIC_SystemReset();              // A) 立即复位
    for(;;) { __NOP(); }             // B) 或者就地停机，方便外部抓log

}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  Log_Pool_Create();     // ★ 在内核 init 后 & start 前创建内存池
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Log_Queue */
  Log_QueueHandle = osMessageQueueNew (20, 4, &Log_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LED_Task */
  LED_TaskHandle = osThreadNew(Start_LED_Task, NULL, &LED_Task_attributes);

  /* creation of RC_Task */
  RC_TaskHandle = osThreadNew(Start_RC_Task, NULL, &RC_Task_attributes);

  /* creation of Log_Task */
  Log_TaskHandle = osThreadNew(Start_Log_Task, NULL, &Log_Task_attributes);

  /* creation of Chassis_Task */
  Chassis_TaskHandle = osThreadNew(Start_Chassis_Task, NULL, &Chassis_Task_attributes);

  /* creation of Gimbal_Task */
  Gimbal_TaskHandle = osThreadNew(Start_Gimbal_Task, NULL, &Gimbal_Task_attributes);

  /* creation of Shoot_Task */
  Shoot_TaskHandle = osThreadNew(Start_Shoot_Task, NULL, &Shoot_Task_attributes);

  /* creation of Referee_Task */
  Referee_TaskHandle = osThreadNew(Start_Referee_Task, NULL, &Referee_Task_attributes);

  /* creation of Gimbal_behavior */
  Gimbal_behaviorHandle = osThreadNew(Start_Gimbal_behave, NULL, &Gimbal_behavior_attributes);

  /* creation of IMU_Task */
  IMU_TaskHandle = osThreadNew(Start_IMU_Task, NULL, &IMU_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  Start_AllRouters();  // 启动所有 CAN Rx 路由任务
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Start_LED_Task */
/**
  * @brief  Function implementing the LED_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_LED_Task */
__weak void Start_LED_Task(void *argument)
{
  /* USER CODE BEGIN Start_LED_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_LED_Task */
}

/* USER CODE BEGIN Header_Start_RC_Task */
/**
* @brief Function implementing the RC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_RC_Task */
__weak void Start_RC_Task(void *argument)
{
  /* USER CODE BEGIN Start_RC_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_RC_Task */
}

/* USER CODE BEGIN Header_Start_Log_Task */
/**
* @brief Function implementing the Log_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Log_Task */
__weak void Start_Log_Task(void *argument)
{
  /* USER CODE BEGIN Start_Log_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_Log_Task */
}

/* USER CODE BEGIN Header_Start_Chassis_Task */
/**
* @brief Function implementing the Chassis_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Chassis_Task */
__weak void Start_Chassis_Task(void *argument)
{
  /* USER CODE BEGIN Start_Chassis_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_Chassis_Task */
}

/* USER CODE BEGIN Header_Start_Gimbal_Task */
/**
* @brief Function implementing the Gimbal_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Gimbal_Task */
__weak void Start_Gimbal_Task(void *argument)
{
  /* USER CODE BEGIN Start_Gimbal_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_Gimbal_Task */
}

/* USER CODE BEGIN Header_Start_Shoot_Task */
/**
* @brief Function implementing the Shoot_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Shoot_Task */
__weak void Start_Shoot_Task(void *argument)
{
  /* USER CODE BEGIN Start_Shoot_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_Shoot_Task */
}

/* USER CODE BEGIN Header_Start_Referee_Task */
/**
* @brief Function implementing the Referee_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Referee_Task */
__weak void Start_Referee_Task(void *argument)
{
  /* USER CODE BEGIN Start_Referee_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_Referee_Task */
}

/* USER CODE BEGIN Header_Start_Gimbal_behave */
/**
* @brief Function implementing the Gimbal_behavior thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Gimbal_behave */
__weak void Start_Gimbal_behave(void *argument)
{
  /* USER CODE BEGIN Start_Gimbal_behave */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_Gimbal_behave */
}

/* USER CODE BEGIN Header_Start_IMU_Task */
/**
* @brief Function implementing the IMU_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_IMU_Task */
__weak void Start_IMU_Task(void *argument)
{
  /* USER CODE BEGIN Start_IMU_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_IMU_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

