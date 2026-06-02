#include "cmsis_os2.h"
#include "_NTFDCAN_Tasks.h"
#include "NTFDCAN_Router.h"

extern "C" void Start_CAN_RxRouter(void *argument); // 由 Router 源文件提供

const osThreadAttr_t attr_canrx1 = {
    .name       = "CAN1_RX_Router",
    .stack_size = 1024 * 4,
    .priority   = (osPriority_t)osPriorityHigh,
};
const osThreadAttr_t attr_canrx2 = {
    .name       = "CAN2_RX_Router",
    .stack_size = 1024 * 4,
    .priority   = (osPriority_t)osPriorityHigh,
};
const osThreadAttr_t attr_canrx3 = {
    .name       = "CAN3_RX_Router",
    .stack_size = 1024 * 4,
    .priority   = (osPriority_t)osPriorityHigh,
};
const osThreadAttr_t attr_cantx1 = {
    .name       = "CAN3_TX_Router",
    .stack_size = 1024 * 4,
    .priority   = (osPriority_t)osPriorityHigh,
};
const osThreadAttr_t attr_cantx2 = {
    .name       = "CAN3_TX_Router",
    .stack_size = 1024 * 4,
    .priority   = (osPriority_t)osPriorityHigh,
};
const osThreadAttr_t attr_cantx3 = {
    .name       = "CAN3_TX_Router",
    .stack_size = 1024 * 4,
    .priority   = (osPriority_t)osPriorityHigh,
};

extern "C" void Start_AllRouters(void)
{
    osThreadNew(Start_CAN_RxRouter, CAN_BUS_ARG(1), &attr_canrx1);
    osThreadNew(Start_CAN_RxRouter, CAN_BUS_ARG(2), &attr_canrx2);
    osThreadNew(Start_CAN_RxRouter, CAN_BUS_ARG(3), &attr_canrx3);
    osThreadNew(Start_CAN_TxRouter, CAN_BUS_ARG(1), &attr_cantx1);
    osThreadNew(Start_CAN_TxRouter, CAN_BUS_ARG(2), &attr_cantx2);
    osThreadNew(Start_CAN_TxRouter, CAN_BUS_ARG(3), &attr_cantx3);
}