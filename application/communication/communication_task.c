#include "communication_task.h"

#include "bsp_uart.h"
#include "cmsis_os.h"
#include "communication.h"

// 任务相关时间
#define COMMUNICATION_TASK_INIT_TIME 100
#define COMMUNICATION_TASK_TIME_MS 2

void communication_task(void const * pvParameters)
{
    Usart1Init();
    // 空闲一段时间
    vTaskDelay(COMMUNICATION_TASK_INIT_TIME);
    while (1) {
        DataPack("Hello World", 13, 0);

        UartSendTxMessage(&huart1, (uint8_t *)(&BOARD_TX_DATA), sizeof(BOARD_TX_DATA), 100);

        DataUnpack();

        // 系统延时
        vTaskDelay(COMMUNICATION_TASK_TIME_MS);
    }
}
