#include "communication_task.h"

#include "cmsis_os.h"
#include "communication.h"

// 任务相关时间
#define COMMUNICATION_TASK_INIT_TIME 100
#define COMMUNICATION_TASK_TIME_MS 2

void communication_task(void const * pvParameters)
{
    // 空闲一段时间
    vTaskDelay(COMMUNICATION_TASK_INIT_TIME);
    while (1) {
        /* code */

        // 系统延时
        vTaskDelay(COMMUNICATION_TASK_TIME_MS);
    }
}

/**
 * @brief USART1 中断处理函数
 * @param  
 */
void USART1_IRQHandler(void) { __Uart2_IRQHandler(); }
