#include "communication_task.h"

#include "cmsis_os.h"

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
