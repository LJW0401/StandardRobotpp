// 开发新功能时可以使用本任务进行功能测试

#include "develop_task.h"

#include "cmsis_os.h"
void develop_task(void const * pvParameters)
{
    // 空闲一段时间
    vTaskDelay(10);

    while (1) {
        // code here
        vTaskDelay(2);
    }
}
