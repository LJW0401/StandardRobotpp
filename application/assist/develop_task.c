// 开发新功能时可以使用本任务进行功能测试

#include "develop_task.h"

#include "cmsis_os.h"
#include "data_exchange.h"
#include "signal_generator.h"
#include "usb_task.h"
#include "user_lib.h"

Imu_t * imu;

void develop_task(void const * pvParameters)
{
    // 空闲一段时间
    vTaskDelay(500);

    Subscribe(&imu, "imu_data");

    while (1) {
        OutputPCData.packets[6].data = imu->yaw;
        OutputPCData.packets[7].data = imu->pitch;
        OutputPCData.packets[8].data = imu->roll;

        // code here
        vTaskDelay(1);
    }
}
