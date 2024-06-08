// 开发新功能时可以使用本任务进行功能测试

#include "develop_task.h"

#include "cmsis_os.h"
#include "signal_generator.h"
#include "user_lib.h"
#include "usb_task.h"

void develop_task(void const * pvParameters)
{
    // 空闲一段时间
    vTaskDelay(10);

    while (1) {
        float in = GenerateSawtoothWave(-M_PI, M_PI, 3);
        float out = ThetaRangeLimit(in, -2.0f, 1.55f, 1);

        OutputPCData.packets[20].data = in;
        OutputPCData.packets[21].data = out;
        // code here
        vTaskDelay(1);
    }
}
