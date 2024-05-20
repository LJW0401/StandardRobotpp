

#include "music_task.h"

#include "music_you.h"
#include "music_1.h"
#include "cmsis_os.h"
#include "music.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */

__weak void MusicInit(void);
__weak void MusicPlay(void);

void music_task(void const * pvParameters)
{
    // 空闲一段时间
    vTaskDelay(MUSIC_TASK_INIT_TIME);
    // 初始化底盘
    MusicInit();

    while (1) {
        MusicPlay();
        // 系统延时
        vTaskDelay(MUSIC_TASK_TIME_MS);
    }
}

__weak void MusicInit(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
    MusicYouInit();
    // Music1Init();
}
__weak void MusicPlay(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
    MusicYouPlay();
    // Music1Play();
}
