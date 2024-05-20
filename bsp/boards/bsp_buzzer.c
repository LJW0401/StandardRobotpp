#include "bsp_buzzer.h"
#include "main.h"
extern TIM_HandleTypeDef htim4;
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

void buzzer_note(uint16_t note)
{
    // 禁用定时器
    __HAL_TIM_DISABLE(&htim4);
    
    // 重置定时器计数器
    htim4.Instance->CNT = 0;
    
    // 设置自动重装载寄存器（ARR），以控制PWM信号的频率
    htim4.Instance->ARR = (21000 / note - 1) * 1u;
    
    // 设置比较寄存器（CCR3），以控制PWM信号的占空比
    htim4.Instance->CCR3 = (10500 / note - 1) * 1u;
    
    // 重新启用定时器
    __HAL_TIM_ENABLE(&htim4);
    
    // 启动PWM信号
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}
