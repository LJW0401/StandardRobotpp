/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_power_control.c/h
  * @brief      底盘功率控制
  * @history
  *  Version    Date            Author          Modification
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"

#define POWER_LIMIT         80.0f
#define WARNING_POWER       40.0f   
#define WARNING_POWER_BUFF  50.0f   

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, 
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f

uint16_t power_model_predict;//P pre
uint16_t 






// 发送期望转速之后产生的功率预测
a=0,b=0,c=0;
for(int i=0;i<4;i++){
Pid_Typedef *P = &(power_limit->pidChassisWheelSpeed[i]);
a += RM3508_R * P->SetPoint * P->SetPoint * (power_limit->K[i]) * (power_limit-
>K[i]);
b += P->SetPoint * (2*RM3508_R*power_limit->K[i]*power_limit->M[i] +
RM3508_K*power_limit->K[i]*power_limit->Speed_Now[i]);
c += (RM3508_R*power_limit->M[i]*power_limit->M[i] + RM3508_K*power_limit-
>M[i]*power_limit->Speed_Now[i]);
}
c += RM3508_P0;
//这里相当于k_c=1时，预测功率
power_limit->predict_send_power = LIMIT_MAX_MIN(a + b + c,1000,-1000);//过滤转速预测异常数

//power_limit->predict_send_power = a + b + c;
// 超功率衰减
if(power_limit->predict_send_power > power_limit->set_power)
{
// 模型衰减计算
if(b*b < 4*(c - power_limit->set_power)*a){// b方小于4ac：没有解
power_limit->k_c = LIMIT_MAX_MIN(-b/(2*a) , 1.0,0.0);// -b/(2a)
}else{
float32_t sqrt_result;
arm_sqrt_f32(b * b - 4*(c - power_limit->set_power)*a,&sqrt_result);
power_limit->k_c = LIMIT_MAX_MIN((-b + sqrt_result) / 2 / a,1.0 ,0.0);
}
//衰减

for(int i=0;i<4;i++){
power_limit->pidChassisWheelSpeed[i].SetPoint *= power_limit->k_c;//衰减
}
}
//PI控制器计算目标电流
for(int i=0;i<4;i++){
power_limit->Current_set[i] = PID_Calc(&(power_limit-
>pidChassisWheelSpeed[i]),power_limit->Speed_Now[i]);
}