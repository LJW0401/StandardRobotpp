/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal_yaw_pitch.c/h
  * @brief      yaw_pitch云台控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *  
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#include "gimbal_yaw_pitch_direct.h"
#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT)
Gimbal_s gimbal_direct;
PID_t gimbal_direct_pid;
/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void InitGimbal(void) 
{
  //step1 获取所有所需变量指针
   gimbal_direct.rc = get_remote_control_point(); 
   gimbal_direct.imu=GetImuDataPoint();=
   //step2 置零所有值
   gimbal_direct.reference.pitch=0;
   gimbal_direct.reference.yaw=0;

   gimbal_direct.feedback.pitch=0;
   gimbal_direct.feedback.yaw=0;

   gimbal_direct.upper_limit.pitch=GIMBAL_UPPER_LIMIT_PITCH;
   gimbal_direct.upper_limit.yaw=GIMBAL_UPPER_LIMIT_YAW;

   gimbal_direct.lower_limit.pitch=GIMBAL_LOWER_LIMIT_PITCH;
   gimbal_direct.lower_limit.yaw=GIMBAL_LOWER_LIMIT_YAW;
   //step3 PID数据清零，设置PID参数
   const static fp32 gimbal_yaw_angle[3]={KP_GIMBAL_YAW_ANGLE,KI_GIMBAL_YAW_ANGLE,KD_GIMBAL_YAW_ANGLE};
   const static fp32 gimbal_yaw_velocity[3]={KP_GIMBAL_YAW_VELOCITY,KI_GIMBAL_YAW_VELOCITY,KD_GIMBAL_YAW_VELOCITY};
   
   const static fp32 gimbal_pitch_angle[3]={KP_GIMBAL_PITCH_ANGLE,KI_GIMBAL_PITCH_ANGLE,KD_GIMBAL_PITCH_ANGLE};
   const static fp32 gimbal_pitch_velocity[3]={KP_GIMBAL_PITCH_VELOCITY,KI_GIMBAL_PITCH_VELOCITY,KD_GIMBAL_PITCH_VELOCITY};

   PID_init(&gimbal_direct_pid.yaw_angle,PID_POSITION,gimbal_yaw_angle,MAX_OUT_GIMBAL_YAW_ANGLE,MAX_IOUT_GIMBAL_YAW_ANGLE);
   PID_init(&gimbal_direct_pid.yaw_velocity,PID_POSITION,gimbal_yaw_velocity,MAX_OUT_GIMBAL_YAW_VELOCITY,MAX_IOUT_GIMBAL_YAW_VELOCITY);

   PID_init(&gimbal_direct_pid.pitch_angle,PID_POSITION,gimbal_pitch_angle,MAX_OUT_GIMBAL_PITCH_ANGLE,MAX_IOUT_GIMBAL_PITCH_ANGLE);
   PID_init(&gimbal_direct_pid.pitch_velocity,PID_POSITION,gimbal_pitch_velocity,MAX_OUT_GIMBAL_PITCH_VELOCITY,MAX_IOUT_GIMBAL_PITCH_VELOCITY);

   //step4 初始化电机
   MotorInit(&gimbal_direct.yaw,GIMBAL_DIRECT_YAW_ID,GIMBAL_DIRECT_YAW_CAN,GIMBAL_DIRECT_YAW_MOTOR_TYPE,GIMBAL_DIRECT_YAW_DIRECTION,GIMBAL_DIRECT_YAW_REDUCTION_RATIO,GIMBAL_DIRECT_YAW_MODE);
   MotorInit(&gimbal_direct.pitch,GIMBAL_DIRECT_PITCH_ID,GIMBAL_DIRECT_PITCH_CAN,GIMBAL_DIRECT_PITCH_MOTOR_TYPE,GIMBAL_DIRECT_PITCH_DIRECTION,GIMBAL_DIRECT_PITCH_REDUCTION_RATIO,GIMBAL_DIRECT_PITCH_MODE);
}
/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void SetGimbalMode(void)
{

}

/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void GimbalObserver(void) {}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void GimbalReference(void) {}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalConsole(void) {}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void SendGimbalCmd(void) {}

#endif  // GIMBAL_YAW_PITCH
