/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       支持DJI电机 GM3508 GM2006 GM6020
  *         未来支持小米电机 Cybergear
  *         未来支持达妙电机 DM8009
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *  V2.0.0     Mar-27-2024     Penguin         1. 优化了CAN发送函数，添加新的电机控制函数，解码中将CAN1 CAN2分开，并兼容原有控制。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"

#define CAN_1 hcan1
#define CAN_2 hcan2

/*DJI电机用相关参数定义*/
typedef enum
{
    DJI_200 = 0x200, // 用于3508,2006的电流控制(ID 1~4)
    DJI_1FF = 0x1FF, // 用于3508,2006的电流控制(ID 5~8);6020的电压控制(ID 1~4)
    DJI_2FF = 0x2FF, // 用于6020的电压控制(ID 5~7)
    DJI_1FE = 0x1FE, // 用于6020的电流控制(ID 1~4)
    DJI_2FE = 0x2FE, // 用于6020的电流控制(ID 5~7)
} DJI_Std_ID;

typedef enum
{
    DJI_M1_ID = 0x201,  // 3508/2006电机ID
    DJI_M2_ID = 0x202,  // 3508/2006电机ID
    DJI_M3_ID = 0x203,  // 3508/2006电机ID
    DJI_M4_ID = 0x204,  // 3508/2006电机ID
    DJI_M5_ID = 0x205,  // 3508/2006电机ID (/6020电机ID 不建议使用)
    DJI_M6_ID = 0x206,  // 3508/2006电机ID (/6020电机ID 不建议使用)
    DJI_M7_ID = 0x207,  // 3508/2006电机ID (/6020电机ID 不建议使用)
    DJI_M8_ID = 0x208,  // 3508/2006电机ID (/6020电机ID 不建议使用)
    DJI_M9_ID = 0x209,  // 6020电机ID
    DJI_M10_ID = 0x20A, // 6020电机ID
    DJI_M11_ID = 0x20B, // 6020电机ID
} DJI_Motor_ID;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} DJI_Motor_Measure_Data_t;

typedef struct
{
    CAN_HandleTypeDef *CAN;
    DJI_Std_ID std_id;
    CAN_TxHeaderTypeDef tx_message;
    uint8_t can_send_data[8];
} DJI_Motor_Send_Data_s;

void CAN_CmdDJIMotor(DJI_Motor_Send_Data_s *DJI_Motor_Send_Data, int16_t curr_1, int16_t curr_2, int16_t curr_3, int16_t curr_4);

// void CAN_CmdDJIMotor_200(CAN_HandleTypeDef *CAN, int16_t curr_1, int16_t curr_2, int16_t curr_3, int16_t curr_4);
// void CAN_CmdDJIMotor_1FF(CAN_HandleTypeDef *CAN, int16_t curr_5, int16_t curr_6, int16_t curr_7, int16_t curr_8);
// void CAN_CmdDJIMotor_2FF(CAN_HandleTypeDef *CAN, int16_t volt_5, int16_t volt_6, int16_t volt_7);

/*小米电机用相关参数定义*/

/*达妙电机用相关参数定义*/

/*--------------------------------------------------下面是旧的--------------------------------------------------*/

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2
    /* CAN send and receive ID */
    typedef enum {
        CAN_CHASSIS_ALL_ID = 0x200,
        CAN_3508_M1_ID = 0x201,
        CAN_3508_M2_ID = 0x202,
        CAN_3508_M3_ID = 0x203,
        CAN_3508_M4_ID = 0x204,

        CAN_YAW_MOTOR_ID = 0x205,
        CAN_PIT_MOTOR_ID = 0x206,
        CAN_TRIGGER_MOTOR_ID = 0x207,
        CAN_GIMBAL_ALL_ID = 0x1FF,

    } can_msg_id_e;

// rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

/**
 * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
 * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000]
 * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
 * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
 * @param[in]      rev: (0x208) reserve motor control current
 * @retval         none
 */
/**
 * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
 * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
 * @param[in]      rev: (0x208) 保留，电机控制电流
 * @retval         none
 */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);

/**
 * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
 * @param[in]      none
 * @retval         none
 */
extern void CAN_cmd_chassis_reset_ID(void);

/**
 * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
 * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384]
 * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384]
 * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384]
 * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384]
 * @retval         none
 */
/**
 * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
 * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
 * @brief          return the yaw 6020 motor data point
 * @param[in]      none
 * @retval         motor data point
 */
/**
 * @brief          返回yaw 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
 * @brief          return the pitch 6020 motor data point
 * @param[in]      none
 * @retval         motor data point
 */
/**
 * @brief          返回pitch 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
 * @brief          return the trigger 2006 motor data point
 * @param[in]      none
 * @retval         motor data point
 */
/**
 * @brief          返回拨弹电机 2006电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
 * @brief          return the chassis 3508 motor data point
 * @param[in]      i: motor number,range [0,3]
 * @retval         motor data point
 */
/**
 * @brief          返回底盘电机 3508电机数据指针
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

#endif
