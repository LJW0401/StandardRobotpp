/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       custom_controller_penguin_mini.c/h
  * @brief      企鹅mini自定义控制器。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     May-27-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/
#include "custom_controller_penguin_mini.h"
#if (CUSTOM_CONTROLLER_TYPE == CUSTOM_CONTROLLER_PENGUIN_MINI)
#include "CAN_communication.h"
#include "usb_task.h"
#include "user_lib.h"

#define BIG_ARM_DATA_ID 1
#define SMALL_ARM_DATA_ID 2

#define MAIN_DATA_ID_1 1

CustomController_s CUSTOM_CONTROLLER;

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void CustomControllerInit(void) { CUSTOM_CONTROLLER.imu = GetImuDataPoint(); }

/*-------------------- Handle exception --------------------*/

/**
 * @brief          异常处理
 * @param[in]      none
 * @retval         none
 */
void CustomControllerHandleException(void) {}

/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void CustomControllerSetMode(void) {}

/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void CustomControllerObserver(void)
{
    // OutputPCData.packets[17].data = uint_to_float(CUSTOM_CONTROLLER.ctrl_data.yaw, -M_PI, M_PI, 16);
    // OutputPCData.packets[18].data = uint_to_float(CUSTOM_CONTROLLER.ctrl_data.big_arm_pitch, -M_PI_2, M_PI_2, 16);
    // OutputPCData.packets[19].data = CUSTOM_CONTROLLER.ctrl_data.small_arm_pitch;
    // OutputPCData.packets[20].data = CUSTOM_CONTROLLER.ctrl_data.small_arm_roll;
}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void CustomControllerReference(void) {}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void CustomControllerConsole(void)
{
#if (__SELF_BOARD_ID == MAIN_BOARD_ID)
    CUSTOM_CONTROLLER.ctrl_data.yaw = GetOtherBoardDataUint16(BIG_ARM_DATA_ID, 0);
    CUSTOM_CONTROLLER.ctrl_data.big_arm_pitch = GetOtherBoardDataUint16(BIG_ARM_DATA_ID, 1);
    CUSTOM_CONTROLLER.ctrl_data.small_arm_pitch = GetOtherBoardDataUint16(SMALL_ARM_DATA_ID, 0);
    CUSTOM_CONTROLLER.ctrl_data.small_arm_roll = GetOtherBoardDataUint16(SMALL_ARM_DATA_ID, 1);
#elif (__SELF_BOARD_ID == BIG_ARM_BOARD_ID)
    // clang-format off
    CUSTOM_CONTROLLER.ctrl_data.yaw = 
        float_to_uint(CUSTOM_CONTROLLER.imu->yaw, -M_PI, M_PI, 16);
    CUSTOM_CONTROLLER.ctrl_data.big_arm_pitch =
        float_to_uint(CUSTOM_CONTROLLER.imu->pitch, -M_PI_2, M_PI_2, 16);
    // clang-format on
#elif (__SELF_BOARD_ID == SMALL_ARM_BOARD_ID)
    // clang-format off
    CUSTOM_CONTROLLER.ctrl_data.small_arm_pitch = 
        float_to_uint(CUSTOM_CONTROLLER.imu->pitch, -M_PI_2, M_PI_2, 16);
    CUSTOM_CONTROLLER.ctrl_data.small_arm_roll =
        float_to_uint(CUSTOM_CONTROLLER.imu->roll, -M_PI, M_PI, 16);
    // clang-format on
#endif
}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void CustomControllerSendCmd(void)
{
#if (__SELF_BOARD_ID == MAIN_BOARD_ID)
    // clang-format off
    CanSendUint16DataToBoard(2, MAIN_DATA_ID_1, CTRL_BOARD_ID,
        CUSTOM_CONTROLLER.ctrl_data.yaw,
        CUSTOM_CONTROLLER.ctrl_data.big_arm_pitch,
        CUSTOM_CONTROLLER.ctrl_data.small_arm_pitch,
        CUSTOM_CONTROLLER.ctrl_data.small_arm_roll);
    // clang-format on
#elif (__SELF_BOARD_ID == BIG_ARM_BOARD_ID)
    // clang-format off
    CanSendUint16DataToBoard(2, BIG_ARM_DATA_ID, MAIN_BOARD_ID, 
        CUSTOM_CONTROLLER.ctrl_data.yaw,
        CUSTOM_CONTROLLER.ctrl_data.big_arm_pitch, 
        0, 0);
    // clang-format on
#elif (__SELF_BOARD_ID == SMALL_ARM_BOARD_ID)
    // clang-format off
    CanSendUint16DataToBoard(2, SMALL_ARM_DATA_ID, MAIN_BOARD_ID, 
        CUSTOM_CONTROLLER.ctrl_data.small_arm_pitch,
        CUSTOM_CONTROLLER.ctrl_data.small_arm_roll, 
        0, 0);
    // clang-format on
#endif
}

#endif
