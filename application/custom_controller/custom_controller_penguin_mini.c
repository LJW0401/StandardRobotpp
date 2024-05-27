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

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void CustomControllerInit(void) {}

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
void CustomControllerObserver(void) {}

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
void CustomControllerConsole(void) {}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void CustomControllerSendCmd(void) {}

#endif
