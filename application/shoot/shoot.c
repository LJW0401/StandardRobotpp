/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       shoot.c/h
  * @brief      射击功能。
  * @note
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

#include "shoot.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_communication.h"
#include "gimbal_console.h"
#include "detect_task.h"
#include "pid.h"

// PID相关宏定义
#define FRIC_KP 10.0f
#define FRIC_KI 0.0f
#define FRIC_KD 0.0f
#define FRIC_MAX_IOUT 1.0f
#define FRIC_MAX_OUT 3000.0f

Shoot_s shoot = {
    .mode = LOAD_STOP,

    .shoot_frequency = 10,
    .dangle = 2 * PI / BULLET_NUM,

    .trigger_pid = {
        .mode = PID_POSITION,
        .Kp = 0.1,
        .Ki = 0,
        .Kd = 0,
        .max_iout = 10,
        .max_out = 1000,
    },
};

const fp32 fric_Kpid[3] = {FRIC_KP, FRIC_KI, FRIC_KD};


/*-------------------- Init --------------------*/
#if (SHOOT_TYPE == SHOOT_NONE)
#elif (SHOOT_TYPE == SHOOT_FRIC)
#elif (SHOOT_TYPE == SHOOT_PNEUMATIC)
#endif

/*-------------------- Observe --------------------*/
#if (SHOOT_TYPE == SHOOT_NONE)
#elif (SHOOT_TYPE == SHOOT_FRIC)
#elif (SHOOT_TYPE == SHOOT_PNEUMATIC)
#endif

/*-------------------- Reference --------------------*/
#if (SHOOT_TYPE == SHOOT_NONE)
#elif (SHOOT_TYPE == SHOOT_FRIC)
#elif (SHOOT_TYPE == SHOOT_PNEUMATIC)
#endif

/*-------------------- Cmd --------------------*/
#if (SHOOT_TYPE == SHOOT_NONE)
#elif (SHOOT_TYPE == SHOOT_FRIC)
#elif (SHOOT_TYPE == SHOOT_PNEUMATIC)
#endif
