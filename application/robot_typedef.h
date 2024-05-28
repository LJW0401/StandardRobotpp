#ifndef ROBOT_TYPEDEF_H
#define ROBOT_TYPEDEF_H

// clang-format off
// 可用底盘硬件类型
#define CHASSIS_NONE            0  // 无底盘
#define CHASSIS_MECANUM_WHEEL   1  // 麦克纳姆轮底盘
#define CHASSIS_OMNI_WHEEL      2  // 全向轮底盘
#define CHASSIS_STEERING_WHEEL  3  // 舵轮底盘
#define CHASSIS_BALANCE         4  // 平衡底盘

// 可用云台硬件类型
#define GIMBAL_NONE                0  // 无云台
#define GIMBAL_YAW_PITCH_DIRECT    1  // yaw-pitch电机直连云台

// 可用的发射机构硬件类型
#define SHOOT_NONE               0  // 无发射机构
#define SHOOT_FRIC_TRIGGER       1  // 摩擦轮+拨弹盘发射机构
#define SHOOT_PNEUMATIC_TRIGGER  2  // 气动+拨弹盘发射机构

// 可用机械臂硬件类型
#define MECHANICAL_ARM_NONE              0  // 无机械臂
#define MECHANICAL_ARM_PENGUIN_MINI_ARM  1  // 企鹅mini机械臂

// 可用自定义控制器硬件类型
#define CUSTOM_CONTROLLER_NONE 0          // 无自定义控制器
#define CUSTOM_CONTROLLER_PENGUIN_MINI 1  // 企鹅mini自定义控制器

// 控制类型（板间通信时用到）
#define CHASSIS_ONLY       0  // 只控制底盘
#define GIMBAL_ONLY        1  // 只控制云台
#define CHASSIS_AND_GIMBAL 2  // 控制底盘和云台

// 可用C板ID
#define BoardC1 1  //C1板
#define BoardC2 2  //C2板
#define BoardC3 3  //C3板
#define BoardC4 4  //C4板

// 可用调参模式
#define TUNING_NONE     0
#define TUNING_CHASSIS  1
#define TUNING_GIMBAL   2
#define TUNING_SHOOT    3
// clang-format on

// 可用电机类型
typedef enum __MotorType {
    DJI_M2006 = 0,
    DJI_M3508,
    DJI_M6020,
    CYBERGEAR_MOTOR,
    DM_8009,
    MF_9025,
} MotorType_e;

#endif /* ROBOT_TYPEDEF_H */
