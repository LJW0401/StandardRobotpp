#ifndef ROBOT_TYPEDEF_H
#define ROBOT_TYPEDEF_H

// clang-format off
// 可用底盘类型
#define CHASSIS_NONE            0  // 无底盘
#define CHASSIS_MECANUM_WHEEL   1  // 麦克纳姆轮底盘
#define CHASSIS_OMNI_WHEEL      2  // 全向轮底盘
#define CHASSIS_STEERING_WHEEL  3  // 舵轮底盘
#define CHASSIS_BALANCE         4  // 平衡底盘

// 可用云台类型
#define GIMBAL_NONE           0  // 无云台
#define GIMBAL_YAW            1  // yaw轴云台
#define GIMBAL_PITCH          2  // pitch轴云台
#define GIMBAL_YAW_PITCH      3  // yaw轴+pitch轴云台
#define GIMBAL_YAW_YAW_PITCH  4  // 大小yaw轴+pitch轴云台
#define GIMBAL_YAW_PITCH_ROLL 5  // 三轴云台

// 可用的发射机构类型
#define SHOOT_NONE             0  // 无发射机构
#define SHOOT_FRIC             1  // 摩擦轮发射机构
#define SHOOT_PNEUMATIC        2  // 气动发射机构

// 可用机械臂类型
#define MECHANICAL_ARM_NONE   0  // 无机械臂
#define MECHANICAL_ARM_3_AXIS 3  // 3轴机械臂
#define MECHANICAL_ARM_4_AXIS 4  // 4轴机械臂
#define MECHANICAL_ARM_5_AXIS 5  // 5轴机械臂
#define MECHANICAL_ARM_6_AXIS 6  // 6轴机械臂
#define MECHANICAL_ARM_7_AXIS 7  // 7轴机械臂

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
