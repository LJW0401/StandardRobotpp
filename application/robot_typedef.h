#ifndef ROBOT_TYPEDEF_H
#define ROBOT_TYPEDEF_H

// 可用底盘类型
#define CHASSIS_MECANUM_WHEEL 0   // 麦克纳姆轮底盘
#define CHASSIS_OMNI_WHEEL 1      // 全向轮底盘
#define CHASSIS_STEERING_WHEEL 2  // 舵轮底盘
#define CHASSIS_BALANCE 3         // 平衡底盘

// 可用云台类型
#define GIMBAL_NONE 0            // yaw轴云台
#define GIMBAL_YAW 1             // yaw轴云台
#define GIMBAL_PITCH 2           // pitch轴云台
#define GIMBAL_YAW_PITCH 3       // yaw轴+pitch轴云台
#define GIMBAL_YAW_YAW_PITCH 4   // 大小yaw轴+pitch轴云台
#define GIMBAL_YAW_PITCH_ROLL 5  // 三轴云台

// 控制链路类型
#define DBUS_LINK 0                // DBUS控制链路
#define IMAGE_TRANSMISSION_LINK 1  // 图传控制链路

// 可用的发射机构类型
#define SHOOT_NONE 0       // 无发射机构
#define SHOOT_FRIC 1       // 摩擦轮发射机构
#define SHOOT_PNEUMATIC 2  // 气动发射机构

// 控制类型（板间通信时用到）
#define CHASSIS_ONLY 0        // 只控制底盘
#define GIMBAL_ONLY 1         // 只控制云台
#define CHASSIS_AND_GIMBAL 2  // 控制底盘和云台

// 可用C板ID
#define BoardC1 1  //C1板
#define BoardC2 2  //C2板
#define BoardC3 3  //C3板
#define BoardC4 4  //C4板

#endif /* ROBOT_TYPEDEF_H */
