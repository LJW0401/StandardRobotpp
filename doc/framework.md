# 框架
## 总体介绍
### 机器人参数
机器人的主要选项在 [robot_typedef](../application/robot_typedef.h) 中定义。

在[robot_param](../application/robot_param.h)中导入个性化定义的每个机器人具体的物理参数，包括但不限于控制中要用到的物理参数、PID参数等等。

通过这种做法达到机器人代码通用化的目的，只需替换不同机器人的 `robot_param.h` 文件即可实现不同车型的适配。

### 模块划分
本框架主要划分为以下几个模块。
- assist 辅助模块
- chassis 底盘模块
- gimbal 云台模块
- IMU 陀螺仪模块
- mechanical_arm 机械臂模块
- other 其他乱七八糟的东西
- referee 裁判系统模块
- robot_cmd 机器人控制模块
- shoot 射击模块

### 模块控制
不同的模块由其对应的控制任务进行控制，如底盘为chassis_task，云台为gimbal_task，射击为shoot_task ... 

#### * 层级结构
每个模块都做成了三层的上中下结构，上层可以调用下层的数据和函数。防止了上下层之间循环调用造成的混乱局面。
- **上层**：task任务层\
    该层中提供freertos的任务函数接口，用来创建本模块的控制任务。同时创建各个任务执行函数的弱定义空函数，当无需用到此模块时使用。
- **中层**：任务执行层\
    该层为任务函数的实体部分，根据不同的硬件条件有不同的执行方式。需对每种硬件条件编写不同的执行代码以实现适配，并通过预编译的方式选择编译对应硬件的控制代码。
- **下层**：任务根基层\
    该层为整个模块的根基部分，定义了在不同硬件条件下都需要用到的类型、函数、变量等等。

以平衡底盘为例，其 ***task任务层*** 为 [chassis_task](../application/chassis/chassis_task.c)， ***任务执行层*** 为 [chassis_balance](../application/chassis/chassis_balance.c)， ***任务根基层*** 为 [chassis](../application/chassis/chassis.c)


#### * 步骤划分
每个控制任务都可以分成以下几个步骤：
1. **发布数据 (xxxPublish)**：发布本模块的数据以供其他模块使用。
2. **初始化 (xxxInit)**：在进入任务循环前先行对该任务所需要的各种参数进行初始化。
3. **获取反馈 (xxxObserver)**：获取各种传感器的反馈数据，以便接下来进行控制处理。
4. **异常处理 (xxxHandleException)**：在任务循环中遇到异常情况时如果有处理方法则进行异常处理（通常为电机失能等等），若无法处理且会造成危险的话则尝试进行报警。
5. **模式设置 (xxxSetMode)** ：设置对应模块的模式，在不同模式下模块会有不同的动作。
6. **更新目标 (xxxReference)**：更新各个目标值，作为控制的目标结果。
7. **计算控制量 (xxxConsole)**：计算出各个执行机构的控制量使得模块效果达到目标。
8. **发送控制量 (xxxSendCmd)**：将控制量发送给执行机构来执行。

<details>
    <summary>整体的一个任务处理顺序</summary>

```C
ChassisInit();
while (1) {
    ChassisObserver();
    ChassisHandleException();
    ChassisSetMode();
    ChassisReference();
    ChassisConsole();
    ChassisSendCmd();
    vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }
```

</details>

#### * 硬件选择与代码编译
每个模块都有不同的硬件条件，比如底盘模块就有平衡底盘、麦轮底盘、全向轮底盘等等。对于不同的硬件有不同的控制逻辑，也使得上文中的每个步骤的具体内容有所不同。

为了应对这种情况，本框架采取了预编译指令的方式，既保证了对不同硬件的支持，又不会因为各种硬件之间的冲突导致编译失败。

具体的实现如下：
- 在 [robot_typedef](../application/robot_typedef.h) 中定义不同硬件的类型ID，如：

      ```C
      #define CHASSIS_NONE            0  // 无底盘
      #define CHASSIS_MECANUM_WHEEL   1  // 麦克纳姆轮底盘
      #define CHASSIS_OMNI_WHEEL      2  // 全向轮底盘
      #define CHASSIS_STEERING_WHEEL  3  // 舵轮底盘
      #define CHASSIS_BALANCE         4  // 平衡底盘
      ```

- 在**机器人参数配置文件**中选择模块的硬件类型，如：
    
    ```C
    #define CHASSIS_TYPE CHASSIS_BALANCE
    ```

  就代表选择平衡底盘作为底盘的硬件部分
- 在不同硬件的代码部分使用预编译指令判断是否需要编译（直接用`#if() #endif` 将整个代码部分都包裹起来）
  
    <details>
        <summary>平衡底盘的源文件部分</summary>

    ```C
    #include "chassis_balance.h"
    #if (CHASSIS_TYPE == CHASSIS_BALANCE)
    #include "CAN_communication.h"
    ...

    void ChassisInit(void);
    void ChassisHandleException(void);
    void ChassisSetMode(void);
    void ChassisObserver(void);
    void ChassisReference(void);
    void ChassisConsole(void);
    void ChassisSendCmd(void);
    ...

    #endif /* CHASSIS_BALANCE */
    ```

    </details>
    
    \
    头文件要注意的稍微多一点，
    1. 必须要在最上面引用 `robot_param` 机器人配置文件。
    2. `#ifndef CHASSIS_BALANCE_H` 应当作用于整个文件，所以要写在最上方。
    <details>
        <summary>平衡底盘的头文件部分</summary>

    ```C
    #ifndef CHASSIS_BALANCE_H
    #define CHASSIS_BALANCE_H
    #include "robot_param.h"

    #if (CHASSIS_TYPE == CHASSIS_BALANCE)
    #include "chassis.h"
    #include "motor.h"
    #include "remote_control.h"
    #include "struct_typedef.h"
    ...
    extern void ChassisInit(void);
    extern void ChassisHandleException(void);
    extern void ChassisSetMode(void);
    extern void ChassisObserver(void);
    extern void ChassisReference(void);
    extern void ChassisConsole(void);
    extern void ChassisSendCmd(void);
    ...
    #endif /* CHASSIS_BALANCE */
    #endif /* CHASSIS_BALANCE_H */
    ```

    </details>

- 当不需要某个模块时为了减少资源的消耗（开启太多任务），还需要在 [freertos](../Src/freertos.c) 中对任务进行预编译处理。

#### * 变量定义
在每个模块中有个模块同名的全局结构体变量用来存储所需要使用到的数据。
<details>
    <summary>结构体主要结构</summary>

```C
typedef struct
{
    const RC_ctrl_t * rc;  // 底盘使用的遥控器指针
    ChassisMode_e mode;    // 底盘模式
    ChassisState_e state;  // 底盘状态
    uint8_t error_code;    // 底盘错误代码

    /*-------------------- Motors --------------------*/
    Motor_s motor[4];
    /*-------------------- Values --------------------*/
    Values_t ref;    // 期望值
    Values_t fdb;     // 状态值
    Values_t upper_limit;  // 上限值
    Values_t lower_limit;  // 下限值

    PID_t pid;  // PID控制器
} Chassis_s;
```
</details>

\
主要变量的作用详解：
- rc ：用于获取遥控器传来的控制量
- mode ：用来设置模块的各个模式
- state ：用来表示模块的工作状态
- error_code ：用来记录运行过程中遇到的错误
- motor ：模块运行所需要用到的电机
- ref ：各控制量的期望值
- fdb ：各控制量的实际状态值也就是反馈值
- upper_limit ：各控制量的上限值
- lower_limit ：各控制量的下限值
- pid ：PID控制器

除了以上提到的变量还可根据实际需求添加其他需要用到的变量。
> 为了实现控制、执行、感知之间的解耦，要认真编写 `Values_t` 结构体，里面量用于在控制过程中的运算。\
> 所有传感器接收到的反馈都要传入 `fdb` 变量，控制用的目标值都传入 `ref` 变量。\
> 最后计算出来的直接控制量给电机赋值（目前的执行机构仅有电机，后续可能会增加其他执行机构）

### 示例
如果要将代码用于平衡步兵机器人
1. 我需要在 [robot_param_balanced_infantry.h](../application/robot_param_balanced_infantry.h) 文件中定义实现控制所需要的各种物理量。
2. 我需要在 [robot_param.h](../application/robot_param.h) 中导入 `robot_param_balanced_infantry.h` 的机器人参数。如下2行：
    ```C
    ...code...

    //导入具体的机器人参数配置文件
    #include "robot_param_balanced_infantry.h"

    ...code...

    ```
<details>
    <summary>参数配件文件框架</summary>

```C
/**
  * @file       robot_param_xxx.h
  * @brief      这里是xxx机器人参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define CHASSIS_TYPE ...    // 选择底盘类型
#define GIMBAL_TYPE ...     // 选择云台类型
#define SHOOT_TYPE ...      // 选择发射机构类型
#define CONTROL_TYPE ...    // 选择控制类型

typedef enum {
...
} MotorId_e;

/*-------------------- Chassis --------------------*/
//physical parameters ---------------------
//upper_limit parameters ---------------------
//lower_limit parameters ---------------------
//PID parameters ---------------------
...
/*-------------------- Gimbal --------------------*/
//physical parameters ---------------------
//PID parameters ---------------------
...
/*-------------------- Shoot --------------------*/
//physical parameters ---------------------
//PID parameters ---------------------
...

#endif /* INCLUDED_ROBOT_PARAM_H */
```
</details>

