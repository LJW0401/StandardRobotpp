# StandardRobot++

## 简介
本项目的计划是基于 DJI StandardRobot 的基础上改造成一个更适合北极熊uu们的通用型机器人代码框架

主要内容框架如下：
![main framework](./doc/pic/mainframework.svg)

## 开发进度
- 校准
  - [ ] 云台校准
  - [ ] 陀螺仪校准
  - [ ] 底盘校准
- 底盘类型
  - [ ] 麦轮底盘
  - [x] 全向轮底盘
  - [ ] 舵轮底盘
  - [ ] 平衡底盘
- 底盘模式
  - [ ] 云台跟随
  - [ ] 独立于云台运动
  - [ ] 小陀螺
- 云台模式
  - [ ] 陀螺仪绝对角度控制
  - [ ] 电机编码角相对角度控制
- 射击模式
  - [ ] 单发
  - [ ] 连发

## 如何使用本框架

在 [robot_param.h](./application/robot_param.h) 中填写机器人参数