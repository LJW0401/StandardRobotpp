# StandardRobot++
<div align=center>

![Logo](./doc/pic/Logo6.png)

[![Author](https://img.shields.io/badge/Author-小企鹅-orange.svg)](https://gitee.com/Ljw0401)
[![Author](https://img.shields.io/badge/Author-YZX-orange.svg)](https://gitee.com/yuan-zaixiu)

![language](https://img.shields.io/badge/language-C-blue.svg)
![license](https://img.shields.io/badge/license-MIT-green.svg)

[![commit](https://svg.hamm.cn/gitee.svg?user=SMBU-POLARBEAR&project=StandardRobotpp&type=commit)](https://gitee.com/SMBU-POLARBEAR/StandardRobotpp)
[![fork](https://gitee.com/SMBU-POLARBEAR/StandardRobotpp/badge/fork.svg?theme=dark)](https://gitee.com/SMBU-POLARBEAR/StandardRobotpp)
[![star](https://gitee.com/SMBU-POLARBEAR/StandardRobotpp/badge/star.svg?theme=dark)](https://gitee.com/SMBU-POLARBEAR/StandardRobotpp)
[![release](https://svg.hamm.cn/gitee.svg?user=SMBU-POLARBEAR&project=StandardRobotpp&type=release)](https://gitee.com/SMBU-POLARBEAR/StandardRobotpp)

</div>

## 简介
> 也许不是最好的？但一定是最适合上手的电控开源！

本项目的计划是基于 DJI StandardRobot 的基础上改造成一个更适合北极熊uu们的通用型机器人代码框架。

本框架致力于实现不同类型的机器人的代码通用化，只需要选择底盘云台的类型，修改一下对应物理参数即可实现对机器人的适配。

## 模块支持
详细信息请参考 [StandardRobot++ 框架](./doc/framework.md)
### 主要模块
- **Chassis**\
  底盘模块：
  - [x] 麦轮底盘
  - [ ] 全向轮底盘
  - [ ] 舵轮底盘
  - [ ] 平衡底盘
  
  详细信息请参考 [CHASSIS_README](./application/chassis/CHASSIS_README.md)

- **Gimbal**\
  云台模块：
  - [ ] yaw-pitch 直连云台
  - [ ] yaw直连-pitch丝杆 云台
  - [ ] yaw-pitch 丝杆云台
  
  详细信息请参考 [GIMBAL_README](./application/gimbal/GIMBAL_README.md)

- **Shoot**\
  发射机构模块：
  - [ ] 摩擦轮+拨弹盘
  - [ ] 摩擦轮+弹鼓+推杆

  详细信息请参考 [SHOOT_README](./application/shoot/SHOOT_README.md)

<!-- - **Mechanical arm**\
  支持以下机械臂类型：
  
  详细信息请参考 [ARM_README](./application/arm/ARM_README.md) -->

## 开发工具
使用VSCode作为代码编辑器，Keil5作为IDE。
> 虽然keil5老，但在嵌入式调试以及环境配置方面是非常简单易上手的。**千万不要用keil5来编辑代码！！！**

> VSCode作为现代化代码编辑器 ，通过各种插件即可实现各种提高效率的开发方式，**尤其是Github Copilot**

`.vscode` 文件夹已经提供了很多非常好用的插件，在`扩展`页面输入 `@recommended` 即可获取这些插件，点击安装即可。

## 欢迎贡献
如果你发现了本项目中的问题和可优化的点，可用创建issue进行讨论。

并且fork本项目后提交pr来贡献你的代码。

我们的管理员审核后会将你的代码合并进来。

> ***贡献代码时请参照 [注意事项](./doc/注意事项.md/#贡献代码) 中的贡献代码部分。*** 

## 后续计划
具体要做的事情在[TODO](./doc/TODO.md)中
- 先完成当前框架，实现对现有机器人的适配。
- 添加上下位机联调方便调参。

## 致谢
感谢各个战队的代码开源
- 跃鹿 [basic_framework-dev](https://gitee.com/hnuyuelurm/basic_framework)
- 未来战队 [XRobot仓库](https://github.com/xrobot-org/XRobot) , [XRobot文档](https://xrobot-org.github.io/)

## 附录
本文的详细补充内容写在 [附录](./doc/appendix.md) 中。

如发现本项目中的问题，请添加至[问题列表](./doc/questions.md)并与相关负责人联系，如果有解决方案可以提交pull request。
