# 附录

- [附录](#附录)
  - [1 代码编写](#1-代码编写)
    - [1.1 编辑器与IDE的选择](#11-编辑器与ide的选择)
    - [1.2 代码格式](#12-代码格式)
    - [1.3 编辑代码](#13-编辑代码)
  - [2 科学调参](#2-科学调参)
    - [2.1 信号发生器的使用](#21-信号发生器的使用)


## 1 代码编写
### 1.1 编辑器与IDE的选择
- **IDE : Keil5**
- **编辑器 : VSCode**

为了方便大家进行环境配置，本项目使用Keil5所带的armcc工具链进行编译，可用完美适配官代。在熟悉官代后可以轻松转移到本项目中进行编辑。

- 关于keil5，经典IDE，环境配置极为方便，对STM系列芯片使用者极为友好
- 关于VSCode，现代化代码编辑器（其实就是一高级版的文本编辑器），支持各种插件进行功能拓展，能极高的提高大家的工作效率。

### 1.2 代码格式
本项目代码格式主要基于Google风格，具体的格式配置信息请参照 [.clang-format](../.clang-format) 文件

- **如何格式化你的代码：** 右键选择 `格式化代码` （默认快捷键 `Shift + Alt + F`）
- **如何在部分区域禁用格式化：** \
    在区域开始的位置添加注释
    ```C
    // clang-format off
    ```
    在区域结束的位置添加注释
    ```C
    // clang-format on
    ```
    这样在格式化代码的时候就不会使用 [.clang-format](../.clang-format) 格式化这部分代码而保持原有风格了。

    > **关于禁用格式化的作用：** 部分地方由于格式化之后变成奇怪格式效果，这时候可以禁用格式化来保持我们所需要的格式。

### 1.3 编辑代码


## 2 科学调参
### 2.1 信号发生器的使用