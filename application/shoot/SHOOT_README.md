# 发射介绍
## 硬件规范
### CAN线
- **摩擦轮+拨弹盘**

    若使用 **GM3508** 作为摩擦轮电机和 **GM2006** 拨弹盘电机，统一接入`CAN2`，并使用 `0x200` 标识符作为发射数据包的标识符

- **摩擦轮+弹鼓+推杆**

    若使用 **GM3508** 作为摩擦轮电机，统一接入`CAN2`，并使用 `0x200` 标识符作为发射数据包的标识符

    若使用 **GM3508** 作为弹鼓电机，统一接入`CAN1`，并使用 `0x200` 标识符作为发射数据包的标识符

### 电机
统一以逆时针旋转为正方向(发送正电流时的电机输出轴旋转方向)

### 电机ID
- **摩擦轮+拨弹盘**
  - 左摩擦轮：1
  - 右摩擦轮：2
  - 拨弹盘：3

- **摩擦轮+弹鼓+推杆**
  - 左上摩擦轮：1
  - 左下摩擦轮：2
  - 右上摩擦轮：3
  - 右上摩擦轮：4
  - 弹鼓：1
  - 推杆：


### 遥控器使用
- 左拨杆：
  - 上档：射击模式
  - 中档：键鼠控制
  - 下档：拨弹盘和摩擦轮锁死，紧急停止
- 右拨杆：
  - 上档：
  - 中档：
  - 下档：拨弹盘和摩擦轮锁死，紧急停止
- 左摇杆：
- 右摇杆：
- 左滚轮：
 


### 键鼠操作
- 左键：射击模式
