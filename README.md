# 并联SCARA-ROBOT

基于 STM32F103C8T6 的并联 SCARA 绘图机器人项目，包含下位机运动控制固件和 PyQt5 上位机控制界面。

## 功能概览

- 并联 SCARA 逆运动学解算
- 双步进电机脉冲控制
- 72 MHz 系统时钟配置
- DDA 插补轨迹规划
- 梯形/S 曲线速度规划
- 舵机抬笔/下笔控制
- 串口 G-code 风格控制协议
- 预置图案绘制：考核图案、五角星、爱心、花形
- 写字功能：轮廓字路径生成
- 示教功能：记录点位、回放轨迹、保存/加载 CSV
- 光电开关状态检测与回零：PB0、PA1，内部上拉，NPN 低电平有效

## 目录说明

```text
Core/                    STM32 下位机核心代码
Core/Src/main.c           运动控制、串口协议、图案、光电开关逻辑
Core/Inc/main.h           引脚定义
Core/Startup/             STM32 启动文件
scara.ioc                 STM32CubeMX/CubeIDE 工程配置
STM32F103C8TX_FLASH.ld    链接脚本
scara_pyqt_host.py        PyQt5 上位机
requirements-host.txt     上位机 Python 依赖
run_scara_host.bat        Windows 启动脚本
scara_drawpath_simulation.m MATLAB 轨迹仿真脚本
```

本仓库只保留核心代码。`Drivers/`、`Debug/`、参考仓库和本地缓存不上传；使用 STM32CubeIDE 打开工程或 `.ioc` 后可重新生成/补齐 HAL 驱动文件。

## 硬件引脚

| 功能 | 引脚 |
| --- | --- |
| 电机 1 STEP | PA8 |
| 电机 1 DIR | PB15 |
| 电机 1 ENA | PB14 |
| 电机 2 STEP | PA6 |
| 电机 2 DIR | PB13 |
| 电机 2 ENA | PB12 |
| 抬笔舵机 PWM | PB6 / TIM4_CH1 |
| 右电机光电开关 A | PB0 |
| 左电机光电开关 B | PA1 |

光电开关采用内部上拉，适合 NPN 开集电极输出：

- 未遮挡：输入为高电平，状态为 `0`
- 遮挡：输入被拉低，状态为 `1`

下位机主动回传示例：

```text
EV SW A1 B0
```

其中 `A` 对应右电机侧 PB0，`B` 对应左电机侧 PA1。

## 下位机串口协议

默认串口参数：

```text
115200 8N1
```

常用命令：

```text
M17                 使能电机
M18                 关闭电机
P0                  抬笔
P1                  下笔
G1 X0 Y315 F12 A35  直线运动
G2/G3               圆弧运动
DRAW1..DRAW4        绘制预置图案
J X+ 5              手动点动
SXY X0 Y314.94      设置当前位置
SZ M1 M2            设置当前电机零点
PPR 6124            设置每圈步数
HOME                光电开关回零
SETHOME             直接把当前位置设为 X0 Y314.94
SW                  查询光电开关状态
STATUS              查询当前位置
!                   急停
?                   查看命令帮助
```

`SW` 查询返回示例：

```text
SW A0 B0
SW A1 B0
SW A0 B1
```

## 上位机运行

安装依赖：

```powershell
pip install -r requirements-host.txt
```

启动：

```powershell
python scara_pyqt_host.py
```

或双击：

```text
run_scara_host.bat
```

上位机支持串口连接、手动控制、预置图案、写字、示教、串口日志和光电开关状态显示。

## 固件构建

推荐使用 STM32CubeIDE：

1. 打开本工程目录，或导入 `scara.ioc`。
2. 根据 `.ioc` 生成/恢复 HAL 驱动文件。
3. 编译并烧录到 STM32F103C8T6。

命令行构建需要本机安装并配置 `arm-none-eabi-gcc`。

## 回零说明

`HOME` 会执行光电开关回零：先抬笔并使能电机，然后先让左电机寻找 `PA1` 的遮挡信号，再让右电机寻找 `PB0` 的遮挡信号，找到后把当前位置标定为机械参考位 `X0 Y314.94`。如果启动回零时某个光电已经处于遮挡状态，程序会先反向退出遮挡，再重新正向寻找遮挡边沿。`SETHOME` 不会移动电机，只是直接把当前位置设为这个参考位，适合手动摆到参考姿态后使用。

回零方向在 `Core/Src/main.c` 顶部的 `HOME_SEEK_M1_DIR` 和 `HOME_SEEK_M2_DIR` 中设置。若回零时电机远离光电开关，或返回 `ER HOME M1` / `ER HOME M2`，把对应方向从 `-1` 改为 `1` 后重新烧录。
