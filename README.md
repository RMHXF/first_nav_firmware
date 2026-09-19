# RM Infantry Chassis Firmware（first_nav_firmware）

RoboMaster 步兵机器人**底盘控制固件**，基于 STM32H723 + FreeRTOS，完成麦克纳姆轮底盘的运动解算、达妙（DM）电机控制、BMI088 姿态解算以及遥控 / MiniPC 双模式通信。

> 工程名 `RM_Infantry`，由 STM32CubeMX 生成框架，使用 arm-none-eabi-gcc + Makefile 编译。

---

## 1. 硬件平台

| 项目 | 说明 |
| --- | --- |
| 主控 | STM32H723VGTx（Cortex-M7 @ 480 MHz，HSE 24 MHz） |
| 驱动电机 | 达妙（DM）系列电机 ×4（底盘麦克纳姆轮，FDCAN 总线） |
| IMU | BMI088（加速度计 + 陀螺仪，SPI 接口） |
| 遥控器 | FS-i6x + iA6B 接收机（SBUS，UART5 DMA） |
| 上位机 | MiniPC（串口，SLAM 导航指令） |
| 调试口 | USB CDC 虚拟串口 |

## 2. 功能特性

- **麦克纳姆轮底盘运动学**：将底盘期望速度 `(Vx, Vy, Wz)` 分解为 4 个麦轮转速。
- **双控制模式**：
  - `REMOTE_CTRL` —— FS-i6x 遥控器遥控；
  - `SLAM` —— MiniPC 通过串口下发目标速度（`Target_V`）。
- **姿态解算**：BMI088 + Mahony 互补滤波，输出 `roll / pitch / yaw`。
- **陀螺仪上电零偏校准**：上电采集 5000 次原始数据求均值，自动扣除零偏。
- **IMU 温度闭环**：PID 温控 + TIM PWM 加热，保证传感器工作温度稳定。
- **电机控制**：达妙电机 FDCAN 通信，支持 MIT / 速度 / 位置等控制模式。
- **串口协议解析**：MiniPC 数据帧（帧头 + CRC16 校验），支持粘包递归解析。
- **遥控失控检测**：SBUS 丢帧 / 失控标志位解析。

## 3. 目录结构

```
first_nav_firmware/
├── Core/                    # CubeMX 生成的 HAL 初始化与外设驱动
│   ├── Inc/                 # main.h、FreeRTOSConfig.h 等
│   └── Src/                 # main.c、gpio.c、fdcan.c、spi.c、freertos.c 等
├── application/             # 应用层任务
│   ├── Inc/
│   │   ├── Chassis_Task.h   # 底盘运动控制
│   │   ├── Trans_Task.h     # 通信 / 遥控 / MiniPC
│   │   └── IMU_Task.h       # 姿态解算 / 温度控制
│   └── Src/                 # 对应实现
├── DM_bsp/                  # 达妙电机驱动（FDCAN 底层 + 控制接口）
│   ├── inc/                 # bsp_fdcan.h、dm_motor_ctrl.h、dm_motor_drv.h
│   └── src/
├── remote_ctrl/             # FS-i6x SBUS 遥控数据解包（i6x.c/h）
├── device/BMI088/           # BMI088 IMU 驱动（SPI）
├── algorithms/
│   ├── Control/             # pid.c/h、CRC_Check.c/h
│   └── Filters/             # MahonyAHRS.c/h 互补滤波
├── USB_DEVICE/              # USB CDC 虚拟串口
├── Middlewares/             # FreeRTOS、STM32 USB Device 库
├── Drivers/                 # STM32H7 HAL、CMSIS
├── Makefile                 # 构建脚本
├── STM32H723XG_FLASH.ld     # 链接脚本
└── RM_Infantry.ioc          # CubeMX 工程文件
```

## 4. RTOS 任务

| 任务 | 函数 | 优先级 | 职责 |
| --- | --- | --- | --- |
| `defaultTask` | `StartDefaultTask` | Normal | 初始化 USB 设备 |
| `Trans_Task` | `Trans_Task_main` | Normal | 遥控接收、MiniPC 通信、电机使能/控制 |
| `Chassic_Task` | `Chassis_Task_main` | Low | 底盘模式切换与运动解算（1ms） |
| `imuTempCtrl` | `IMU_Task` | Low | IMU 数据读取、姿态解算、温控（1kHz） |

## 5. 关键模块说明

### 5.1 底盘控制（Chassis_Task）

- 三种底盘模式（`Chassis_mode_t`）：`silence`（静默，四轮输出 0）、`normal`（正常）、`follow`（跟随，待实现）。
- 四种控制源（`Chassis_ctrl_mode_t`）：`REMOTE_CTRL`、`KEYBOARD_MOUSE`、`CUSTOM_CONTROLLOR`、`SLAM`。
- 麦克纳姆轮分解公式（`Chassis_normal_mode`）：

```c
motor[0] = -Vx + Vy + Wz;
motor[1] = -Vx - Vy + Wz;
motor[2] =  Vx - Vy + Wz;
motor[3] =  Vx + Vy + Wz;
```

- **正运动学解算**（`Chassis_Fwd_solution`）：由 4 个麦轮实际转速反解底盘当前速度 `(vx, vy, wz)`，用于向 MiniPC 回传底盘实际速度。

### 5.2 通信（Trans_Task）

- **遥控接收**：UART5 DMA（`HAL_UARTEx_ReceiveToIdle_DMA`）接收 SBUS 帧，`sbus_to_i6x` 解包，映射到 `ch[6]`（摇杆/旋钮）和 `s[4]`（拨杆），含 `frame_lost` / `failsafe` 标志。
- **MiniPC 通信**：串口协议帧，接收头 `0x5A`、尾 `0x5B`，发送头 `0xA5`、尾 `0xB5`，负载结构见 [Trans_Task.h](application/Inc/Trans_Task.h)，带 CRC16 校验；`MiniPC_Data_Read` 支持多帧粘包递归解析，底盘当前速度通过 USB CDC（`MiniPC_Data_Transmit`）回传给 MiniPC。

### 5.3 姿态解算（IMU_Task）

- 1kHz 采样，Mahony 互补滤波融合陀螺仪与加速度计。
- 上电先做 5000 次陀螺仪零偏校准，完成后才输出姿态角（`yaw` 映射到 `[0°, 360°]`）。
- 温度 PID（`IMU_TEMP_SET = 47℃`）控制 TIM3 CH4 PWM 加热，超温（60℃）自动关断。

### 5.4 电机驱动（DM_bsp）

- FDCAN 总线，`dm_motor_enable / dm_motor_disable` 使能/失能，`spd_ctrl` 速度控制等。
- 控制模式：MIT 模式、位置模式、速度模式、力矩模式。

## 6. 构建与烧录

### 6.1 依赖

- `arm-none-eabi-gcc` 工具链（需加入 `PATH`，或通过 `make GCC_PATH=xxx` 指定）
- `make`
- 烧录：OpenOCD（CMSIS-DAP 调试器）

### 6.2 编译

```bash
make          # 生成 build/RM_Infantry.elf / .hex / .bin
make clean    # 清理构建产物
```

### 6.3 烧录

```bash
openocd -f interface/cmsis-dap.cfg \
        -f target/stm32h7x.cfg \
        -c "program build/RM_Infantry.hex verify reset exit"
```

## 7. 外设映射

| 外设 | 用途 |
| --- | --- |
| FDCAN1 / FDCAN2 | 达妙电机 CAN 总线 |
| SPI2 | BMI088 IMU |
| UART5（DMA） | SBUS 遥控接收 |
| UART7 / USART1 / USART10 | MiniPC 通信 / 调试串口 |
| TIM3 | PWM 加热输出（IMU 温控） |
| TIM6 | HAL 时基 |
| USB_DEVICE | USB CDC 虚拟串口 |

## 8. 协议定义

MiniPC 与底盘之间使用带 CRC16 校验的定长帧，详细字段定义见
[Trans_Task.h](application/Inc/Trans_Task.h) 中的 `ReceivePacket` / `SendPacket` 结构体：

- 接收帧（MiniPC → 底盘）：`header(0x5A)` + `Target_V(vx,vy,wz)` + `IMU_lidar(roll,pitch,yaw)` + `timestamp` + `tail` + `checksum`
- 发送帧（底盘 → MiniPC）：`header(0xA5)` + `Current_V(vx,vy,wz)` + `timestamp` + `tail` + `checksum`

## 9. 更新日志

### 2026-09-19

本次更新完善了底盘运动解算与 MiniPC 回传链路，并修复了 USB 虚拟串口时钟配置问题。

**新增**

- **正运动学解算**：新增 `Chassis_Fwd_solution()`（[Chassis_Task.c](application/Src/Chassis_Task.c)），根据 4 个麦轮实际转速反解底盘当前速度 `(vx, vy, wz)`。
- **MiniPC 回传链路**：新增 `MiniPC_Data_Send_Process()` / `MiniPC_Data_Transmit()`（[Trans_Task.c](application/Src/Trans_Task.c)），底盘通过 USB CDC 定时向 MiniPC 回传 `Current_V` 数据帧（帧头 `0xA5`、帧尾 `0xB5`，CRC16 校验）。
- **USB CDC 波特率协商**：新增 `USBD_CDC_LineCoding`，实现 `CDC_SET_LINE_CODING` / `CDC_GET_LINE_CODING` 处理，默认 115200-8-N-1。
- **底盘常量**：新增车轮半径、`SIN_45`、底盘几何尺寸 `CHASSIS_LX / CHASSIS_LY` 等宏定义（[Chassis_Task.h](application/Inc/Chassis_Task.h)）。

**修改 / 修复**

- **USB 时钟源**：USB 时钟由 PLL 改为 HSI48（48 MHz），修复 USB CDC 无法正常通信的问题（`main.c`、`usbd_conf.c`、`RM_Infantry.ioc`）。
- **麦轮逆运动学公式**：修正 `Chassis_normal_mode()` 中 4 轮速度分解公式。
- **遥控方向**：修正 `REMOTE_CTRL` 模式下 `Vy / Wz` 的符号。
- **静默模式空指针**：`Chassis_Mode_Loop()` 中 `Chassis_Data` → `Chassis_Data_p`，修复未使用入参的问题。
- **失控保护**：恢复遥控丢帧时底盘电机失能 / 使能逻辑（`Trans_Task.c`）。
- **电机控制**：`ctrl_dm_motor()` 恢复使用 `Chassis_Data.motor_val[0]`。
- **IMU 温控任务优先级**：`imuTempCtrl` 优先级由 Low 提升为 AboveNormal（`freertos.c`、`RM_Infantry.ioc`）。

---

*工程基于 STM32CubeMX + FreeRTOS + HAL，持续开发中。*
