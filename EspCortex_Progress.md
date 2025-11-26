# EspCortex 项目进度汇总文档

**更新时间:** 2025-11-26
**当前阶段:** 基础软件框架搭建完成 / 硬件调试与环境配置阶段
**目标硬件:** ESP32-S3 (Custom Board) + ICM-42688-P + QMC5883L + CAN/I2C/UART 扩展
**上层架构:** RK3588 (ROS2) <--> ESP32-S3 (micro-ROS) <--> STM32 (Actuators)

---

## 1. 软件架构设计 (Software Architecture)

采用了 **"DataBus (Topic) + CommandBus (Queue)"** 的分层解耦架构，类似于 PX4 uORB 或 Mini-ROS。

### 核心机制
1.  **RobotBus (单例总线):** 全局唯一的通信枢纽，包含所有的 Topic 和 Queue。
2.  **Topic<T> (状态流):**
    * 用于传输 **"最新值"** (如 IMU 数据、姿态)。
    * 机制：`std::mutex` 保护 + `generation` 版本号检查 + `TaskNotify` 零延迟唤醒。
    * 特点：多读一写，允许丢包，只取最新。
3.  **CommandQueue<T> (命令流):**
    * 用于传输 **"事件/指令"** (如控制命令、模式切换)。
    * 机制：基于 FreeRTOS `xQueue`。
    * 特点：先进先出，严格保序，不可丢失。

### 任务调度与核心分配 (Core Pinning)
| 任务名称 | 核心 (Core) | 优先级 | 职责 | 触发机制 |
| :--- | :--- | :--- | :--- | :--- |
| **Sensor Task** | **Core 1** | High (5) | 读取硬件传感器 -> Publish Topic | 定时 (200Hz) |
| **Control Task** | **Core 1** | Realtime (6) | 姿态解算 + 电机控制 | **事件驱动** (等待 Sensor 通知) |
| **uROS Task** | **Core 0** | Low (2) | 与 RK3588 通信 (WiFi/UART) | 轮询 + 队列 |

---

## 2. 工程文件结构

目前工程已创建，位于 `EspCortex/main/` 下，核心文件如下：

* `main.cpp`: 任务创建 (app_main) 与具体的 Task 逻辑实现。
* `robot_bus.hpp`: `RobotBus` 单例类定义，管理所有总线资源。
* `topic.hpp`: `Topic<T>` 模板类实现 (含 Mutex 和 Notify 逻辑)。
* `command_queue.hpp`: `CommandQueue<T>` 模板类实现。
* `shared_types.h`: 定义 `ImuData`, `AttitudeData`, `MotionCommand` 等纯数据结构。

**关键配置 (Menuconfig):**
* `FreeRTOS Tick Rate`: **1000 Hz**
* `C++ Exceptions`: **Enable** (为了 std::mutex 安全)
* `Flash Size`: **16 MB** (根据硬件实际情况修改)

---

## 3. 硬件调试记录 (Hardware Debugging)

### 3.1 已知硬件问题
* **现象:** 下载程序后，程序不自动运行，必须手动按一下 `EN` (复位) 键。偶尔自动下载失败。
* **原因:** 自动下载电路 (Auto-Download Circuit) 的时序与 EN 引脚的 RC 延时电路不匹配。原理图中 EN 引脚的电容 **C1** (100nF) 容值偏小，导致 EN 上电过快，锁存了错误的 Boot 状态。
* **解决方案:** 建议在 C1 上并联一个 **1uF ~ 10uF** 的电容。

### 3.2 硬件配置修正
* **现象:** 编译警告 `Detected size(16384k) larger than ... binary image header(2048k)`.
* **修复:** 已在 menuconfig 中将 Flash Size 从默认的 2MB 修改为 **16MB**。

### 3.3 调试接口说明
* `/dev/ttyUSB0` (CH340): **稳定**。用于下载固件 (`idf.py flash`) 和查看日志 (`idf.py monitor`)。
* `/dev/ttyACM0` (Native USB): **不稳定** (受复位电路影响)。用于 JTAG 在线调试 (OpenOCD)。

---

## 4. 当前遇到的报错与修复 (Troubleshooting Log)

### ❌ 错误: `LIBUSB_ERROR_ACCESS`
* **场景:** 尝试使用 VS Code 进行 OpenOCD JTAG 调试时报错。
* **原因:** Linux 当前用户 (`qinzihang`) 没有权限直接访问 USB 设备。
* **解决方案 (已执行):**
    1.  创建 udev 规则: `/etc/udev/rules.d/60-openocd.rules`。
    2.  将用户加入 `dialout` 组。
    3.  **待执行动作:** 拔掉 USB 线，等待 3 秒重新插入，以生效权限。

### ❌ 编译错误: `missing initializer` / `esp_timer_get_time`
* **原因:** C++ 结构体初始化语法严格；缺少头文件引用。
* **修复:**
    * 结构体初始化改为 `ImuData data = {};`。
    * `main.cpp` 增加 `#include "esp_timer.h"`。

---

## 5. 下一步计划 (Next Steps)

1.  **权限验证:** 重新插拔 USB，验证 `LIBUSB_ERROR_ACCESS` 是否消失，确认 OpenOCD 能否启动。
2.  **IMU 驱动开发:**
    * 硬件: **ICM-42688-P** (SPI2 接口)。
    * 目标: 编写 SPI 驱动，替换 `task_sensor` 中的正弦波模拟数据，读取真实的加速度/角速度。
3.  **Micro-ROS 联调:** 在 PC 端运行 ROS2 agent，验证板载 ESP32 能否通过 `ttyUSB0` 或 WiFi 与上位机通讯。
