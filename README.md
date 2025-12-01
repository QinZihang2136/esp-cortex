# EspCortex: Advanced Robot Controller based on ESP32-S3

**EspCortex** 是一个基于 **ESP32-S3** 的高性能机器人底层控制器固件。它采用现代化的 C++ 模块化设计和 **DataBus（发布-订阅）** 架构，旨在为移动机器人、无人机和平衡车提供硬实时的姿态解算与运动控制。

该项目作为机器人的 “小脑”，负责处理传感器数据融合（IMU / 磁力计 / 气压计）、闭环控制以及与上位机（如 ROS 2 / Micro-ROS）的高速通信。

---

## 🚀 核心特性 (Key Features)

- **高性能架构**
  - **双核异构调度**：Core 1 负责硬实时计算（传感器 / 控制），Core 0 负责通信（Wi‑Fi / ROS）。
  - **DataBus 总线**：基于 `Topic`（状态流）和 `CommandQueue`（指令流）的解耦架构，支持低延迟任务唤醒。

- **智能参数系统 (ParamRegistry)**
  - **持久化存储**：基于 NVS (Non-Volatile Storage)，支持 `float` / `int32_t` 参数掉电保存。
  - **动态注册**：模块化设计，支持在运行时动态注册参数，无需硬编码集中配置表。
  - **线程安全**：内置互斥锁，支持多任务并发安全读写。
  - **僵尸参数检测**：提供 `print_unused()` 功能，自动识别 Flash 中残留的废弃参数。

- **强大的传感器驱动**
  - **IMU**：支持 **ICM-42688-P**（SPI，200 Hz），采用 GPIO 中断驱动，实现极低延迟读取。
  - **磁力计**：支持 **QMC5883L**（I2C），采用主从分频读取策略。
  - **气压计**：支持 **ICP-20100**（I2C），采用主从分频读取策略。

- **模块化设计**
  - 驱动代码封装为标准 ESP-IDF 组件（位于 `components/`）。
  - 支持硬件中断 (Hardware Interrupt) 与轮询 (Polling) 模式灵活切换。

---

## 🛠️ 硬件规格 (Hardware Specifications)

| 组件              | 型号              | 接口          | 频率 (ODR) | 备注                             |
| :---------------- | :---------------- | :------------ | :--------- | :------------------------------- |
| **MCU**           | ESP32-S3-WROOM-1  | -             | 240 MHz    | Flash: 16 MB, PSRAM: 8 MB        |
| **IMU**           | ICM-42688-P       | SPI2 (FSPI)   | 200 Hz     | 主时钟源，触发中断               |
| **Magnetometer**  | QMC5883L          | I2C           | 50 / 100 Hz| 配合 IMU 分频读取                |
| **Barometer**     | ICP-20100         | I2C           | 25 / 40 Hz | 配合 IMU 分频读取                |

### 🔌 引脚定义 (Pin Map)

**SPI 总线 (IMU)**

- **MOSI**: GPIO 4  
- **MISO**: GPIO 5  
- **SCLK**: GPIO 6  
- **CS**: GPIO 7  
- **INT**: GPIO 15 (Active High, Push-Pull)

**I2C 总线 (Mag & Baro)**

- **SDA**: GPIO 10  
- **SCL**: GPIO 11  

---

## 📂 项目结构 (Project Structure)

```text
EspCortex/
├── CMakeLists.txt          # 项目构建脚本
├── components/             # 独立驱动组件库
│   ├── param_registry/     # 参数管理系统 (NVS + Mutex)
│   ├── spi_bus/            # 通用 SPI 总线封装 (C++)
│   ├── i2c_bus/            # 通用 I2C 总线封装 (C++)
│   ├── icm42688/           # ICM-42688-P 驱动 (支持中断/轮询)
│   ├── qmc5883l/           # QMC5883L 驱动 (含椭球拟合校准)
│   └── icp20100/           # ICP-20100 驱动
├── main/
│   ├── include/
│   │   ├── board_config.h  # 全局引脚定义
│   │   ├── robot_bus.hpp   # DataBus 总线定义 (Topic/Queue)
│   │   └── shared_types.h  # 数据结构定义
│   ├── modules/
│   │   └── sensor/         # 传感器采集任务 (Task Sensor)
│   └── main.cpp            # 系统入口与任务启动
└── sdkconfig               # ESP-IDF 项目配置 (Flash = 16 MB)
```

---

## 🔨 编译与烧录 (Build & Flash)

本项目基于 **ESP-IDF v5.x** 开发。请确保已正确安装 ESP-IDF 工具链与 VS Code / 命令行环境。

### 1. 环境准备

参考官方文档完成 ESP-IDF 安装，并确保命令行中可使用 `idf.py`。

### 2. 配置项目

```bash
idf.py menuconfig
```

重点检查：

- **Flash Size** 设置为 **16MB**
- **FreeRTOS Tick Rate** 为 **1000 Hz**（提升控制任务分辨率）

### 3. 编译

```bash
idf.py build
```

### 4. 烧录与串口监视

```bash
idf.py flash monitor
```

如需退出 `monitor`，使用快捷键 `Ctrl + ]`。

---

## 📅 开发日志 (Changelog)

- **Initial Commit**
  - 完成基础架构搭建（DataBus + 传感器任务框架）。

- **Driver Update**
  - 集成 ICM-42688-P 驱动，支持 SPI 硬件中断读取（200 Hz）。

- **Driver Update**
  - 集成 QMC5883L 与 ICP-20100 驱动，实现 I2C 分频读取策略。

- **Feature Add**
  - 新增 `ParamRegistry` 组件，实现基于 NVS 的参数持久化存储、动态注册与僵尸参数清理机制。
