# EspCortex: Advanced Robot Controller based on ESP32-S3

**EspCortex** 是一个基于 **ESP32-S3** 的高性能机器人底层控制器固件。它采用现代化的 **C++ 模块化设计** 和 **DataBus（发布-订阅）** 架构，旨在为移动机器人、无人机和平衡车提供硬实时的 **AHRS 姿态解算** 与运动控制。

本项目集成了 **Web 可视化控制台**，无需安装任何上位机软件，通过浏览器即可完成参数整定、状态监控和可视化传感器校准。

---

## 🚀 核心特性 (Key Features)

### 1) 高性能架构

- **双核异构调度**：Core 1 负责硬实时计算（传感器 / EKF / 控制），Core 0 负责通信（Wi‑Fi / WebServer）。
- **DataBus 总线**：基于 `Topic`（状态流）和 `CommandQueue`（指令流）的解耦架构，支持低延迟任务唤醒。

### 2) 专业级 AHRS 姿态解算 (State Estimation) [NEW]

- **9 轴融合算法**：内置扩展卡尔曼滤波（EKF）核心，融合加速度计、陀螺仪和磁力计数据。
- **抗干扰设计**：采用 **倾斜补偿航向融合算法（Tilt-Compensated Heading Fusion）**，有效消除磁倾角对 Roll/Pitch 的干扰，实现零延迟、无漂移的姿态跟踪。
- **FRD 坐标系**：全系统统一采用航空航天标准的 **前-右-下（Forward-Right-Down）** 坐标系，符合 PX4 / ArduPilot 规范。
- **Eigen 数学库**：集成 Eigen 线性代数库，保证矩阵运算的高效性与数值稳定性。

### 3) 交互式 Web 控制台 (Web Dashboard)

- **可视化校准向导**：[NEW] 新增基于 3D 飞机图标的交互式校准界面，指引用户完成加速度计六面校准与磁力计椭球拟合。
- **零依赖前端**：基于 HTML5 / Bootstrap / Three.js，前端资源直接存储在片上 Flash（SPIFFS）中。
- **WebSocket 通信**：毫秒级低延迟数据传输，支持实时波形显示与 3D 姿态同步。
- **实时海拔监测**：基于气压数据的实时相对高度计算与显示。

### 4) 智能参数系统 (ParamRegistry)

- **远程在线调参**：支持通过 Web 界面实时查看和修改参数（如 PID、EKF 噪声矩阵、传感器校准偏置），修改后立即生效。
- **持久化存储**：基于 NVS（Non-Volatile Storage），支持 `float` / `int32_t` 等参数掉电保存。
- **动态注册**：模块化设计，支持在运行时动态注册参数，无需在单一集中表中硬编码。

### 5) 强大的传感器驱动

- **IMU**：支持 ICM‑42688‑P（SPI，200 Hz），采用 GPIO 中断驱动，实现极低延迟读取。
- **磁力计**：支持 QMC5883L（I2C），采用主从分频读取策略，包含软/硬磁干扰修正。
- **气压计**：支持 ICP‑20100 与 LPS22HH（I2C），系统启动时自动识别传感器型号并加载对应驱动。

---

## 🛠️ 硬件与坐标系 (Hardware & Coordinates)

### 📐 坐标系定义 (Coordinate System)

本项目严格遵循 **FRD（前-右-下）右手坐标系**定义，与 PX4 / ROS 2 标准兼容：

- **X 轴**：指向机头前方（Forward）
- **Y 轴**：指向机身右侧（Right）
- **Z 轴**：指向地心（Down）

> 注意：固件内部已包含 **Sensor Frame → Body Frame** 的映射层。请根据 `task_sensor.cpp` 中的配置确保传感器安装方向与机体坐标系对齐。

### 🔌 硬件规格 (Hardware Spec)

| 组件 | 型号 | 接口 | 频率 (ODR) | 备注 |
|---|---|---|---:|---|
| MCU | ESP32‑S3‑WROOM‑1 | - | 240 MHz | Flash: 16 MB, PSRAM: 8 MB |
| IMU | ICM‑42688‑P | SPI2 (FSPI) | 200 Hz | 主时钟源，触发中断 |
| Magnetometer | QMC5883L | I2C | 100 Hz | 2 分频读取（50 Hz EKF Update） |
| Barometer | LPS22HH / ICP‑20100 | I2C | 75 Hz / 25 Hz | 自动探测型号 |

### 📌 引脚定义 (Pin Map)

**SPI 总线（IMU）**

- MOSI: GPIO 4  
- MISO: GPIO 5  
- SCLK: GPIO 6  
- CS: GPIO 7  
- INT: GPIO 15（Active High）

**I2C 总线（Mag & Baro）**

- SDA: GPIO 10  
- SCL: GPIO 11  

---

## 📂 项目结构 (Project Structure)

```text
EspCortex/
├── CMakeLists.txt          # 项目构建脚本
├── data/                   # 前端网页资源 (HTML/JS/CSS) -> 烧录至 SPIFFS
├── components/             # 独立驱动组件库
│   ├── common/             # 通用类型定义 (shared_types.h)
│   ├── ekf/                # EKF 数学核心库 (含 fuse_accel/fuse_mag 实现)
│   ├── web_server/         # HTTP/WebSocket 服务器
│   ├── wifi_manager/       # WiFi 连接管理
│   ├── spiffs_manager/     # SPIFFS 文件系统挂载
│   ├── param_registry/     # 参数管理系统 (NVS)
│   ├── spi_bus/            # SPI 总线封装
│   ├── i2c_bus/            # I2C 总线封装
│   ├── icm42688/           # ICM-42688-P 驱动
│   ├── qmc5883l/           # QMC5883L 驱动
│   ├── icp20100/           # ICP-20100 驱动
│   └── lps22hh/            # LPS22HH 驱动
├── main/
│   ├── include/
│   │   ├── board_config.h  # 全局引脚定义
│   │   └── robot_bus.hpp   # DataBus 总线定义
│   ├── modules/
│   │   ├── sensor/         # 传感器任务 (坐标映射/校准应用/FRD转换)
│   │   └── estimator/      # 姿态估算任务 (EKF Predict/Update)
│   └── main.cpp            # 系统入口
└── sdkconfig               # ESP-IDF 项目配置
```

---

## 🌐 网络与使用 (Connectivity)

设备启动后会自动尝试连接预设的 Wi‑Fi：

- **STA 模式（默认）**：尝试连接已配置的路由器  
- **AP 模式（后备）**：如果连接失败，自动开启热点  
  - SSID: `EspCortex`  
  - Password: `12345678`

访问控制台：

1. 电脑 / 手机连接到同一网络（或连接设备 AP 热点）。
2. 浏览器访问：  
   - `http://espcortex.local`  
   - 或 `http://<device-ip>`

---

## 🔨 编译与烧录 (Build & Flash)

本项目基于 **ESP-IDF v5.x** 开发。

### 1) 编译与烧录固件

```bash
idf.py build
idf.py flash
```

### 2) 烧录网页资源 (SPIFFS)

⚠️ **重要**：Web 控制台的所有静态资源（HTML / JS / Icons）都存储在 **SPIFFS 分区**。每次修改 `data/` 目录或初次烧录时，必须执行以下操作之一：

- **方法 A（推荐）**：使用 VS Code 插件命令 **"ESP-IDF: Flash SPIFFS Partition"**。
- **方法 B（手动）**：使用 `parttool.py` 或自定义 CMake 目标进行烧录。

---

## 📅 开发日志 (Changelog)

### AHRS Complete & FRD Transition (Current)

- **Architecture**：全面迁移至 FRD（前-右-下）坐标系；重构了 Sensor Task 中的坐标映射逻辑。
- **EKF Upgrade**：完成磁力计融合（`fuse_mag`）；实现 **倾斜补偿航向融合** 算法，解决了磁倾角对横滚/俯仰的耦合干扰。
- **Web UI**：新增 **可视化校准向导**，使用 3D 飞机图标直观指示校准动作；修复了模拟器数据的坐标系方向。
- **Fix**：修复加速度计校准参数（`ACCEL_SCALE`）误置零导致 Pitch 轴数据丢失的严重 Bug。

### Driver Update

- 新增 **LPS22HH 气压计**支持（75 Hz 低噪声模式）。
- 实现 I2C 设备自动扫描与识别（`0x5C/0x5D/ICP-20100`）。
- 新增基于气压数据的实时海拔计算（Altitude Calculation）。

### Initial Framework

- 完成 DataBus 总线架构与 `param_registry` 参数系统。
- 实现 WebSocket 双向通信与 Web 调参基础功能。
