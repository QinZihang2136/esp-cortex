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
- **倾斜补偿航向融合**：采用**Tilt-Compensated Heading Fusion** 算法，使用只包含 Roll/Pitch 的旋转矩阵，避免 Yaw 分量干扰磁力计计算，有效消除磁倾角对横滚/俯仰的耦合干扰。
- **自适应残差门限**：基于残差稳定性（标准差）动态调整融合门限（10° → 60°），自动区分"暂时干扰"和"系统性偏置"，避免"拒绝融合死锁"。
- **初始航向对齐**：首次磁力计融合时强制对齐，解决启动时残差过大问题。
- **FRD 坐标系**：全系统统一采用航空航天标准的 **前-右-下（Forward-Right-Down）** 坐标系，符合 PX4 / ArduPilot 规范。
- **Eigen 数学库**：集成 Eigen 线性代数库，保证矩阵运算的高效性与数值稳定性。

### 3) EKF 调试可视化系统 (NEW) 🎯

- **Web 实时调试面板**：20Hz 高频刷新，显示残差、零偏、协方差、卡尔曼增益等完整 EKF 内部状态。
- **Telemetry 自适应推流**：默认 30Hz 推送，支持异常自动回退到安全频率（20Hz），兼顾流畅度与稳定性。
- **3D 向量视图增强**：添加磁力计实测航向箭头（红色）和 IMU 预测航向箭头（青色），直观对比融合效果。
- **残差稳定性检测**：实时计算残差标准差，自动诊断磁力计状态（稳定偏置 vs 波动干扰）。
- **智能警告系统**：颜色编码提示（绿色正常、黄色警告、红色异常），快速定位问题。
- **零偏监控**：实时显示 3 轴陀螺仪零偏估计，防止发散（±0.05 rad/s 保护机制）。

### 4) 交互式 Web 控制台 (Web Dashboard)

- **可视化校准向导**：[NEW] 新增基于 3D 飞机图标的交互式校准界面，指引用户完成加速度计六面校准与磁力计椭球拟合。
- **零依赖前端**：基于 HTML5 / Bootstrap / Three.js，前端资源直接存储在片上 Flash（SPIFFS）中。
- **WebSocket 通信**：毫秒级低延迟数据传输，支持实时波形显示与 3D 姿态同步。
- **实时海拔监测**：基于气压数据的实时相对高度计算与显示。

### 5) 智能参数系统 (ParamRegistry)

- **远程在线调参**：支持通过 Web 界面实时查看和修改参数（如 PID、EKF 噪声矩阵、传感器校准偏置），修改后立即生效。
- **持久化存储**：基于 NVS（Non-Volatile Storage），支持 `float` / `int32_t` 等参数掉电保存。
- **动态注册**：模块化设计，支持在运行时动态注册参数，无需在单一集中表中硬编码。

### 6) 强大的传感器驱动

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

### 🐛 调试输出控制 (Debug Log Control) [NEW]

项目提供了灵活的调试输出控制系统，可通过宏定义精确控制各模块的打印输出：

**控制文件**：`components/common/include/debug_log.hpp`

**主要功能**：
- **全局开关**：`DEBUG_LOG_GLOBAL_ENABLE` - 一键关闭所有调试输出
- **模块化控制**：针对传感器任务和姿态估计任务的独立开关
- **分类打印**：IMU、磁力计、气压计、欧拉角、陀螺仪、加速度计等分类控制
- **频率调节**：可配置打印间隔，避免高频输出影响性能

**使用示例**：

```cpp
// 关闭所有调试输出（生产环境）
#define DEBUG_LOG_GLOBAL_ENABLE 0

// 只关闭 IMU 数据打印，保留其他
#define DEBUG_SENSOR_IMU 0

// 关闭估计器的统计信息
#define DEBUG_ESTIMATOR_STATISTICS 0
```

**适用模块**：
- **传感器任务** (`task_sensor.cpp`)：IMU、磁力计、气压计、I2C 扫描、校准参数
- **姿态估计任务** (`task_estimator.cpp`)：欧拉角、陀螺仪、加速度计、零偏、时间戳、统计计数器

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
├── docs/                   # 📘 详细模块文档 (新增)
│   ├── sensor.md          # 传感器采集模块文档
│   ├── ekf.md             # 姿态估计算法模块文档
│   ├── web_server.md      # Web 服务器与通信模块文档
│   ├── param_system.md    # 参数管理系统文档
│   └── development.md     # 开发环境与编译指南
├── components/             # 独立驱动组件库
│   ├── common/             # 通用类型定义与调试控制 (shared_types.h, debug_log.hpp)
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
│   │   ├── estimator/      # 姿态估算任务 (EKF Predict/Update)
│   │   └── telemetry/      # 遥测任务 (WebSocket 数据推送)
│   └── main.cpp            # 系统入口
└── sdkconfig               # ESP-IDF 项目配置
```

---

## 📚 详细文档 (Documentation)

本项目提供详细的模块文档，帮助开发者快速了解各模块的原理和使用方法。

### 模块文档

| 文档 | 描述 |
|------|------|
| **[传感器采集模块](docs/sensor.md)** | IMU、磁力计、气压计驱动，坐标映射，校准系统 |
| **[姿态估计模块](docs/ekf.md)** | ES-EKF 算法原理，传感器融合，参数调优 |
| **[Web 服务器模块](docs/web_server.md)** | HTTP/WebSocket 通信，前端架构，API 接口 |
| **[Telemetry 30Hz 更新说明](docs/telemetry_30hz_update.md)** | 本次推流提频、保护机制、验证步骤与调参建议 |
| **[参数管理系统](docs/param_system.md)** | NVS 持久化存储，在线调参，参数注册 |
| **[开发环境指南](docs/development.md)** | 环境搭建，编译烧录，调试技巧 |

### 快速链接

- 🚀 **快速开始**: 查看 [开发环境指南](docs/development.md) 搭建开发环境
- 📖 **传感器校准**: 参考 [传感器模块文档](docs/sensor.md) 进行六面校准
- 🔧 **EKF 调优**: 阅读 [姿态估计模块](docs/ekf.md) 调整滤波器参数
- 🌐 **Web 接口**: 查看 [Web 服务器文档](docs/web_server.md) 了解通信协议

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

### Telemetry 30Hz & 自适应回退 (Latest) - 2026-02-10

- **推流频率升级**：Telemetry 默认目标频率从 20Hz 提升到 30Hz（约 33ms 周期）。
- **前端渲染对齐**：Dashboard 渲染节流从 80ms 调整到 33ms，避免“发送提频但显示不刷新”。
- **链路保护机制**：新增发送侧自动回退逻辑，连续异常时自动降到安全频率，恢复后自动退出回退。
- **运行参数化**：新增 `TELM_RATE_HZ_DASH` / `TELM_RATE_HZ_SAFE` / `TELM_AUTO_FALLBACK_EN`，支持在线调参。
- **可观测性增强**：Telemetry 日志新增 `target/active/fallback` 状态，便于判断是否进入回退模式。

### EKF 融合算法优化 & 调试可视化系统 (Latest) - 2026-01-29

**核心算法修复**:
- **倾斜补偿航向计算**：使用只包含 Roll/Pitch 的旋转矩阵，避免 Yaw 分量干扰磁力计计算
- **航向方向修正**：取反以匹配 IMU 航向方向（FRD 右手定则）
- **初始航向对齐**：首次磁力计融合时强制对齐，解决启动时残差过大问题
- **自适应残差门限**：基于残差稳定性（标准差）动态调整门限（10° → 60°），自动区分"干扰"和"系统性偏置"
- **零偏保护机制**：限制陀螺仪零偏在 ±0.05 rad/s，防止发散

**调试可视化系统**:
- **EKF 调试面板**：新增 Web 实时调试界面，显示残差、零偏、协方差、卡尔曼增益等完整信息
- **3D 向量视图增强**：添加磁力计实测航向箭头（红色）和预测航向箭头（青色），直观对比融合效果
- **残差稳定性检测**：实时计算残差标准差，自动诊断磁力计状态
- **WebSocket 数据推送**：20Hz 高频传输 EKF 调试数据，无延迟监控融合过程

**自适应恢复机制**:
- 区分"暂时干扰"（残差波动）和"系统性偏置"（残差稳定但大）
- 检测到稳定偏置时自动放宽门限到 60°，避免"拒绝融合死锁"
- 参考 PX4 创新/Innovation 门限，增加自适应恢复能力

**文档完善**:
- 更新 `docs/ekf.md`：添加倾斜补偿算法、初始对齐、自适应门限详细说明
- 新增 EKF 调试诊断指南，帮助快速定位问题

### Debug System & Output Optimization - 2026-01-28

- **Debug Control System**：新增统一的调试输出控制系统（`debug_log.hpp`），支持通过宏定义灵活控制各模块打印输出
- **Modular Logging**：传感器任务和姿态估计任务的打印输出按类别分离（IMU、磁力计、气压计、欧拉角、陀螺仪等）
- **Performance**：可配置打印频率，避免高频输出影响实时性能；一键关闭所有调试输出用于生产环境

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
