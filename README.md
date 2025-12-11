# EspCortex: Advanced Robot Controller based on ESP32-S3

**EspCortex** 是一个基于 **ESP32-S3** 的高性能机器人底层控制器固件。它采用现代化的 C++ 模块化设计和 **DataBus（发布-订阅）** 架构，旨在为移动机器人、无人机和平衡车提供硬实时的姿态解算与运动控制。

本项目集成了 **Web 可视化控制台**，无需安装任何上位机软件，通过浏览器即可完成参数整定、状态监控和传感器校准。

---

## 🚀 核心特性 (Key Features)

- **高性能架构**
  - **双核异构调度**：Core 1 负责硬实时计算（传感器 / 估算 / 控制），Core 0 负责通信（Wi‑Fi / WebServer）。
  - **DataBus 总线**：基于 `Topic`（状态流）和 `CommandQueue`（指令流）的解耦架构，支持低延迟任务唤醒。

- **专业级状态估计 (State Estimation)** [NEW]
  - **Eigen 数学库**：集成 Eigen 线性代数库，支持高效的矩阵运算。
  - **EKF 框架**：内置扩展卡尔曼滤波 (Extended Kalman Filter) 核心，支持多速率传感器融合（Multi-rate Fusion）。
  - **零延迟同步**：估算任务通过信号量与 IMU 中断严格同步，最大限度减少积分漂移。

- **交互式 Web 控制台 (Web Dashboard)**
  - **零依赖**：基于 HTML5 / Bootstrap / Three.js，前端资源直接存储在片上 Flash（SPIFFS）中。
  - **WebSocket 通信**：毫秒级低延迟数据传输，支持实时波形显示与 3D 姿态同步。
  - **mDNS 支持**：支持通过 `http://espcortex.local` 域名直接访问，无需手动查询 IP。

- **智能参数系统 (ParamRegistry)**
  - **远程在线调参**：支持通过 Web 界面实时查看和修改参数（如 PID、EKF 噪声矩阵），修改后立即生效。
  - **持久化存储**：基于 NVS (Non-Volatile Storage)，支持 `float` / `int32_t` 等参数掉电保存。
  - **动态注册**：模块化设计，支持在运行时动态注册参数，无需在单一集中表中硬编码。
  - **僵尸参数检测**：自动识别 Flash 中残留的废弃参数并提供清理接口。

- **强大的传感器驱动**
  - **IMU**：支持 **ICM-42688-P**（SPI，200 Hz），采用 GPIO 中断驱动，实现极低延迟读取。
  - **磁力计**：支持 **QMC5883L**（I2C），采用主从分频读取策略。
  - **气压计**：支持 **ICP-20100**（I2C），采用主从分频读取策略。

---

## 🛠️ 硬件规格 (Hardware Specifications)

| 组件              | 型号              | 接口          | 频率 (ODR)  | 备注                               |
| :---------------- | :---------------- | :------------ | :---------- | :--------------------------------- |
| **MCU** | ESP32-S3-WROOM-1  | -             | 240 MHz     | Flash: 16 MB, PSRAM: 8 MB          |
| **IMU** | ICM-42688-P       | SPI2 (FSPI)   | 200 Hz      | 主时钟源，触发中断                 |
| **Magnetometer** | QMC5883L          | I2C           | 50 / 100 Hz | 配合 IMU 分频读取                  |
| **Barometer** | ICP-20100         | I2C           | 25 / 40 Hz  | 配合 IMU 分频读取                  |

### 🔌 引脚定义 (Pin Map)

**SPI 总线（IMU）**

- **MOSI**: GPIO 4  
- **MISO**: GPIO 5  
- **SCLK**: GPIO 6  
- **CS**: GPIO 7  
- **INT**: GPIO 15 （Active High, Push-Pull）

**I2C 总线（Mag & Baro）**

- **SDA**: GPIO 10  
- **SCL**: GPIO 11  

---

## 📂 项目结构 (Project Structure)

```text
EspCortex/
├── CMakeLists.txt          # 项目构建脚本
├── data/                   # 前端网页资源 (HTML/JS/CSS) -> 烧录至 SPIFFS
├── components/             # 独立驱动组件库
│   ├── common/             # [NEW] 通用类型定义 (shared_types.h)
│   ├── ekf/                # [NEW] EKF 数学核心库 (依赖 Eigen)
│   ├── web_server/         # HTTP/WebSocket 服务器 (含 JSON 解析)
│   ├── wifi_manager/       # WiFi 连接管理 (STA/AP 自动切换)
│   ├── spiffs_manager/     # SPIFFS 文件系统挂载与管理
│   ├── param_registry/     # 参数管理系统 (NVS + Mutex + JSON 序列化)
│   ├── spi_bus/            # 通用 SPI 总线封装 (C++)
│   ├── i2c_bus/            # 通用 I2C 总线封装 (C++)
│   ├── icm42688/           # ICM-42688-P 驱动
│   ├── qmc5883l/           # QMC5883L 驱动
│   └── icp20100/           # ICP-20100 驱动
├── main/
│   ├── include/
│   │   ├── board_config.h  # 全局引脚定义
│   │   └── robot_bus.hpp   # DataBus 总线定义
│   ├── modules/
│   │   ├── sensor/         # 传感器采集任务 (Task Sensor)
│   │   └── estimator/      # [NEW] 姿态估算任务 (Task Estimator)
│   └── main.cpp            # 系统入口与任务启动
└── sdkconfig               # ESP-IDF 项目配置 (Flash = 16 MB)
```

## 🌐 网络与使用 (Connectivity)

设备启动后会自动尝试连接预设的 Wi-Fi。

- **STA 模式（默认）**：尝试连接已配置的路由器。  
- **AP 模式（后备）**：如果连接失败，自动开启热点：  
  - SSID: `EspCortex`  
  - Password: `12345678`  

访问控制台：

1. 电脑 / 手机连接到同一网络（或直接连到 AP 热点）。  
2. 打开浏览器，访问：  
   - `http://espcortex.local`（推荐，依赖 mDNS）；或  
   - 通过串口日志查看设备 IP 地址，直接访问 `http://<device-ip>`。

---

## 🔨 编译与烧录 (Build & Flash)

本项目基于 **ESP-IDF v5.x** 开发。

### 1. 配置项目

```bash
idf.py menuconfig
```

确保：

- 已开启 HTTP Server / WebSocket 支持；  
- Partition Table 中为 SPIFFS / NVS 分配了足够的 storage 分区。

### 2. 编译代码

```bash
idf.py build
```

> 注意：初次编译时，构建系统会自动下载 Eigen 等托管组件，需保持网络连接。

### 3. 烧录固件与网页

> ⚠️ 注意：初次使用，或每次修改 `data/` 目录下的网页文件后，均需要重新烧录 SPIFFS 分区。

```bash
# 烧录固件
idf.py flash

# 烧录网页资源
# 方法 A (推荐): 使用 VS Code 插件的 "ESP-IDF: Flash SPIFFS Partition"
# 方法 B: 使用分区工具生成 bin 并烧录
```

### 4. 串口监控

```bash
idf.py monitor
```

按 `Ctrl+]` 可退出监控。

---

## 📅 开发日志 (Changelog)

### EKF & Eigen Integration (Current)

- 架构重构：抽离 `common` 组件，优化头文件依赖关系。  
- 核心升级：引入 Eigen 数学库，实现 `EspEKF` 类基础框架（预测步）。  
- 任务调度：新增 `task_estimator`，实现 IMU 数据驱动的零延迟姿态解算。  

### Initial Commit

- 完成基础架构搭建（DataBus + 传感器任务框架）。  

### Driver Update

- 集成 ICM-42688-P（SPI IRQ）、QMC5883L、ICP-20100 驱动。  

### System Update (ParamRegistry)

- 实现基于 NVS 的参数持久化存储。  
- 支持参数的动态注册与僵尸参数清理。  

### Feature Add (Web Control)

- 集成 `esp_http_server` 与 `mdns` 组件。  
- 实现 WebSocket 双向通信。  
- 完成 Web 端参数列表的拉取与在线修改功能（基于 JSON 协议）。  
