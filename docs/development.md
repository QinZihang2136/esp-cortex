# 开发环境与编译指南 (Development Guide)

## 概述

本文档介绍如何搭建 EspCortex 项目的开发环境，以及如何编译、烧录和调试固件。

## 系统要求

### 硬件要求

- **ESP32-S3 开发板** (推荐: ESP32-S3-DevKitC-1)
  - Flash: ≥ 4 MB (本项目使用 16 MB)
  - PSRAM: ≥ 8 MB (推荐)
- **USB 数据线** (支持数据传输，非仅充电)
- **传感器模块**:
  - ICM-42688-P (IMU)
  - QMC5883L (磁力计)
  - ICP-20100 或 LPS22HH (气压计)

### 软件要求

- **操作系统**:
  - Windows 10/11
  - macOS 10.15+
  - Linux (Ubuntu 20.04+)

- **ESP-IDF**: v5.1 或更高版本

- **工具链**:
  - CMake 3.16+
  - Python 3.8+
  - Git

## 开发环境搭建

### 方法 1: 使用 VS Code + ESP-IDF 插件 (推荐)

#### 步骤 1: 安装 VS Code

1. 下载并安装 [VS Code](https://code.visualstudio.com/)
2. 打开 VS Code

#### 步骤 2: 安装 ESP-IDF 插件

1. 点击左侧 **扩展** 图标 (或按 `Ctrl+Shift+X`)
2. 搜索 `ESP-IDF`
3. 安装 **Espressif ESP-IDF** 官方插件

#### 步骤 3: 初始化 ESP-IDF

1. 按 `F1` 打开命令面板
2. 输入 `ESP-IDF: Configure ESP-IDF extension`
3. 选择 **Use existing setup**
4. 选择 **Express installation** (自动下载 ESP-IDF v5.1)
5. 等待安装完成 (~30 分钟)

#### 步骤 4: 克隆项目

```bash
git clone https://github.com/your-repo/EspCortex.git
cd EspCortex
```

#### 步骤 5: 打开项目

1. VS Code → 文件 → 打开文件夹
2. 选择 `EspCortex` 文件夹
3. VS Code 会自动识别 ESP-IDF 项目

### 方法 2: 命令行搭建 (Linux/macOS)

#### 步骤 1: 安装依赖

**Ubuntu**:
```bash
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

**macOS**:
```bash
brew install cmake python3 python3-venv ninja dfu-util
```

#### 步骤 2: 克隆并安装 ESP-IDF

```bash
# 克隆 ESP-IDF
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout v5.1
git submodule update --init --recursive

# 安装工具链
./install.sh esp32s3

# 设置环境变量
. ./export.sh
```

#### 步骤 3: 克隆 EspCortex 项目

```bash
cd ~
git clone https://github.com/your-repo/EspCortex.git
cd EspCortex
```

## 项目配置

### 配置目标芯片

```bash
idf.py set-target esp32s3
```

### 配置项目参数

```bash
idf.py menuconfig
```

**关键配置项**:

```
→ Component config
  → ESP32S3-Specific
    → CPU frequency
      - 240 MHz (推荐)

  → Wi-Fi
    → Enable WiFi
    - Default WiFi TX power: 20 dBm

  → FreeRTOS
    → Tick rate (Hz)
      - 1000 Hz (默认)

  → FAT Filesystem
    → Long filenames support
    - Enable

→ Partition Table
  - Custom partition table (分区表: partitions.csv)
```

### 自定义分区表

**文件**: `partitions.csv`

```csv
# Name,   Type, SubType, Offset,  Size,    Flags
nvs,      data, nvs,     0x9000,  0x6000,
phy_init, data, phy,     0xf000,  0x1000,
factory,  app,  factory, 0x10000, 1M,
storage,  data, spiffs,  ,        2M,
```

**说明**:
- `nvs`: 参数存储 (24 KB)
- `factory`: 固件分区 (1 MB)
- `storage`: SPIFFS 文件系统 (2 MB，存放 Web 资源)

## 编译固件

### 完整编译

```bash
idf.py build
```

### 仅编译 (不烧录)

```bash
idf.py app
```

### 清理构建文件

```bash
idf.py fullclean
```

## 烧录固件

### 方法 1: 使用 VS Code

1. 连接 ESP32-S3 开发板 (USB 线)
2. 点击底部状态栏的 **烧录图标** (⚡)
3. 或按 `F1` → `ESP-IDF: Flash Device`

### 方法 2: 命令行

```bash
# 烧录固件
idf.py flash

# 烧录并立即监控串口
idf.py flash monitor
```

### 指定端口烧录

```bash
# Windows
idf.py -p COM3 flash monitor

# Linux
idf.py -p /dev/ttyUSB0 flash monitor

# macOS
idf.py -p /dev/tty.usbserial-0001 flash monitor
```

## 烧录 SPIFFS 文件系统

### 方法 1: VS Code (推荐)

1. 按 `F1` → `ESP-IDF: Flash SPIFFS Partition`
2. 等待烧录完成

### 方法 2: 命令行

```bash
# 构建SPIFFS镜像
idf.py build-flash

# 烧录SPIFFS分区
idf.py flash
```

**注意**: 修改 `data/` 目录内容后，必须重新烧录 SPIFFS！

## 监控与调试

### 串口监控

```bash
idf.py monitor
```

**快捷键**:
- `Ctrl+]` - 退出监控
- `Ctrl+T`, `Ctrl+L` - 切换 DTR/RTS
- `Ctrl+T`, `Ctrl+R` - 重启设备

### 编译 + 烧录 + 监控 (一键完成)

```bash
idf.py build flash monitor
```

### 查看 Flash 使用情况

```bash
idf.py partition-table
```

**输出示例**:
```
# Name            Type          SubType      Offset   Size      Flags
nvs              data          nvs          0x9000   0x6000
phy_init         data          phy          0xf000   0x1000
factory          app           factory      0x10000  0x100000
storage          data          spiffs              0x200000
```

## 调试技巧

### 1. 启用调试输出

修改 `components/common/include/debug_log.hpp`:

```cpp
// 全局调试开关
#define DEBUG_LOG_GLOBAL_ENABLE 1

// 开启详细输出
#define DEBUG_SENSOR_IMU 1
#define DEBUG_ESTIMATOR_EULER 1
```

### 2. 查看系统日志

```bash
idf.py monitor
```

**日志级别**:
- `E` - Error (错误)
- `W` - Warning (警告)
- `I` - Info (信息)
- `D` - Debug (调试)
- `V` - Verbose (详细)

**过滤日志**:
```bash
# 只显示 ERROR
idf.py monitor | grep E

# 只显示传感器任务日志
idf.py monitor | grep SENSOR
```

### 3. 性能分析

**查看任务栈使用**:

```bash
idf.py monitor
# 输入 Ctrl+T, Ctrl+H
```

**输出示例**:
```
Task | Stack | Watermark | CPU
Sensor | 8192 | 4532 | 12%
Estimator | 8192 | 3210 | 25%
Telemetry | 4096 | 2134 | 5%
```

**Watermark 说明**:
- 剩余栈空间
- 如果 < 500 字节，需要增大栈大小

### 4. 内存分析

**查看堆内存**:

```cpp
ESP_LOGI(TAG, "Free heap: %d KB", esp_get_free_heap_size() / 1024);
```

**查看最大可分配块**:

```cpp
ESP_LOGI(TAG, "Largest free block: %d KB",
         heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) / 1024);
```

### 5. GDB 调试

**启动 GDB 调试器**:

```bash
idf.py gdb
```

**VS Code GDB 调试**:

1. 按 `F5` 或点击 **调试** 图标
2. 选择 **ESP-IDF Debug** 配置
3. 设置断点并调试

**GDB 常用命令**:
```
break main                    # 设置断点
continue                     # 继续执行
step                         # 单步执行
print variable_name          # 打印变量
backtrace                    # 查看调用栈
```

## 常见编译问题

### Q: 编译错误 "fatal error: eigen3/Eigen/Dense: No such file or directory"

**A**: Eigen 组件未正确添加

```bash
cd components/eigen
git submodule update --init --recursive
```

### Q: 链接错误 "undefined reference to `esp_timer_get_time'"

**A**: 缺少组件依赖

在 `main/CMakeLists.txt` 中添加:
```cmake
REQUIRES eigen ekf
```

### Q: Flash 空间不足

**A**: 检查分区表大小

```bash
idf.py partition-table
```

或减小固件大小:
```bash
idf.py menuconfig
→ Compiler options
  → Optimize for size (-Os)
```

### Q: 烧录失败 "Failed to connect"

**A**: 检查以下事项
1. USB 线是否支持数据传输
2. 端口是否被占用
3. 是否进入下载模式 (按住 BOOT 键后按 RESET)
4. 驱动是否正确安装 (CP2102/CH340)

## 工作流示例

### 典型开发流程

```bash
# 1. 修改代码
vim main/modules/sensor/task_sensor.cpp

# 2. 编译
idf.py build

# 3. 烧录固件
idf.py flash

# 4. 烧录 SPIFFS (如果修改了 data/)
idf.py flash

# 5. 监控串口
idf.py monitor
```

### 仅修改 C/C++ 代码

```bash
idf.py build flash monitor
```

### 修改 Web 前端 (data/)

```bash
idf.py flash  # 烧录 SPIFFS 分区
```

### 修改分区表

```bash
idf.py fullclean  # 清理构建
idf.py build flash monitor
```

## 性能优化

### 加速编译

**使用 ccache**:
```bash
idf.py menuconfig
→ Compiler options
  → Enable ccache
```

**并行编译**:
```bash
idf.py -j8 build  # 使用8线程
```

### 减小固件大小

1. **启用 LTO (Link Time Optimization)**:
```bash
idf.py menuconfig
→ Compiler options
  → Enable link-time optimization (LTO)
```

2. **移除调试符号**:
```bash
idf.py menuconfig
→ Bootloader config
  → Bootloader log verbosity
    → No output
```

3. **优化级别**:
```bash
idf.py menuconfig
→ Compiler options
  → Optimization Level
    → Optimize for size (-Os)
```

## 相关文件

- **构建配置**: [CMakeLists.txt](../CMakeLists.txt)
- **主程序构建**: [main/CMakeLists.txt](../main/CMakeLists.txt)
- **分区表**: [partitions.csv](../partitions.csv)
- **项目配置**: [sdkconfig](../sdkconfig)

## 参考资料

- **ESP-IDF 编程指南**: https://docs.espressif.com/projects/esp-idf/en/latest/
- **ESP-IDF VS Code 插件**: https://github.com/espressif/vscode-esp-idf-extension
- **ESP32-S3 技术手册**: https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_cn.pdf
- **FreeRTOS 文档**: https://www.freertos.org/RTOS.html
