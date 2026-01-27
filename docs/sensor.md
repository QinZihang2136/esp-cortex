# 传感器采集模块 (Sensor Module)

## 概述

传感器模块负责驱动和管理所有机载传感器，包括IMU、磁力计和气压计。采用**中断驱动**和**任务通知**机制，实现低延迟、高频的数据采集。

## 架构设计

```
┌─────────────────────────────────────────────────────┐
│                  Sensor Task (Core 1)               │
│  Priority: 5  |  Stack: 8192  |  Frequency: 200Hz  │
└─────────────────────────────────────────────────────┘
                          │
         ┌────────────────┼────────────────┐
         │                │                │
    ┌────▼────┐     ┌─────▼─────┐    ┌───▼────┐
    │   IMU   │     │  Magnetom │    │ Baromtr │
    │ (SPI)   │     │   (I2C)   │    │  (I2C)  │
    │ 200 Hz  │     │  100 Hz   │    │ 75 Hz   │
    └────┬────┘     └─────┬─────┘    └───┬────┘
         │                │                │
         └────────────────┴────────────────┘
                          │
                    ┌─────▼─────┐
                    │ RobotBus  │
                    │  publish  │
                    └───────────┘
```

## 传感器规格

### IMU (ICM-42688-P)

**接口**: SPI (FSPI)
**频率**: 200 Hz
**用途**: 主时钟源，触发系统中断

**输出数据**:
- 加速度 (m/s²): ±16g 量程
- 角速度 (rad/s): ±2000°/s 量程
- 温度 (°C)

**关键配置**:
```cpp
// main/modules/sensor/task_sensor.cpp

// 加速度计配置
accel_fullscale = ICM42688_ACCEL_FS_16g;
accel_odr = ICM42688_ODR_200HZ;

// 陀螺仪配置
gyro_fullscale = ICM42688_GYRO_FS_2000DPS;
gyro_odr = ICM42688_ODR_200HZ;
```

### 磁力计 (QMC5883L)

**接口**: I2C (0x0D)
**频率**: 100 Hz (采样) / 50 Hz (融合)
**用途**: 航向测量

**输出数据**:
- 磁场强度 (Gauss): X, Y, Z 三轴

**校准支持**:
- 硬铁干扰补偿 (偏置)
- 软铁干扰补偿 (缩放因子)
- 椭球拟合校准

### 气压计 (ICP-20100 / LPS22HH)

**接口**: I2C (自动扫描 0x5C/0x5D)
**频率**: 25 Hz (ICP-20100) / 75 Hz (LPS22HH)
**用途**: 气压高度测量

**输出数据**:
- 气压 (Pa)
- 温度 (°C)
- 计算高度 (m)

## 坐标系映射

### FRD 坐标系

所有传感器数据统一映射到 **FRD (前-右-下)** 坐标系：

```
     X (Forward/前)
     ↑
     |
     |────→ Y (Right/右)
    /
   /
  ↓ Z (Down/下)
```

### 传感器安装方向配置

**IMU 坐标映射**:
```cpp
// task_sensor.cpp - apply_sensor_calibration()

// 加速度计符号映射 (根据实际安装方向调整)
imu_data.ax = raw_accel[0];  // X轴: 前方
imu_data.ay = raw_accel[1];  // Y轴: 右方
imu_data.az = raw_accel[2];  // Z轴: 下方

// 陀螺仪符号映射
imu_data.gx = raw_gyro[0];   // rad/s
imu_data.gy = raw_gyro[1];
imu_data.gz = raw_gyro[2];
```

**磁力计坐标映射**:
```cpp
// 磁力计已做 FRD 映射，直接使用
mag_vec.x() = mag_data_snapshot.x;
mag_vec.y() = mag_data_snapshot.y;
mag_vec.z() = mag_data_snapshot.z;
```

## 数据流程

### 1. IMU 数据采集流程

```
ICM-42688 INT
    ↓
GPIO ISR (vtasksensor_notify)
    ↓
ulTaskNotifyTake (阻塞等待通知)
    ↓
icm42688_read_accel_gyro()
    ↓
坐标映射 + 校准应用
    ↓
RobotBus::instance().imu.publish(imu_data)
    ↓
唤醒 Estimator Task
```

### 2. 磁力计数据采集流程

```
IMU 中断触发 (每 5ms)
    ↓
qmc5883l_read_mag()
    ↓
应用校准参数 (偏置/缩放)
    ↓
RobotBus::instance().mag.publish(mag_data)
    ↓
Estimator Task 融合
```

### 3. 气压计数据采集流程

```
主循环 (低优先级)
    ↓
icp20100_read_pressure() 或 lps22hh_read()
    ↓
计算海拔高度
    ↓
RobotBus::instance().baro.publish(baro_data)
```

## 校准系统

### 加速度计校准

**目的**: 消除传感器零偏和比例误差

**校准方法**: 六面校准法
1. 设备平放 (Z轴向下)
2. 设备倒放 (Z轴向上)
3. 设备左侧 (X轴向下)
4. 设备右侧 (X轴向上)
5. 设备头部 (Y轴向下)
6. 设备尾部 (Y轴向上)

**存储位置**: NVS 参数
```cpp
ACCEL_BIAS_X  // 加速度计零偏
ACCEL_BIAS_Y
ACCEL_BIAS_Z
ACCEL_SCALE_X // 加速度计缩放
ACCEL_SCALE_Y
ACCEL_SCALE_Z
```

**应用校准**:
```cpp
imu_data.ax = (raw_ax - ACCEL_BIAS_X) * ACCEL_SCALE_X;
imu_data.ay = (raw_ay - ACCEL_BIAS_Y) * ACCEL_SCALE_Y;
imu_data.az = (raw_az - ACCEL_BIAS_Z) * ACCEL_SCALE_Z;
```

### 磁力计校准

**目的**: 消除硬铁和软磁干扰

**校准方法**: 椭球拟合法
- 设备在三维空间中缓慢旋转
- 收集大量磁场样本点
- 拟合椭球方程，计算偏置和缩放矩阵

**存储位置**: NVS 参数
```cpp
MAG_BIAS_X    // 硬铁偏置
MAG_BIAS_Y
MAG_BIAS_Z
MAG_SCALE_X   // 软磁缩放
MAG_SCALE_Y
MAG_SCALE_Z
```

**Web 界面校准**:
1. 访问 Web 控制台 → 传感器标定
2. 点击"开始磁力计校准"
3. 按照提示缓慢旋转设备
4. 完成后参数自动保存至 NVS

## 调试输出

### 模块化调试控制

通过 `debug_log.hpp` 精确控制打印输出：

```cpp
// components/common/include/debug_log.hpp

// 传感器任务调试开关
#define DEBUG_SENSOR_IMU 1          // IMU 原始数据
#define DEBUG_SENSOR_MAG 1          // 磁力计数据
#define DEBUG_SENSOR_BARO 1         // 气压计数据
#define DEBUG_SENSOR_I2C_SCAN 1     // I2C 扫描结果
#define DEBUG_SENSOR_CALIB 1        // 校准参数
```

### 调试输出示例

```text
[SENSOR] IMU: ax=0.012 ay=0.023 az=9.810 | gx=0.00123 gy=-0.00045 gz=0.00012
[SENSOR] Mag: x=0.234 y=-0.123 z=0.456
[SENSOR] Baro: P=101325 Pa | T=25.3 C | Alt=102.4 m
```

## 性能指标

| 指标 | 数值 |
|------|------|
| IMU 采样率 | 200 Hz |
| IMU 延迟 | < 1 ms |
| 磁力计采样率 | 100 Hz |
| 气压计采样率 | 25-75 Hz |
| 任务优先级 | 5 (高优先级) |
| 任务栈大小 | 8192 bytes |
| CPU 占用 (Core 1) | ~15% |

## 使用示例

### 查看传感器原始数据

1. 连接设备串口 (115200 波特率)
2. 查看终端输出：
```bash
idf.py monitor
```

### 调整 IMU 采样率

修改 `task_sensor.cpp`:
```cpp
// 修改为 100 Hz
accel_odr = ICM42688_ODR_100HZ;
gyro_odr = ICM42688_ODR_100HZ;
```

### 修改传感器安装方向

如果传感器安装方向与标准不同，调整符号映射：
```cpp
// 示例: X轴反向安装
imu_data.ax = -raw_accel[0];
imu_data.ay = raw_accel[1];
imu_data.az = raw_accel[2];
```

## 常见问题

### Q: IMU 数据不更新？

**A**: 检查以下几点：
1. SPI 接线是否正确 (MOSI/MISO/SCLK/CS)
2. INT 引脚是否连接到 GPIO 15
3. 查看终端是否有 ICM-42688 初始化成功信息

### Q: 磁力计读数为 0？

**A**:
1. 检查 I2C 地址扫描是否成功 (查看终端输出)
2. 确认 QMC5883L 供电正常
3. 尝试重新校准磁力计

### Q: 气压计高度跳变？

**A**:
1. 确认气压计型号已自动识别
2. 检查气压计是否受风扇气流干扰
3. 增加气压滤波参数

## 相关文件

- **源代码**: [main/modules/sensor/task_sensor.cpp](../main/modules/sensor/task_sensor.cpp)
- **头文件**: [main/modules/sensor/task_sensor.hpp](../main/modules/sensor/task_sensor.hpp)
- **调试控制**: [components/common/include/debug_log.hpp](../components/common/include/debug_log.hpp)
- **IMU 驱动**: [components/icm42688/](../components/icm42688/)
- **磁力计驱动**: [components/qmc5883l/](../components/qmc5883l/)
- **气压计驱动**: [components/icp20100/](../components/icp20100/)
