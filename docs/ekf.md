# 姿态估计模块 (EKF Module)

## 概述

姿态估计模块采用**误差状态扩展卡尔曼滤波器 (ES-EKF)** 融合 IMU、加速度计和磁力计数据，实时解算设备的姿态（Roll/Pitch/Yaw）。

## 核心特性

- **9 轴传感器融合**：陀螺仪 + 加速度计 + 磁力计
- **误差状态卡尔曼滤波**：数值稳定性好，收敛速度快
- **倾斜补偿航向融合**：避免磁倾角对 Roll/Pitch 的干扰
- **零偏在线估计**：实时估计并补偿陀螺仪零偏
- **Eigen 数学库**：高效矩阵运算

## 算法原理

### ES-EKF 结构

```
┌─────────────────────────────────────────────────────────┐
│                   ES-EKF 滤波器                         │
├─────────────────────────────────────────────────────────┤
│  状态向量 (6维):                                         │
│  x = [δθ_x, δθ_y, δθ_z, b_gx, b_gy, b_gz]^T             │
│                                                          │
│  其中:                                                   │
│  - δθ: 姿态误差 (3轴)                                    │
│  - b_g: 陀螺仪零偏 (3轴)                                 │
└─────────────────────────────────────────────────────────┘
         │                    │                    │
    ┌────▼────┐          ┌────▼────┐         ┌───▼────┐
    │ Gyro    │          │ Accel   │         │ Magnet │
    │ (预测)  │          │ (修正)  │         │ (修正)  │
    └────┬────┘          └────┬────┘         └───┬────┘
         │                    │                    │
         │               Roll/Pitch             Yaw
         │               修正                   修正
```

### 预测步 (Predict Step)

陀螺仪积分预测姿态：

```
ω_corrected = ω_measured - b_g (去除零偏)

q_k = q_{k-1} ⊗ [cos(θ/2), sin(θ/2)·ω] (四元数积分)

P_k = F·P_{k-1}·F^T + Q (协方差传播)
```

**F 矩阵** (状态转移矩阵):
```
F = [ I - [ω]x   -I·dt ]
    [   0          I    ]
```

### 更新步 1: 加速度计融合 (fuse_accel)

**目的**: 修正 Roll 和 Pitch

**原理**: 加速度计在静止时测量重力方向

```
z = a_measured / |a_measured| (归一化测量)

h(x) = R(q)^T · [0, 0, 1]^T (预测重力方向)

y = z - h(x) (残差/新息)

K = P·H^T·(H·P·H^T + R)^-1 (卡尔曼增益)

δx = K·y (误差状态)

注入误差:
  q = q ⊗ [1, 0.5·δθ]
  b_g = b_g + δb
```

**关键约束**:
```cpp
K.row(2).setZero(); // 禁止 accel 更新 yaw
K.row(5).setZero(); // 禁止 accel 更新 bias_z
```

### 更新步 2: 磁力计融合 (fuse_mag)

**目的**: 修正 Yaw

**原理**: 磁力计提供航向参考

```
m_body = mag_measured / |mag_measured|
m_world = q · m_body (旋转到世界坐标系)

yaw_measured = atan2(m_world.y, m_world.x)
yaw_predicted = 从 q 提取
yaw_residual = yaw_measured - yaw_predicted

H = [0, 0, 1, 0, 0, 0] (只观测 yaw)

K = P·H^T / (H·P·H^T + R_mag)

δx = K · yaw_residual
```

**关键约束**:
```cpp
K(0) = 0.0f; // roll
K(1) = 0.0f; // pitch
K(3) = 0.0f; // bias_x
K(4) = 0.0f; // bias_y
// K(2) yaw 保留
// K(5) bias_z 保留
```

## 坐标系

### 世界坐标系 (NED - 北东地)

- **X轴**: 指向正北
- **Y轴**: 指向正东
- **Z轴**: 指向地心

### 机体坐标系 (FRD - 前右下)

- **X轴**: 指向前方
- **Y轴**: 指向右侧
- **Z轴**: 指向下方

**姿态角定义**:
- **Roll (φ)**: 绕 X 轴旋转（横滚角）
- **Pitch (θ)**: 绕 Y 轴旋转（俯仰角）
- **Yaw (ψ)**: 绕 Z 轴旋转（偏航角）

## 使用方法

### 初始化

```cpp
// main/modules/estimator/task_estimator.cpp

EspEKF ekf;

// 使用加速度计初始化姿态
ekf.init(Eigen::Vector3f(0.0f, 0.0f, 9.81f));
```

### 预测步 (每 IMU 周期)

```cpp
// 获取陀螺仪数据 (rad/s)
Eigen::Vector3f gyro;
gyro.x() = imu_data.gx;
gyro.y() = imu_data.gy;
gyro.z() = imu_data.gz;

// 计算时间步长
float dt = (now_imu_us - last_imu_time_us) * 1e-6f;

// EKF 预测
ekf.predict(gyro, dt);
```

### 加速度计融合 (条件触发)

```cpp
// 归一化加速度
Eigen::Vector3f accel;
accel.x() = -imu_data.ax;
accel.y() = -imu_data.ay;
accel.z() = -imu_data.az;

// 检查是否适合融合 (静止或匀速)
float acc_norm = accel.norm();
if (acc_norm > 5.0f && acc_norm < 15.0f) {
    ekf.fuse_accel(accel);
}
```

### 磁力计融合 (每磁力计周期)

```cpp
// 归一化磁场
Eigen::Vector3f mag;
mag.x() = mag_data.x;
mag.y() = mag_data.y;
mag.z() = mag_data.z;

// EKF 融合
ekf.fuse_mag(mag);
```

### 获取姿态角

```cpp
// 获取欧拉角 (弧度)
Eigen::Vector3f euler = ekf.get_euler_angles();

// 转换为角度
float roll_deg = euler.x() * 57.29578f;
float pitch_deg = euler.y() * 57.29578f;
float yaw_deg = euler.z() * 57.29578f;
```

### 获取陀螺仪零偏

```cpp
Eigen::Vector3f bias = ekf.get_gyro_bias();

ESP_LOGI(TAG, "Gyro Bias: x=%.5f, y=%.5f, z=%.5f rad/s",
         bias.x(), bias.y(), bias.z());
```

## 调试输出

### EKF 内部状态可视化

Web 界面实时显示：

**航向残差**:
```
yaw_measured:  磁力计实测航向
yaw_predicted: 当前姿态预测航向
yaw_residual:  残差 (正常 < 2°)
```

**零偏估计**:
```
bias_x, bias_y, bias_z: 陀螺仪零偏 (rad/s)
静止时应收敛到常数
```

**卡尔曼增益**:
```
K(yaw, bias_z): 磁力计对 bias_z 的修正权重
正常值: 0.001 ~ 0.01
```

**协方差**:
```
P_yaw: yaw 的不确定性
应逐渐收敛到 < 0.01
```

### 调试日志控制

```cpp
// components/common/include/debug_log.hpp

#define DEBUG_ESTIMATOR_EULER 1    // 欧拉角输出
#define DEBUG_ESTIMATOR_GYRO 1     // 陀螺仪和零偏
#define DEBUG_ESTIMATOR_ACCEL 1    // 加速度计融合状态
#define DEBUG_ESTIMATOR_MAG 1      // 磁力计数据
#define DEBUG_ESTIMATOR_TIME 1     // 时间戳健康度
#define DEBUG_ESTIMATOR_STATISTICS 1 // 统计计数器
```

**输出示例**:
```text
[ESTIMATOR] Euler(deg) R:   0.12   P:   1.23   Y:  45.67 | Ydot:   0.12 deg/s
[ESTIMATOR] Gyro(rad/s) [0.00123 0.00045 0.00012] | Bias [0.00001 0.00002 0.00003]
[ESTIMATOR] AccelUsed [-0.01 0.02 9.81] | AccNorm: 9.81 | Gate:OK
[ESTIMATOR] Stats imu=1000 back=0 cap=0 acc_ok=950 acc_rej=50 mag_ok=100 mag_rej=0
```

## 参数调优

### 过程噪声矩阵 Q

**作用**: 描述系统模型的不确定性

```cpp
// ekf.cpp - 构造函数
Q.setIdentity();
Q.topLeftCorner(3, 3) *= 0.0005f;      // 角度积分噪声
Q.bottomRightCorner(3, 3) *= 0.00001f; // 零偏随机游走
```

**调优建议**:
- **姿态漂移大**: 增大 `q_angle`
- **零偏收敛慢**: 增大 `q_bias`
- **噪声敏感**: 减小 `q_bias`

### 观测噪声矩阵 R

**加速度计噪声 R_accel**:
```cpp
R_accel.setIdentity();
R_accel *= 0.05f; // 震动大时调大到 0.1~0.5
```

**磁力计噪声 R_mag**:
```cpp
R_mag.setIdentity();
R_mag *= 0.1f; // 磁干扰大时调大
```

**调优建议**:
- **Roll/Pitch 抖动**: 增大 `R_accel`
- **Yaw 抖动**: 增大 `R_mag`
- **收敛慢**: 减小对应 R 值

### 融合门限

**加速度计融合门限**:
```cpp
// task_estimator.cpp
if (acc_norm > 5.0f && acc_norm < 15.0f) {
    // 融合加速度计
}
```

**磁力计融合门限**:
```cpp
const float mag_min_valid = 0.15f;  // Gauss
const float mag_max_valid = 1.0f;   // Gauss

if (mag_norm > mag_min_valid && mag_norm < mag_max_valid) {
    // 融合磁力计
}
```

## 常见问题

### Q: Yaw 角持续抖动？

**可能原因**:
1. 磁力计数据噪声大
2. 磁干扰未校准
3. R_mag 值过小

**解决方法**:
1. 重新进行磁力计椭球拟合校准
2. 增大 `R_mag *= 0.2f`
3. 检查 Web 界面 `yaw_residual` 是否正常 (< 5°)

### Q: Roll/Pitch 漂移？

**可能原因**:
1. 加速度计融合被频繁拒绝
2. R_accel 值过大

**解决方法**:
1. 检查 `accel_used` 统计，确认融合率 > 80%
2. 减小 `R_accel *= 0.03f`
3. 确认加速度计已校准

### Q: 姿态响应慢？

**可能原因**:
1. 过程噪声 Q 过小
2. 零偏估计错误

**解决方法**:
1. 增大 `Q.topLeftCorner(3,3) *= 0.001f`
2. 检查 `gyro_bias` 是否正常 (< 0.01 rad/s)

### Q: 初始姿态错误？

**可能原因**:
1. 初始化时设备未水平
2. 加速度计零偏未校准

**解决方法**:
1. 初始化时保持设备水平静止
2. 重新校准加速度计六面偏置

## 性能指标

| 指标 | 数值 |
|------|------|
| 更新频率 | 200 Hz (IMU 驱动) |
| 姿态延迟 | < 5 ms |
| Roll/Pitch 精度 | < 1° (静态) |
| Yaw 精度 | < 2° (已校准) |
| 零偏收敛时间 | ~10 s |
| CPU 占用 (Core 1) | ~25% |
| 内存占用 | ~50 KB (Eigen 栈) |

## 相关文件

- **EKF 核心**: [components/ekf/ekf.cpp](../components/ekf/ekf.cpp)
- **EKF 头文件**: [components/ekf/include/ekf.hpp](../components/ekf/include/ekf.hpp)
- **估计器任务**: [main/modules/estimator/task_estimator.cpp](../main/modules/estimator/task_estimator.cpp)
- **调试数据结构**: [components/common/include/shared_types.h](../components/common/include/shared_types.h)

## 参考文献

- **ES-EKF 理论**: Quaternion Kinematics for the Error-State Kalman Filter
- **传感器融合**: MEMS IMU 和磁力计数据融合方法
- **PX4 EKF2**: PX4 飞控 ECL 滤波器设计文档
