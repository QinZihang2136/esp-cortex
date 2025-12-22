#include "ekf.hpp" 
#include "esp_log.h"
#include <cmath>

static const char* TAG = "EKF";

// =========================================================================================
// 辅助函数: 角度归一化
// [Engineering] 防止角度积分无限增加导致精度丢失或控制逻辑错误。
// 保持角度在 [-PI, PI] 范围内。
// =========================================================================================
static float normalize_angle(float angle)
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

// 构造函数
EspEKF::EspEKF()
{
    // 1. 初始化状态向量 x (全0)
    // x = [Roll, Pitch, Yaw, Bias_Gx, Bias_Gy, Bias_Gz]^T
    x.setZero();

    // 2. 初始化协方差矩阵 P (对角阵)
    // [Engineering] P 代表我们对当前状态 x 的"不确定度"。
    // 初始时我们假设知道得大概准，但不完全准，给个初始方差。
    P.setIdentity();
    P = P * 0.1f;

    // 3. 初始化过程噪声矩阵 Q
    // [Math] Q 代表系统模型预测的不确定性 (Predict Step Noise)。
    // 即使没有外部干扰，积分过程也会累积误差。
    Q.setIdentity();
    // [Engineering] 这里的参数通常需要根据实际传感器噪声 spec 或实际飞行 log 调节。
    Q.topLeftCorner(3, 3) *= 0.001f;      // 角度积分噪声 (相信陀螺仪积分，但给一点点不确定性)
    Q.bottomRightCorner(3, 3) *= 0.0001f; // 零偏随机游走 (Bias 随时间缓慢漂移)

    // 4. 初始化观测噪声矩阵 R
    // [Math] R 代表传感器测量的不确定性 (Update Step Noise)。
    // 值越大，EKF 越不相信这个传感器，收敛越慢；值越小，越容易受震动干扰。
    R_accel.setIdentity();
    R_accel *= 0.1f; // 加速度计噪声 (震动大时应调大此值)

    R_mag.setIdentity();
    R_mag *= 0.1f;   // 磁力计噪声 (环境磁干扰大时应调大此值)

    ESP_LOGI(TAG, "EKF Initialized. Eigen Version: %d.%d.%d",
        EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION);

    is_initialized = true;
}

void EspEKF::init()
{
    // 复位所有状态，通常在解锁或断电重启时调用
    x.setZero();
    P.setIdentity();
    P = P * 0.1f;
    is_initialized = true;
}

// =========================================================================================
// 核心：预测函数 (Predict Step)
// [Math] x_{k|k-1} = f(x_{k-1}, u_k)
// [Math] P_{k|k-1} = F_k * P_{k-1} * F_k^T + Q_k
// =========================================================================================
void EspEKF::predict(const Vector3f& gyro, float dt)
{
    if (!is_initialized) return;

    // --- 1. 提取变量 (增加可读性) ---
    float phi = x(0);   // Roll
    float theta = x(1); // Pitch
    float psi = x(2);   // Yaw

    float bgx = x(3);   // Gyro Bias X
    float bgy = x(4);   // Gyro Bias Y
    float bgz = x(5);   // Gyro Bias Z

    // 减去估计的零偏，得到更准的角速度
    float gx = gyro.x() - bgx;
    float gy = gyro.y() - bgy;
    float gz = gyro.z() - bgz;

    // --- 2. 更新状态向量 x (非线性状态转移) ---
    // [Math] 姿态微分方程 (小角度近似，假设机体角速度 ≈ 欧拉角速度)
    // 如果是大机动，这里应该使用旋转矩阵运动学方程: \dot{\Theta} = W \cdot \omega
    // 对于无人机悬停和平稳飞行，简单的积分通常足够。
    float new_phi = phi + gx * dt;
    float new_theta = theta + gy * dt;
    float new_psi = psi + gz * dt;

    // [Engineering] 归一化角度，处理 +180 -> -180 的跳变问题
    x(0) = normalize_angle(new_phi);
    x(1) = normalize_angle(new_theta);
    x(2) = normalize_angle(new_psi);

    // Bias 保持不变 (假设为常量或随机游走模型: \dot{b} = 0 + noise)

    // --- 3. 计算雅可比矩阵 F (State Transition Jacobian) ---
    // [Math] F = \frac{\partial f}{\partial x}
    // 描述当前状态的微小变化如何传递到下一个时刻
    Matrix<float, X_DIM, X_DIM> F;
    F.setIdentity(); // 对角线为1，表示状态会保持下去

    // 填充偏导数项：角度对 Bias 的偏导
    // new_phi = phi - bgx * dt  => d(new_phi)/d(bgx) = -dt
    F(0, 3) = -dt;
    F(1, 4) = -dt;
    F(2, 5) = -dt;

    // --- 4. 更新协方差矩阵 P ---
    // [Math] P = F P F^T + Q
    // 预测步骤会增加不确定性 (加 Q)
    P = F * P * F.transpose() + Q;

    // [Engineering] 强制对称性，消除数值计算误差
    P = 0.5f * (P + P.transpose());
}

// =========================================================================================
// 更新步：融合加速度计 (Fuse Accelerometer)
// 用于修正 Roll 和 Pitch
// =========================================================================================
void EspEKF::fuse_accel(const Vector3f& accel)
{
    if (!is_initialized) return;

    // [Engineering] 震动/异常保护
    // 如果加速度模长偏离 1g 太远 (比如 >1.5g 或 <0.5g)，说明有非重力加速度干扰，不应融合。
    // 这里简单处理：如果太小认为是自由落体或错误，不融合。
    if (accel.norm() < 0.1f) return;

    // 归一化观测向量 (我们只关心重力的方向)
    Vector3f z = accel.normalized();

    // --- 1. 提取当前姿态 ---
    float phi = x(0);
    float theta = x(1);
    // Yaw 对重力投影没有影响，所以这里不需要 psi

    float sp = sin(phi);   float cp = cos(phi);
    float st = sin(theta); float ct = cos(theta);

    // --- 2. 计算预测观测值 h(x) ---
    // [Math] 坐标系定义: FRD (前-右-下)
    // 导航系重力向量: g_n = [0, 0, 1]^T (指向地心，正Z)
    // 加速度计测量的是比力 (Reaction Force): a_m = -R_{nb}^T * g_n = R_{nb}^T * [0, 0, -1]^T
    // 这相当于旋转矩阵 R_{nb} 的第3列乘以 -1。
    // R_{nb} (3-2-1 Euler) 的第3列为: [-sin\theta, sin\phi cos\theta, cos\phi cos\theta]^T
    // 取反后得到 h(x):
    Vector3f h_x;
    h_x(0) = st;           // sin(theta)
    h_x(1) = -sp * ct;     // -sin(phi) * cos(theta)
    h_x(2) = -cp * ct;     // -cos(phi) * cos(theta)

    // --- 3. 计算残差 (Innovation) ---
    Vector3f y = z - h_x;

    // --- 4. 计算雅可比矩阵 H (Observation Jacobian) ---
    // [Math] H = \frac{\partial h}{\partial x} (3x6矩阵)
    Matrix<float, 3, X_DIM> H;
    H.setZero();

    // 对 h_x[0] = sin(theta) 求导
    H(0, 1) = ct;          // d/dtheta

    // 对 h_x[1] = -sin(phi)cos(theta) 求导
    H(1, 0) = -cp * ct;    // d/dphi
    H(1, 1) = sp * st;     // d/dtheta (注意 cos导数是-sin，负负得正)

    // 对 h_x[2] = -cos(phi)cos(theta) 求导
    H(2, 0) = sp * ct;     // d/dphi (cos导数是-sin，负负得正)
    H(2, 1) = cp * st;     // d/dtheta

    // --- 5. 标准 EKF 更新 ---

    // 计算残差协方差 S = H P H^T + R
    Matrix<float, 3, 3> S = H * P * H.transpose() + R_accel;

    // 计算卡尔曼增益 K = P H^T S^{-1}
    Matrix<float, X_DIM, 3> K = P * H.transpose() * S.inverse();

    // 更新状态 x = x + K * y
    // [Math] x_new = x_old + Gain * Error
    x = x + K * y;

    // 更新协方差 P = (I - K H) P
    // [Math] 更新步会减小不确定性
    Matrix<float, X_DIM, X_DIM> I;
    I.setIdentity();
    P = (I - K * H) * P;
}

// =========================================================================================
// 更新步：融合磁力计 (Fuse Magnetometer)
// 主要用于修正 Yaw (偏航角)
// =========================================================================================
// =========================================================================================
// 更新步：融合磁力计 (改进版：倾斜补偿航向融合)
// 解决 "磁力计干扰 Roll/Pitch" 和 "磁倾角导致平放不平" 的问题
// =========================================================================================
void EspEKF::fuse_mag(const Vector3f& mag)
{
    if (!is_initialized) return;

    // [Engineering] 异常保护: 磁场模长过大过小都可能是干扰
    float mag_norm = mag.norm();
    if (mag_norm < 0.1f || mag_norm > 10.0f) return;

    // --- 1. 获取当前姿态 (Roll/Pitch) ---
    // 我们信任加速度计和陀螺仪算出来的水平姿态
    float phi = x(0);
    float theta = x(1);
    float psi = x(2); // 当前估计的 Yaw

    float sp = sin(phi);   float cp = cos(phi);
    float st = sin(theta); float ct = cos(theta);

    // --- 2. 倾斜补偿 (Tilt Compensation) ---
    // 将机体坐标系下的磁力计读数，反向旋转回“水平面” (Horizontal Plane)
    // 公式推导基于 FRD 旋转矩阵的逆变换

    float mag_x = mag.x();
    float mag_y = mag.y();
    float mag_z = mag.z();

    // 在水平面上，磁向量的 X 分量 (指向磁北)
    // Bx = mx * cos(theta) + my * sin(theta)*sin(phi) + mz * sin(theta)*cos(phi)
    float Bx = mag_x * ct + mag_y * st * sp + mag_z * st * cp;

    // 在水平面上，磁向量的 Y 分量 (指向磁东)
    // By = my * cos(phi) - mz * sin(phi)
    float By = mag_y * cp - mag_z * sp;

    // --- 3. 计算实测航向角 (Measured Yaw) ---
    // atan2(y, x) 返回范围 [-PI, PI]
    // 注意 FRD 坐标系下，Yaw 顺时针为正，而 atan2 逆时针为正，所以通常需要负号
    // 但根据右手定则和 atan2 定义：heading = atan2(-By, Bx)
    float yaw_measured = atan2(-By, Bx);

    // --- 4. 计算残差 (Innovation) ---
    // 也就是 "测量值 - 预测值"
    float yaw_predicted = psi;

    // [关键] 处理角度周期性问题 (-180 度 和 +180 度是同一个点)
    // 我们需要算出最小的旋转路径
    float yaw_error = normalize_angle(yaw_measured - yaw_predicted);

    // --- 5. 构建雅可比矩阵 H ---
    // 我们的观测方程简化为: y = x(2) + noise
    // 所以 H 矩阵只有 Yaw 那一项是 1，其他是 0
    // 这意味着：磁力计只准修正 Yaw，绝对不准碰 Roll 和 Pitch！
    Matrix<float, 1, X_DIM> H;
    H.setZero();
    H(0, 2) = 1.0f;

    // --- 6. 标准 EKF 更新 (标量版) ---
    // 因为观测维数变成了 1 维 (只观测 Yaw)，矩阵运算简化为标量运算

    // S = H P H^T + R
    // 提取 P 矩阵中 Yaw 的方差 P(2,2)
    float P_yaw = P(2, 2);
    float S = P_yaw + R_mag(0, 0); // R_mag 这里取一个标量值即可

    // K = P H^T S^-1
    // 计算卡尔曼增益向量
    VectorX K = P.col(2) * (1.0f / S);

    // 更新状态 x = x + K * error
    x = x + K * yaw_error;

    // 再次归一化角度，防止更新后溢出
    x(0) = normalize_angle(x(0));
    x(1) = normalize_angle(x(1));
    x(2) = normalize_angle(x(2));

    // 更新协方差 P = (I - K H) P
    Matrix<float, X_DIM, X_DIM> I;
    I.setIdentity();
    // K * H 会得到一个 6x6 矩阵，其中只有第 3 列(索引2)有值
    P = (I - K * H) * P;
}

Vector3f EspEKF::get_euler_angles()
{
    // 返回 [Roll, Pitch, Yaw]
    return Vector3f(x(0), x(1), x(2));
}

Vector3f EspEKF::get_gyro_bias()
{
    // 返回估计的陀螺仪零偏 [BiasX, BiasY, BiasZ] (Z轴零偏我们没有在predict里建模，这里返回0即可)
    // 修正：实际上我们在 x(3), x(4), x(5) 中都有状态，应该全部返回
    return Vector3f(x(3), x(4), x(5));
}

// 参数动态调整接口
void EspEKF::set_process_noise(float q_angle, float q_bias)
{
    Q.setIdentity();
    Q.topLeftCorner(3, 3) *= q_angle;
    Q.bottomRightCorner(3, 3) *= q_bias;
}

void EspEKF::set_measure_noise_accel(float r_accel)
{
    R_accel.setIdentity();
    R_accel *= r_accel;
}