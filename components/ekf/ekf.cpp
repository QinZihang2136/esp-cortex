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
    state = NominalState();

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

void EspEKF::init(const Vector3f& accel_meas)
{
    // 1. 归一化加速度 (只关心方向，不关心大小)
    Vector3f acc_norm = accel_meas.normalized();

    // 2. 计算初始 Roll 和 Pitch
    // 2. 计算初始欧拉角 (根据重力分量反推)
    // 注意：这里假设静止，所以加速度计测到的仅仅是重力方向
    float pitch = atan2f(-acc_norm.x(), sqrtf(acc_norm.y() * acc_norm.y() + acc_norm.z() * acc_norm.z()));
    float roll = atan2f(acc_norm.y(), acc_norm.z());
    float yaw = 0.0f; // 暂时假设 Yaw 为 0 (或者以后接磁力计)
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

    // 顺序：Z * Y * X (这是航空标准的旋转顺序)
    state.q = yawAngle * pitchAngle * rollAngle;
    state.q.normalize(); // 养成好习惯

    is_initialized = true;
    ESP_LOGI(TAG, "EKF initialized with Roll: %.2f, Pitch: %.2f", roll, pitch);
}

// =========================================================================================
// 核心：预测函数 (Predict Step)
// [Math] x_{k|k-1} = f(x_{k-1}, u_k)
// [Math] P_{k|k-1} = F_k * P_{k-1} * F_k^T + Q_k
// =========================================================================================
void EspEKF::predict(const Vector3f& gyro_meas, float dt)
{
    if (!is_initialized) return;


    // 1. 去零偏 (直接向量减法)
    Vector3f w_unbiased = gyro_meas - state.gyro_bias;
    // 2. 计算旋转向量 (角度 * 轴)
    Vector3f delta_theta = w_unbiased * dt;
    float angle = delta_theta.norm();

    // 3. 构建增量四元数 (利用 Eigen 的 AngleAxis 自动处理 sin/cos 和半角)
    Eigen::Quaternionf delta_q;

    if (angle > 1e-6f)
    {
        // 大角度：使用精确公式
        // axis = delta_theta / angle (归一化轴)
        delta_q = Eigen::AngleAxisf(angle, delta_theta / angle);
    }
    else
    {
        // 小角度近似 (泰勒展开，避免除以0)
        // [1, 0.5*dx, 0.5*dy, 0.5*dz]
        delta_q = Eigen::Quaternionf(1.0f, 0.5f * delta_theta.x(), 0.5f * delta_theta.y(), 0.5f * delta_theta.z());
        delta_q.normalize();
    }

    // 4. 姿态更新 (右乘，因为是机体坐标系的旋转)
    state.q = state.q * delta_q;

    // 5. 归一化 (防止积分漂移导致模长不为1)
    state.q.normalize();

    // ==========================================
    // P 矩阵预测 (Error State Covariance Update)
    // F = [ I   -dt * I ]
    //     [ 0      I    ]
    // ==========================================

    // 1. 构建 F 矩阵 (6x6)
    MatrixP F;
    F.setIdentity(); // 先填满对角线为 1

    // 右上角块：角度误差受零偏误差的影响
    // 物理意义：如果 Bias 估错了 1 deg/s，经过 dt 秒，角度就会错 dt 度。
    // 符号是负的，因为 state.q = state.q * delta_q，而测量值是 w - bias
    F.block<3, 3>(0, 3) = -dt * Matrix3f::Identity();

    // 2. 更新 P 矩阵
    // P = F * P * F^T + Q
    // 这一步告诉滤波器：随着时间推移，因为有积分，我们的不确定性(P)会越来越大
    P = F * P * F.transpose() + Q;

}

void EspEKF::fuse_accel(const Vector3f& accel_meas)
{
    if (!is_initialized) return;

    // [Engineering] 异常保护
    // 只有当加速度模长接近 1g (9.8 m/s^2 或 1.0g) 时，我们才认为它是重力。
    // 如果模长太大（震动/转弯离心力）或太小（自由落体），说明加速度计测的不是纯重力，不能用来修姿态。
    float acc_norm = accel_meas.norm();
    if (acc_norm < 0.8f || acc_norm > 1.2f)
    {
        return; // 放弃这次更新，相信陀螺仪积分
    }

    // 1. 准备测量值 (z)
    // 归一化，因为我们只关心重力的"方向"来修正姿态
    Vector3f z = accel_meas.normalized();

    // 2. 准备预测值 h(x)
    // 利用当前名义姿态 state.q，把世界系重力 [0,0,1] 旋转到机体坐标系
    // [Math] g_pred = R(q)^T * [0,0,1]
    // Eigen 的 inverse() * vector 自动处理了共轭旋转 (即 R^T * v)
    Vector3f g_world = Vector3f::UnitZ();
    Vector3f g_pred = state.q.inverse() * g_world;

    // 3. 计算残差 (Innovation)
    // [Math] y = z - h(x)
    Vector3f y = z - g_pred;

    // 4. 构建观测雅可比矩阵 H (Measurement Jacobian)
    // [Math] H = [ [g_pred]x   0 ]  (维度 3x6)
    // 左边是 g_pred 的反对称矩阵，右边是 0 (因为加速度计无法观测陀螺仪零偏)
    Eigen::Matrix<float, 3, DIM_ERR> H;
    H.setZero();

    // 填充左上角 3x3 (反对称矩阵 Skew-symmetric matrix)
    // [  0   -gz   gy ]
    // [  gz   0   -gx ]
    // [ -gy   gx    0 ]
    H(0, 1) = -g_pred.z();
    H(0, 2) = g_pred.y();
    H(1, 0) = g_pred.z();
    H(1, 2) = -g_pred.x();
    H(2, 0) = -g_pred.y();
    H(2, 1) = g_pred.x();

    // 5. 计算卡尔曼增益 K
    // [Math] S = H * P * H^T + R
    // [Math] K = P * H^T * S^-1

    // R_accel 是 3x3 矩阵
    Eigen::Matrix3f S = H * P * H.transpose() + R_accel;

    // 计算 K (6x3 矩阵)
    // 这里的 S.inverse() 对 3x3 矩阵很快，不用担心性能
    Eigen::Matrix<float, DIM_ERR, 3> K = P * H.transpose() * S.inverse();

    // 6. 计算误差状态 delta_x (Error State)
    // [Math] dx = K * y
    // delta_x 是 6维向量: [d_roll, d_pitch, d_yaw, d_bgx, d_bgy, d_bgz]
    Eigen::Matrix<float, DIM_ERR, 1> delta_x = K * y;

    // ==========================================================
    // 7. 误差注入 (Error Injection) - ES-EKF 的灵魂
    // 这一步把算出来的"线性误差"融合进"非线性真身"里
    // ==========================================================

    // A. 修正姿态 (名义四元数)
    // 取出前 3 维 (角度误差)
    Vector3f delta_theta = delta_x.head<3>();

    // 把角度误差向量转换成微小旋转四元数
    // 使用小角度近似: dq = [1, 0.5*dx, 0.5*dy, 0.5*dz]
    Eigen::Quaternionf dq;
    dq.w() = 1.0f;
    dq.vec() = 0.5f * delta_theta; // 实部是1，虚部是角度的一半

    // 注入误差：q_new = q_old * dq
    state.q = state.q * dq;
    state.q.normalize(); // 必须归一化，否则误差累积会导致模长发散

    // B. 修正零偏 (名义零偏)
    // 取出后 3 维 (零偏误差) 并直接加到当前零偏上
    state.gyro_bias += delta_x.tail<3>();

    // ==========================================================
    // 8. 协方差更新 (Update Covariance)
    // ==========================================================
    // [Math] P = (I - K*H) * P
    // 更新 P 矩阵，表示我们通过测量，不确定性减小了
    Eigen::Matrix<float, DIM_ERR, DIM_ERR> I;
    I.setIdentity();

    // 使用 Joseph form 公式会更数值稳定，但这里用简易版 (I-KH)P 足够了
    P = (I - K * H) * P;

    // [注意] 我们不需要显式重置 delta_x，因为 delta_x 只是个局部变量。
    // 在下一次 predict 循环中，我们依然假设误差均值为 0。
    // 我们已经把这次算出来的 delta_x "吃"进 state.q 和 state.gyro_bias 里了。
}

// =========================================================================================
// 更新步：融合磁力计 (Fuse Magnetometer)
// 主要用于修正 Yaw (偏航角)
// =========================================================================================
// =========================================================================================
// 更新步：融合磁力计 (改进版：倾斜补偿航向融合)
// 解决 "磁力计干扰 Roll/Pitch" 和 "磁倾角导致平放不平" 的问题
// =========================================================================================
// =========================================================================================
// ES-EKF 更新步：融合磁力计 (Fuse Magnetometer)
// 核心策略：只修正 Yaw 轴，不干扰 Roll/Pitch
// =========================================================================================
void EspEKF::fuse_mag(const Vector3f& mag_meas)
{
    if (!is_initialized) return;

    // [Engineering] 异常保护
    // 磁场模长剧烈变化通常意味着有电机干扰或铁磁物体靠近
    float mag_norm = mag_meas.norm();
    if (mag_norm < 0.1f || mag_norm > 10.0f) return;

    // 1. 归一化测量值
    Vector3f m_body = mag_meas.normalized();

    // 2. 将机体坐标系下的磁场，旋转回世界坐标系 (Estimate World Mag)
    // 利用当前预测的姿态 state.q
    // [Math] m_world_pred = q * m_body * q^{-1}
    Vector3f m_world = state.q * m_body;

    // 3. 计算残差 (Yaw Error)
    // 在世界系下，磁场向量投影到水平面 (X-Y平面) 的角度，就是当前的实测航向
    // atan2(y, x) 计算的是 m_world 在世界系下的角度
    float yaw_measured = atan2(m_world.y(), m_world.x());

    // 我们期望的参考磁场方向应该是正北 (或者初始设定的任意 0 度方向)
    // 所以这里的 residual 就是 measured_yaw - 0
    // [Engineering] 这一步非常关键：我们通过旋转回世界系，巧妙地把 Roll/Pitch 的影响消除了
    float yaw_residual = yaw_measured;

    // 角度归一化 (限制在 -PI 到 +PI)
    while (yaw_residual > M_PI)  yaw_residual -= 2.0f * M_PI;
    while (yaw_residual < -M_PI) yaw_residual += 2.0f * M_PI;

    // 4. 构建雅可比矩阵 H (1x6)
    // 我们构建一个"虚拟观测"：z = yaw
    // 所以 H 矩阵只需要对应 Yaw 误差的状态，即 delta_theta_z (索引 2)
    // [Math] H = [0, 0, 1, 0, 0, 0]
    Eigen::Matrix<float, 1, DIM_ERR> H;
    H.setZero();
    H(0, 2) = 1.0f; // 强行指定：这个残差只与 Yaw 误差有关

    // 5. 计算卡尔曼增益 K
    // 此时观测维度是 1 (标量)

    // [Math] S = H P H^T + R
    // 提取 P(2,2) 即 Yaw 的方差
    float P_yaw = P(2, 2);
    float S = P_yaw + R_mag(0, 0); // R_mag 取第一个元素作为方差

    // [Math] K = P H^T S^{-1}
    // K 是一个 6x1 的向量 (6行1列)
    // P.col(2) 取出 P 的第3列 (对应 Yaw 误差与其他状态的协方差)
    Eigen::Matrix<float, DIM_ERR, 1> K = P.col(2) * (1.0f / S);

    // 6. 计算误差状态 delta_x
    // dx = K * y (向量 * 标量)
    Eigen::Matrix<float, DIM_ERR, 1> delta_x = K * yaw_residual;

    // ==========================================================
    // 7. 误差注入 (Error Injection) - 与 fuse_accel 完全一致
    // ==========================================================

    // A. 修正姿态
    Vector3f delta_theta = delta_x.head<3>();
    Eigen::Quaternionf dq;
    dq.w() = 1.0f;
    dq.vec() = 0.5f * delta_theta;

    state.q = state.q * dq;
    state.q.normalize();

    // B. 修正零偏
    state.gyro_bias += delta_x.tail<3>();

    // ==========================================================
    // 8. 协方差更新
    // ==========================================================
    Eigen::Matrix<float, DIM_ERR, DIM_ERR> I;
    I.setIdentity();
    P = (I - K * H) * P;
}
Eigen::Vector3f EspEKF::get_euler_angles()
{
    // 将四元数转换为欧拉角 (Z-Y-X 顺序: Yaw, Pitch, Roll)
    // Eigen 的 eulerAngles 返回的是 [0, 1, 2] 对应 [Yaw, Pitch, Roll]
    // 但为了稳健性，推荐使用手动计算公式

    // 提取四元数分量
    float qw = state.q.w();
    float qx = state.q.x();
    float qy = state.q.y();
    float qz = state.q.z();

    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    float roll = atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (qw * qy - qz * qx);
    float pitch;
    if (std::abs(sinp) >= 1.0f)
        pitch = std::copysign(M_PI / 2.0f, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    float yaw = atan2(siny_cosp, cosy_cosp);

    return Vector3f(roll, pitch, yaw);
}

Eigen::Vector3f EspEKF::get_gyro_bias()
{
    // 直接返回名义状态中的零偏
    return state.gyro_bias;
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