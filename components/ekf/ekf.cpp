#include "ekf.hpp"
#include "esp_log.h"
#include "esp_timer.h"
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
    P = P * 0.01f; // 初始协方差设小一点，防止启动瞬间震荡

    // 3. 初始化过程噪声矩阵 Q
    // [Math] Q 代表系统模型预测的不确定性 (Predict Step Noise)。
    // 即使没有外部干扰，积分过程也会累积误差。
    Q.setIdentity();
    // [Engineering] 这里的参数通常需要根据实际传感器噪声 spec 或实际飞行 log 调节。
    // 如果发现静止时姿态漂移大，减小 bias noise；如果反应迟钝，增大 angle noise。
    Q.topLeftCorner(3, 3) *= 0.0005f;      // 角度积分噪声 (rad/s)^2 * dt
    Q.bottomRightCorner(3, 3) *= 0.00001f; // 零偏随机游走 (Bias instability)

    // 4. 初始化观测噪声矩阵 R
    // [Math] R 代表传感器测量的不确定性 (Update Step Noise)。
    // 值越大，EKF 越不相信这个传感器，收敛越慢；值越小，越容易受震动干扰。
    R_accel.setIdentity();
    R_accel *= 0.05f; // 加速度计噪声 (如果震动大，把这个值调大到 0.1 ~ 0.5)

    R_mag.setIdentity();
    R_mag *= 0.1f;   // 磁力计噪声 (环境磁干扰大时应调大此值)

    is_initialized = false;

    ESP_LOGI(TAG, "EKF Initialized. Eigen Version: %d.%d.%d",
        EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION);
}

void EspEKF::init(const Vector3f& accel_meas)
{
    // 1. 归一化加速度 (只关心方向，不关心大小)
    Vector3f acc_norm = accel_meas.normalized();

    // 2. 计算初始欧拉角 (根据重力分量反推)
    // 注意：这里假设静止，所以加速度计测到的仅仅是重力方向
    // Pitch: 绕Y轴旋转
    float pitch = atan2f(-acc_norm.x(), sqrtf(acc_norm.y() * acc_norm.y() + acc_norm.z() * acc_norm.z()));
    // Roll: 绕X轴旋转
    float roll = atan2f(acc_norm.y(), acc_norm.z());
    float yaw = 0.0f; // 暂时假设 Yaw 为 0 (或者以后接磁力计初始化)

    // 从欧拉角构建初始四元数
    // 顺序：Z * Y * X (这是航空标准的旋转顺序 3-2-1)
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

    state.q = yawAngle * pitchAngle * rollAngle;
    state.q.normalize(); // 养成好习惯

    // 初始化零偏为0 (或者可以进行一次静态平均)
    state.gyro_bias.setZero();

    is_initialized = true;
    ESP_LOGI(TAG, "EKF initialized with Roll: %.2f deg, Pitch: %.2f deg", roll * 57.295f, pitch * 57.295f);
}

// =========================================================================================
// 核心：预测函数 (Predict Step)
// [Math] x_{k|k-1} = f(x_{k-1}, u_k)
// [Math] P_{k|k-1} = F_k * P_{k-1} * F_k^T + Q_k
// =========================================================================================
void EspEKF::predict(const Vector3f& gyro_meas, float dt)
{
    if (!is_initialized) return;

    // 1. 去除零偏的角速度
    Vector3f omega = gyro_meas - state.gyro_bias;
    float omega_norm = omega.norm();

    // 2. 名义状态更新 (四元数积分)
    // 使用 0 阶积分 (Zeroth Order Integration)
    Eigen::Quaternionf dq;
    if (omega_norm > 1e-5f)
    {
        // 轴角公式: [cos(theta/2), sin(theta/2) * v]
        float theta_half = omega_norm * dt * 0.5f;
        float sin_theta_half = std::sin(theta_half);
        dq.w() = std::cos(theta_half);
        dq.vec() = (omega / omega_norm) * sin_theta_half;
    }
    else
    {
        // 小角度近似 (避免除以0)
        dq.w() = 1.0f;
        dq.vec() = omega * dt * 0.5f;
    }
    state.q = state.q * dq;
    state.q.normalize(); // [重要] 必须归一化，否则数值漂移会导致 NaN

    // 3. 误差状态协方差更新 P = F P F^T + Q
    // Fx 矩阵定义 (6x6)
    // [ I - [w*dt]x   -I * dt ]
    // [      0           I    ]
    Eigen::Matrix<float, DIM_ERR, DIM_ERR> Fx;
    Fx.setIdentity();

    // 计算旋转部分的反对称矩阵 (Skew symmetric matrix)
    // [  0 -wz  wy ]
    // [ wz   0 -wx ]
    // [-wy  wx   0 ]
    Eigen::Matrix3f omega_skew;
    omega_skew << 0, -omega.z(), omega.y(),
        omega.z(), 0, -omega.x(),
        -omega.y(), omega.x(), 0;

    // 填充 Fx (关键修复点：关联角度误差和零偏误差)
    // 角度误差传递: I - [w]x * dt  (这里的负号取决于误差定义，通常为 I - [w]x * dt)
    Fx.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() - omega_skew * dt;
    // 零偏对角度的影响: -I * dt (零偏会导致反向的角度漂移)
    Fx.block<3, 3>(0, 3) = -Eigen::Matrix3f::Identity() * dt;

    // 更新 P
    P = Fx * P * Fx.transpose() + Q;

    // [工程技巧] 强制 P 对称，防止数值发散导致"静止波动"
    P = 0.5f * (P + P.transpose());
}

// =========================================================================================
// 更新步：融合加速度计 (Fuse Accelerometer)
// 主要用于修正 Roll 和 Pitch
// =========================================================================================
void EspEKF::fuse_accel(const Vector3f& accel_meas)
{
    if (!is_initialized) return;

    // 清空 debug
    accel_dbg_ = AccelDebug();

    // [Engineering] 异常保护
    // 只有当加速度模长接近 1g (9.8 m/s^2 或 1.0g) 时，我们才认为它是重力。
    // 如果模长太大（震动/转弯离心力）或太小（自由落体），说明加速度计测的不是纯重力，不能用来修姿态。
    float acc_norm = accel_meas.norm();
    accel_dbg_.acc_norm = acc_norm;

    // 假设输入单位是 m/s^2，正常应该在 9.8 附近。放宽到 0.9g ~ 1.1g 范围
    if (acc_norm < 8.8f || acc_norm > 10.8f)
    {
        accel_dbg_.used = false;
        return; // 放弃这次更新，只相信陀螺仪积分
    }

    // 1. 准备测量值 (z)
    // 归一化，因为我们只关心重力的"方向"来修正姿态
    Vector3f z = accel_meas.normalized();

    // 2. 准备预测值 h(x)
    // 利用当前名义姿态 state.q，把世界系重力 [0,0,1] 旋转到机体坐标系
    // [Math] g_pred = R(q)^T * [0,0,1] (世界->机体)
    Vector3f g_world = Vector3f::UnitZ();
    Vector3f g_pred = state.q.inverse() * g_world;

    // 3. 计算残差 (Innovation)
    // [Math] y = z - h(x)
    Vector3f y = z - g_pred;

    // debug 记录
    accel_dbg_.used = true;
    accel_dbg_.z = z;
    accel_dbg_.g_pred = g_pred;
    accel_dbg_.innov = y;

    // 4. 构建观测雅可比矩阵 H (Measurement Jacobian)
    // 对于重力观测，H 矩阵关联误差状态到观测残差
    // [Math] H = [ [g_pred]x   0 ]  (维度 3x6)
    // 左边是 g_pred 的反对称矩阵，右边是 0 (因为加速度计无法直接观测陀螺仪零偏)
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
    Eigen::Matrix3f S = H * P * H.transpose() + R_accel;

    // [Math] K = P * H^T * S^-1
    // 这里的 S.inverse() 对 3x3 矩阵很快，不用担心性能
    Eigen::Matrix<float, DIM_ERR, 3> K = P * H.transpose() * S.inverse();

    // =====================================================================================
    // [关键工程约束]
    // 1) 加速度计只用于修正 Roll/Pitch，Yaw 不可观测：禁止更新 dtheta_z
    // 2) 同理，Yaw 轴 gyro bias 也不可由 accel “可靠”观测：禁止更新 dbias_z
    //    否则会出现你日志里那种 bias_z 变到 -0.1 rad/s 量级，导致 yaw 越跑越快/趋势怪。
    // =====================================================================================
    K.row(2).setZero(); // 禁止 accel 更新 yaw 误差
    K.row(5).setZero(); // 禁止 accel 更新 z 轴 gyro bias

    // 6. 计算误差状态 delta_x (Error State)
    // [Math] dx = K * y
    // delta_x 是 6维向量: [d_roll, d_pitch, d_yaw, d_bgx, d_bgy, d_bgz]
    Eigen::Matrix<float, DIM_ERR, 1> delta_x = K * y;

    // ==========================================================
    // 7. 误差注入 (Error Injection) - ES-EKF 的灵魂
    // ==========================================================

    // A. 修正姿态 (名义四元数)
    Vector3f delta_theta = delta_x.head<3>();

    // [Engineering] 限幅保护：防止异常瞬间把姿态“打飞”
    // 正常情况下 delta_theta 应该很小（毫弧度~几毫弧度）
    const float dtheta_max = 0.20f; // rad，约 11.5deg（已经很宽了）
    if (delta_theta.norm() > dtheta_max)
    {
        delta_theta = delta_theta.normalized() * dtheta_max;
    }

    // 把角度误差向量转换成微小旋转四元数
    // 使用小角度近似: dq = [1, 0.5*dx, 0.5*dy, 0.5*dz]
    Eigen::Quaternionf dq;
    dq.w() = 1.0f;
    dq.vec() = 0.5f * delta_theta;

    // 注入误差：q_new = q_old * dq (注意乘法顺序)
    state.q = state.q * dq;
    state.q.normalize(); // 必须归一化

    // B. 修正零偏 (名义零偏)
    // 取出后 3 维 (零偏误差) 并直接加到当前零偏上
    Vector3f dbias = delta_x.tail<3>();

    // [Engineering] 限幅保护：避免 bias 被一次更新拉飞
    const float dbias_max = 0.05f; // rad/s
    if (dbias.norm() > dbias_max)
    {
        dbias = dbias.normalized() * dbias_max;
    }

    state.gyro_bias += dbias;

    // debug 记录（注入后的）
    accel_dbg_.dtheta = delta_theta;
    accel_dbg_.dbias = dbias;

    // ==========================================================
    // 8. 协方差更新 (Update Covariance) - Joseph form 更稳健
    // ==========================================================
    // [Math] P = (I - K*H) * P * (I - K*H)^T + K*R*K^T
    Eigen::Matrix<float, DIM_ERR, DIM_ERR> I = Eigen::Matrix<float, DIM_ERR, DIM_ERR>::Identity();
    Eigen::Matrix<float, DIM_ERR, DIM_ERR> IKH = (I - K * H);
    P = IKH * P * IKH.transpose() + K * R_accel * K.transpose();

    // [工程技巧] 再次强制对称，防止更新步骤引入非对称性
    P = 0.5f * (P + P.transpose());
}

// =========================================================================================
// ES-EKF 更新步：融合磁力计 (Fuse Magnetometer)
// 核心策略：只修正 Yaw 轴，不干扰 Roll/Pitch
// 解决 "磁力计干扰 Roll/Pitch" 和 "磁倾角导致平放不平" 的问题
// =========================================================================================
void EspEKF::fuse_mag(const Vector3f& mag_meas)
{
    if (!is_initialized) return;

    // 清空 debug
    mag_dbg_ = MagDebug();

    // [Engineering] 异常保护
    // 磁场模长剧烈变化通常意味着有电机干扰或铁磁物体靠近
    float mag_norm = mag_meas.norm();
    mag_dbg_.mag_norm = mag_norm;

    // 假设归一化后的磁场大约是 1.0 (如果未标定，可能是几十uT)
    // 这里使用较宽的阈值，仅排除极端异常值
    if (mag_norm < 0.01f || mag_norm > 1000.0f)
    {
        mag_dbg_.used = false;
        return;
    }

    // 1. 归一化测量值
    Vector3f m_body = mag_meas.normalized();

    // 2. 倾斜补偿航向计算
    // [问题] 不能直接用 m_body 计算航向（设备倾斜时不准确）
    // [问题] 不能用 state.q * m_body（因为 state.q 包含 Yaw，会导致错误旋转）
    // [解决] 创建只包含 Roll 和 Pitch 的旋转矩阵，把 m_body 旋转到水平面

    // 2.1 从当前姿态四元数提取 Roll 和 Pitch
    float qw = state.q.w();
    float qx = state.q.x();
    float qy = state.q.y();
    float qz = state.q.z();

    // Roll (φ): 绕 X 轴旋转
    float roll = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));

    // Pitch (θ): 绕 Y 轴旋转
    float pitch = asinf(2.0f * (qw * qy - qz * qx));

    // 2.2 构建只包含 Roll 和 Pitch 的旋转矩阵
    float cos_roll = cosf(roll);
    float sin_roll = sinf(roll);
    float cos_pitch = cosf(pitch);
    float sin_pitch = sinf(pitch);

    // R_rp = R_pitch * R_roll (FRD 坐标系)
    // 先绕 X 轴旋转 Roll，再绕 Y 轴旋转 Pitch
    Matrix3f R_rp;
    R_rp << cos_pitch,  sin_roll * sin_pitch,  cos_roll * sin_pitch,
            0,          cos_roll,             -sin_roll,
            -sin_pitch, sin_roll * cos_pitch,  cos_roll * cos_pitch;

    // 2.3 用 R_rp 把 m_body 旋转到水平面
    Vector3f m_horizontal = R_rp * m_body;

    // 2.4 在水平面上计算航向（投影到 XY 平面）
    // [修复] 取反以匹配 IMU 航向方向（FRD 坐标系右手定则）
    float yaw_measured = -atan2f(m_horizontal.y(), m_horizontal.x());

    // 3. (用于调试对比) 原来的错误方法
    Vector3f m_world = state.q * m_body;

    // 4. 从当前姿态四元数提取预测的 yaw 角度
    float yaw_predicted = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));

    // [关键修复] 残差 = 测量值 - 预测值（而不是测量值 - 0）
    float yaw_residual = normalize_angle(yaw_measured - yaw_predicted);

    // [调试模式] 只计算磁力计航向，不更新 EKF 状态
    // 用于观察陀螺仪积分航向 vs 磁力计航向的差异
    const bool enable_mag_debug_only = false;  // 启用正常融合

    // 保存 debug 数据（即使不融合也保存）
    mag_dbg_.used = false;  // 标记为未实际融合
    mag_dbg_.yaw_measured = yaw_measured;
    mag_dbg_.yaw_predicted = yaw_predicted;
    mag_dbg_.yaw_residual = yaw_residual;
    mag_dbg_.m_world = m_world;
    mag_dbg_.m_body = m_body;

    // 如果是调试模式，只保存数据后直接返回
    if (enable_mag_debug_only) {
        return;
    }

    // 以下是正常融合流程（调试模式下不执行）

    // =====================================================================================
    // [初始对齐] 首次磁力计融合：强制将 IMU 航向对齐到磁力计航向
    // =====================================================================================
    if (!mag_yaw_aligned)
    {
        // 首次融合：强制修正 yaw，不通过卡尔曼滤波
        ESP_LOGI("EKF", "[MAG_INIT] Initial yaw alignment: yaw_measured=%.1f°, yaw_predicted=%.1f°, residual=%.1f°",
                 yaw_measured * 57.29578f,
                 yaw_predicted * 57.29578f,
                 yaw_residual * 57.29578f);

        // 提取当前的 roll 和 pitch（保持不变）
        float roll = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));
        float pitch = asinf(2.0f * (qw * qy - qz * qx));

        // 使用磁力计的 yaw
        float yaw = yaw_measured;

        // 从欧拉角重建四元数 (ZYX 顺序，即 yaw -> pitch -> roll)
        float cy = cosf(yaw * 0.5f);
        float sy = sinf(yaw * 0.5f);
        float cp = cosf(pitch * 0.5f);
        float sp = sinf(pitch * 0.5f);
        float cr = cosf(roll * 0.5f);
        float sr = sinf(roll * 0.5f);

        state.q.w() = cr * cp * cy + sr * sp * sy;
        state.q.x() = sr * cp * cy - cr * sp * sy;
        state.q.y() = cr * sp * cy + sr * cp * sy;
        state.q.z() = cr * cp * sy - sr * sp * cy;

        mag_yaw_aligned = true;  // 标记已完成对齐

        mag_dbg_.used = true;
        mag_dbg_.dtheta = Vector3f(0, 0, yaw_residual);  // 记录修正量
        mag_dbg_.dbias = Vector3f::Zero();

        ESP_LOGI("EKF", "[MAG_INIT] Yaw aligned to magnetometer: %.1f°", yaw * 57.29578f);
        return;  // 对齐后直接返回，等待下一次融合
    }

    // [残差稳定性检测] - 参考 PX4 创新门限，但增加稳定性判断
    // 区分两种情况：
    // 1. 残差波动大（方差大）→ 磁力计受干扰 → 拒绝
    // 2. 残差稳定但大（方差小，均值大）→ 系统性偏置 → 允许融合修正

    // 计算最近 N 次残差的统计特性
    const int window_size = 20;  // 20 个样本 @ 50Hz = 0.4 秒
    static float residual_buffer[20] = {0};
    static int residual_index = 0;
    static int residual_count = 0;

    // 存储当前残差
    residual_buffer[residual_index] = yaw_residual;
    residual_index = (residual_index + 1) % window_size;
    if (residual_count < window_size) residual_count++;

    // 基础门限（正常情况）
    float residual_max = 0.175f;  // 10°
    float residual_abs = std::fabs(yaw_residual);

    // 当样本足够时，计算统计特性
    if (residual_count >= window_size) {
        // 计算均值
        float mean = 0.0f;
        for (int i = 0; i < window_size; i++) {
            mean += residual_buffer[i];
        }
        mean /= window_size;

        // 计算方差（标准差的平方）
        float variance = 0.0f;
        for (int i = 0; i < window_size; i++) {
            float diff = residual_buffer[i] - mean;
            variance += diff * diff;
        }
        variance /= window_size;
        float std_dev = sqrtf(variance);

        // 稳定性判断：如果标准差 < 5°，说明残差稳定
        const float stable_threshold = 0.087f;  // 5°

        if (std_dev < stable_threshold) {
            // 残差稳定，但均值大 → 系统性偏置，放宽门限到 60°
            residual_max = 1.047f;  // 60° (约1.047弧度)

            // 首次检测到稳定偏置时打印日志
            static bool was_stable = false;
            if (!was_stable) {
                ESP_LOGW("EKF", "[MAG_STABLE] Residual stable: mean=%.1f°, std=%.1f°, relaxing gate to 60°",
                         mean * 57.29578f, std_dev * 57.29578f);
                was_stable = true;
            }
        } else {
            // 残差不稳定，说明受干扰，保持严格门限
            static bool was_unstable = false;
            if (!was_unstable && residual_count >= window_size) {
                ESP_LOGW("EKF", "[MAG_UNSTABLE] Residual fluctuating: mean=%.1f°, std=%.1f°, keeping strict gate 10°",
                         mean * 57.29578f, std_dev * 57.29578f);
                was_unstable = true;
            }
        }

        // 保存统计信息到 debug（用于网页显示）
        mag_dbg_.yaw_residual_std = std_dev;
    } else {
        mag_dbg_.yaw_residual_std = 0.0f;
    }

    // [诊断] 打印（已注释，避免影响网页显示）
    /*
    static int mag_fuse_count = 0;
    static uint64_t last_print_us = 0;
    uint64_t now_us = esp_timer_get_time();

    if (mag_fuse_count < 100) {
        ESP_LOGW("EKF", "[MAG_DEBUG] m_body: x=%.3f y=%.3f z=%.3f | "
                      "m_world: x=%.3f y=%.3f z=%.3f | "
                      "yaw_meas=%.1f° yaw_pred=%.1f° | residual=%.1f°",
                 m_body.x(), m_body.y(), m_body.z(),
                 m_world.x(), m_world.y(), m_world.z(),
                 yaw_measured * 57.29578f,
                 yaw_predicted * 57.29578f,
                 yaw_residual * 57.29578f);
        mag_fuse_count++;
        last_print_us = now_us;
    }
    else if ((now_us - last_print_us) > 5000000) {
        ESP_LOGI("EKF", "[MAG_STATUS] yaw_meas=%.1f° yaw_pred=%.1f° | residual=%.1f°",
                 yaw_measured * 57.29578f,
                 yaw_predicted * 57.29578f,
                 yaw_residual * 57.29578f);
        last_print_us = now_us;
    }
    */

    if (residual_abs > residual_max) {
        // 残差过大，拒绝本次磁力计融合
        ESP_LOGW("EKF", "[MAG_REJECT] Residual too large: %.1f° (max: %.1f°), std: %.1f°",
                 yaw_residual * 57.29578f, residual_max * 57.29578f, mag_dbg_.yaw_residual_std * 57.29578f);

        mag_dbg_.used = false;
        mag_dbg_.yaw_measured = yaw_measured;
        mag_dbg_.yaw_predicted = yaw_predicted;
        mag_dbg_.yaw_residual = yaw_residual;
        mag_dbg_.m_world = m_world;
        mag_dbg_.m_body = m_body;
        return;  // 直接返回，不更新状态和协方差
    }

    // 残差正常，接受融合
    // debug
    mag_dbg_.used = true;
    mag_dbg_.yaw_measured = yaw_measured;
    mag_dbg_.yaw_predicted = yaw_predicted;
    mag_dbg_.yaw_residual = yaw_residual;
    mag_dbg_.m_world = m_world;  // 保存世界坐标磁场向量
    mag_dbg_.m_body = m_body;    // 保存机体坐标磁场向量

    // 4. 构建雅可比矩阵 H (1x6)
    // [Math] H = [0, 0, 1, 0, 0, 0]
    Eigen::Matrix<float, 1, DIM_ERR> H;
    H.setZero();
    H(0, 2) = 1.0f;

    // 5. 计算卡尔曼增益 K
    float P_yaw = P(2, 2);
    float S = P_yaw + R_mag(0, 0);

    Eigen::Matrix<float, DIM_ERR, 1> K = P.col(2) * (1.0f / S);

    // =====================================================================================
    // [关键工程约束]
    // 你注释写的是“只修正 Yaw 轴，不干扰 Roll/Pitch”
    // 那就必须强制 K 只保留 yaw 和（可选）bias_z，其它通道清零，避免协方差耦合污染。
    // =====================================================================================
    K(0) = 0.0f; // roll
    K(1) = 0.0f; // pitch
    K(3) = 0.0f; // bias_x
    K(4) = 0.0f; // bias_y
    // K(2) yaw 保留
    // K(5) bias_z 保留（磁力计可以帮助约束 yaw 漂移，间接抑制 z-bias）

    // 保存卡尔曼增益（用于调试）
    last_mag_k_yaw_bias_z = K(5);

    // 6. 计算误差状态 delta_x
    Eigen::Matrix<float, DIM_ERR, 1> delta_x = K * yaw_residual;

    // A. 修正姿态
    Vector3f delta_theta = delta_x.head<3>();

    Eigen::Quaternionf dq;
    dq.w() = 1.0f;
    dq.vec() = 0.5f * delta_theta;

    state.q = state.q * dq;
    state.q.normalize();

    // B. 修正零偏
    Vector3f dbias = delta_x.tail<3>();
    state.gyro_bias += dbias;

    // [紧急修复] 限制零偏范围，防止发散
    // 正常的陀螺仪零偏应该在 ±0.05 rad/s 范围内
    // 原来设置为 0.1 太大，导致零偏被钳制在边界值
    const float max_bias = 0.05f;  // rad/s (约 2.9°/s)
    state.gyro_bias = state.gyro_bias.cwiseMax(Vector3f(-max_bias, -max_bias, -max_bias));
    state.gyro_bias = state.gyro_bias.cwiseMin(Vector3f(max_bias, max_bias, max_bias));

    // debug
    mag_dbg_.dtheta = delta_theta;
    mag_dbg_.dbias = dbias;

    // ==========================================================
    // 8. 协方差更新 (Joseph form)
    // ==========================================================
    Eigen::Matrix<float, DIM_ERR, DIM_ERR> I = Eigen::Matrix<float, DIM_ERR, DIM_ERR>::Identity();
    Eigen::Matrix<float, DIM_ERR, DIM_ERR> IKH = (I - K * H);
    P = IKH * P * IKH.transpose() + (K * R_mag(0, 0)) * K.transpose();

    P = 0.5f * (P + P.transpose());
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
    float roll = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (qw * qy - qz * qx);
    float pitch;
    if (std::abs(sinp) >= 1.0f)
        pitch = std::copysign(M_PI / 2.0f, sinp); // use 90 degrees if out of range
    else
        pitch = asinf(sinp);

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    float yaw = atan2f(siny_cosp, cosy_cosp);

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
