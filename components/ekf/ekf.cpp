#include "ekf.hpp" // 注意这里是你修改后的文件名
#include "esp_log.h"
#include <cmath>

static const char* TAG = "EKF";

// 构造函数
EspEKF::EspEKF()
{
    // 1. 初始化状态向量 x (全0)
    x.setZero();

    // 2. 初始化协方差矩阵 P (对角阵)
    P.setIdentity();
    P = P * 0.1f; // 初始不确定性不用太大

    // 3. 初始化噪声矩阵 Q (过程噪声)
    Q.setIdentity();
    // 默认给一个很小的值，防止 P 矩阵收敛到 0
    // 实际值应该通过 set_process_noise 接口设置
    Q.topLeftCorner(2, 2) *= 0.001f;     // 角度噪声
    Q.bottomRightCorner(2, 2) *= 0.0001f; // 零偏随机游走噪声

    // 4. 初始化观测噪声 R (暂时还没用到，先占位)
    R_accel.setIdentity();
    R_accel *= 0.1f;

    ESP_LOGI(TAG, "EKF Initialized. Eigen Version: %d.%d.%d",
        EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION);

    is_initialized = true;
}

void EspEKF::init()
{
    // 复位状态
    x.setZero();
    P.setIdentity();
    P = P * 0.1f; // 初始不确定性不用太大
    R_accel *= 0.1f;
    is_initialized = true;
}

// 核心：预测函数
void EspEKF::predict(const Vector3f& gyro, float dt)
{
    if (!is_initialized) return;

    // --- 1. 提取变量 (为了代码可读性) ---
    // x(0)=Roll, x(1)=Pitch, x(2)=BiasX, x(3)=BiasY
    float phi = x(0);
    float theta = x(1);
    float bgx = x(2);
    float bgy = x(3);

    float gx = gyro.x();
    float gy = gyro.y();
    // gz 暂时不用 (Yaw 不可观)

    // --- 2. 更新状态向量 x = f(x, u) ---
    // 简单的线性积分模型
    float new_phi = phi + (gx - bgx) * dt;
    float new_theta = theta + (gy - bgy) * dt;

    x(0) = new_phi;
    x(1) = new_theta;
    // Bias 保持不变 (随机游走模型: b_k = b_{k-1})

    // --- 3. 计算雅可比矩阵 F ---
    // F = I + (df/dx) * dt
    // 这里的 F 其实是 离散化后的状态转移矩阵 (Transition Matrix)

    Matrix<float, X_DIM, X_DIM> F;
    F.setIdentity(); // 先设为单位阵

    // 填充偏导数项
    // d(phi)/d(bgx) = -dt
    F(0, 2) = -dt;
    // d(theta)/d(bgy) = -dt
    F(1, 3) = -dt;

    // --- 4. 更新协方差矩阵 P = F*P*F' + Q ---
    P = F * P * F.transpose() + Q;
    P = 0.5f * (P + P.transpose());
}

// 后面这两个函数先留空，或者保持原样
void EspEKF::fuse_accel(const Vector3f& accel)
{
    if (!is_initialized) return;

    // 这里是融合加速度计的代码
    // 计算观测值和预测值的残差
    // 然后计算卡尔曼增益 K
    // 最后更新状态 x 和协方差 P
    if (accel.norm() < 0.1)
    {
        // 避免除以零
        return;
    }
    Vector3f z = accel.normalized(); // 实际观测值 (Measurement)

    // --- 2. 计算预期观测值 h(x) ---
    // 根据当前预测的姿态，算一下“理论上”加速度计应该读到多少
    float phi = x(0);
    float theta = x(1);
    float sp = sin(phi);   float cp = cos(phi);
    float st = sin(theta); float ct = cos(theta);

    Vector3f h_x;
    h_x(0) = -st;
    h_x(1) = sp * ct;
    h_x(2) = cp * ct;

    Vector3f y = z - h_x; // 残差 (Innovation)

    // --- 4. 计算雅可比矩阵 H ---
    // H 描述了状态(角度)微小变化如何影响观测(加速度)
    // 维度: 3(观测) x 4(状态)
    Matrix<float, 3, X_DIM> H;
    H.setZero(); // 先清零，因为后两列(Bias)对加速度没影响，保持为0即可

    // 第一行: d(acc_x) / d(phi, theta)
    H(0, 1) = -ct;

    // 第二行: d(acc_y) / d(phi, theta)
    H(1, 0) = cp * ct;
    H(1, 1) = -sp * st;

    // 第三行: d(acc_z) / d(phi, theta)
    H(2, 0) = -sp * ct;
    H(2, 1) = -cp * st;

    // --- 5. 标准 EKF 更新步骤 (The Magic) ---

    // S = H * P * Ht + R
    // S 是残差的协方差，代表“这次观测和预测之间的差异，有多大程度是不可信的噪声”
    Matrix<float, 3, 3> S = H * P * H.transpose() + R_accel;

    // K = P * Ht * S^-1
    // K 是卡尔曼增益，代表“我应该多大程度上相信这次残差”
    Matrix<float, X_DIM, 3> K = P * H.transpose() * S.inverse();

    // 更新状态向量: x = x + K * y
    x = x + K * y;

    // 更新协方差矩阵: P = (I - K * H) * P
    Matrix<float, X_DIM, X_DIM> I;
    I.setIdentity();
    P = (I - K * H) * P;

    // 由于篇幅限制，这里不展开具体实现
}

Vector3f EspEKF::get_euler_angles()
{
    return Vector3f(x(0), x(1), 0.0f);
}

Vector3f EspEKF::get_gyro_bias()
{
    return Vector3f(x(2), x(3), 0.0f);
}

// 参数设置接口实现
void EspEKF::set_process_noise(float q_angle, float q_bias)
{
    Q.setIdentity();
    Q.topLeftCorner(2, 2) *= q_angle;
    Q.bottomRightCorner(2, 2) *= q_bias;
}

void EspEKF::set_measure_noise_accel(float r_accel)
{
    R_accel.setIdentity();
    R_accel *= r_accel;
}