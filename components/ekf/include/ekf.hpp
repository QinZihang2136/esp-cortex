#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> // 必须引入 Geometry 才能用 Quaternion
#include "shared_types.h"

class EspEKF
{
public:
    // ==========================================
    // 1. 定义维度 (关键修改)
    // ==========================================
    // 注意：这是"误差状态"的维度 (Error State Dimension)
    // 包含: [Angle_Err_X, Angle_Err_Y, Angle_Err_Z, Bias_Err_X, Bias_Err_Y, Bias_Err_Z]
    static const int DIM_ERR = 6;

    // 观测维度
    static const int DIM_MEAS_ACC = 3;
    static const int DIM_MEAS_MAG = 3;

    // ==========================================
    // 2. 类型定义 (数学公式化)
    // ==========================================
    // 协方差矩阵 P 是 6x6 的 (对应误差状态)
    using MatrixP = Eigen::Matrix<float, DIM_ERR, DIM_ERR>;
    using MatrixQ = Eigen::Matrix<float, DIM_ERR, DIM_ERR>;

    // 观测矩阵与噪声
    using Matrix3f = Eigen::Matrix3f;
    using Vector3f = Eigen::Vector3f;
    // 误差状态向量 (只在计算过程中临时用到，但定义出来方便理解)
    using VectorErr = Eigen::Matrix<float, DIM_ERR, 1>;

    // ==========================================
    // 3. 核心数据结构：名义状态 (Nominal State)
    // ==========================================
    // 这就是我们的"真身"，它包含 7 个数
    struct NominalState
    {
        Eigen::Quaternionf q;      // 姿态 (4维)
        Vector3f gyro_bias;        // 零偏 (3维)

        // 构造函数：初始化为单位四元数和零偏
        NominalState()
        {
            q.setIdentity();       // w=1, x=0, y=0, z=0
            gyro_bias.setZero();
        }
    };

    // ==========================================
    // 4. Debug 信息结构体 (工程调试用)
    // ==========================================
    struct AccelDebug
    {
        bool used = false;
        float acc_norm = 0.0f;
        Vector3f z = Vector3f::Zero();        // 归一化后的测量加速度方向
        Vector3f g_pred = Vector3f::Zero();   // 预测重力方向(机体系)
        Vector3f innov = Vector3f::Zero();    // y = z - g_pred
        Vector3f dtheta = Vector3f::Zero();   // 注入的姿态小角
        Vector3f dbias = Vector3f::Zero();    // 注入的 bias 修正量
    };

    struct MagDebug
    {
        bool used = false;
        float mag_norm = 0.0f;
        float yaw_measured = 0.0f;
        float yaw_predicted = 0.0f;
        float yaw_residual = 0.0f;
        float yaw_residual_std = 0.0f;  // 残差标准差（稳定性检测）
        Vector3f m_world;       // 新增：世界坐标系的磁场向量
        Vector3f m_body;        // 新增：机体坐标系的磁场向量
        Vector3f dtheta = Vector3f::Zero();
        Vector3f dbias = Vector3f::Zero();
    };

    EspEKF();

    // --- 接口函数 (API) ---
    void init(const Vector3f& accel_meas);
    void predict(const Vector3f& gyro_meas, float dt);
    void fuse_accel(const Vector3f& accel);
    void fuse_mag(const Vector3f& mag);

    // --- Getter ---
    Vector3f get_euler_angles();
    Vector3f get_gyro_bias();
    float get_P_yaw() const { return P(2, 2); }
    float get_last_mag_k_yaw_bias_z() const { return last_mag_k_yaw_bias_z; }

    // Debug Getter
    AccelDebug get_last_accel_debug() const { return accel_dbg_; }
    MagDebug   get_last_mag_debug()   const { return mag_dbg_; }

    // --- Setter ---
    // 设置过程噪声 (Process Noise)
    void set_process_noise(float q_angle, float q_bias);
    // 设置测量噪声 (Measurement Noise)
    void set_measure_noise_accel(float r_accel);

private:
    // ==========================================
    // 5. 成员变量
    // ==========================================

    // 【核心变化】：不再用 VectorX x，改用结构体
    NominalState state;

    // 协方差矩阵 (6x6) - 描述误差的不确定性
    MatrixP P;

    // 噪声矩阵
    MatrixQ Q;               // 预测过程噪声
    Matrix3f R_accel;        // 加速度计测量噪声 (3x3)
    Matrix3f R_mag;          // 磁力计测量噪声 (3x3)

    // Debug 缓存
    AccelDebug accel_dbg_;
    MagDebug   mag_dbg_;

    bool is_initialized = false;
    bool mag_yaw_aligned = false;  // 磁力计航向是否已初始对齐

private:
    float last_mag_k_yaw_bias_z = 0.0f;  // 保存最后一次磁力计融合的卡尔曼增益
};
