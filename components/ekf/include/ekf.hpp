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

    EspEKF();

    // --- 接口函数 (API) ---
    void init(const Vector3f& accel_meas);
    void predict(const Vector3f& gyro_meas, float dt);
    void fuse_accel(const Vector3f& accel);
    void fuse_mag(const Vector3f& mag);

    // --- Getter ---
    Vector3f get_euler_angles();
    Vector3f get_gyro_bias();

    // --- Setter ---
    // 设置过程噪声 (Process Noise)
    void set_process_noise(float q_angle, float q_bias);
    // 设置测量噪声 (Measurement Noise)
    void set_measure_noise_accel(float r_accel);

private:
    // ==========================================
    // 4. 成员变量
    // ==========================================

    // 【核心变化】：不再用 VectorX x，改用结构体
    NominalState state;

    // 协方差矩阵 (6x6) - 描述误差的不确定性
    MatrixP P;

    // 噪声矩阵
    MatrixQ Q;               // 预测过程噪声
    Matrix3f R_accel;        // 加速度计测量噪声 (3x3)
    Matrix3f R_mag;          // 磁力计测量噪声 (3x3)

    bool is_initialized = false;
};