#pragma once

#include<eigen3/Eigen/Dense>
#include "shared_types.h" // 引入你的 ImuData 等定义

// 使用 Eigen 命名空间，代码更简洁
using namespace Eigen;

class EspEKF
{
public:
    // ==========================================
    // 1. 定义维度 (方便未来扩展)
    // ==========================================
    // 当前阶段：状态量 = [Roll, Pitch, Gyro_Bias_X, Gyro_Bias_Y]
    static const int X_DIM = 4;
    // 观测维度：加速度计 [ax, ay, az] (虽然Z轴对Roll/Pitch贡献小，但通常还是3轴)
    static const int Z_DIM_ACC = 3;

    // 类型别名 (Type Aliases) - 让代码读起来像数学公式
    using VectorX = Matrix<float, X_DIM, 1>;      // 状态向量 x
    using MatrixP = Matrix<float, X_DIM, X_DIM>;  // 协方差矩阵 P
    using MatrixQ = Matrix<float, X_DIM, X_DIM>;  // 过程噪声矩阵 Q
    using MatrixR_Acc = Matrix<float, Z_DIM_ACC, Z_DIM_ACC>; // 测量噪声矩阵 R

    EspEKF();

    /**
     * @brief 初始化 EKF
     * 设置 P, Q, R 的初始值
     */
    void init();

    /**
     * @brief 预测步 (Prediction) - 对应公式 x = Fx + Bu
     * @param gyro  陀螺仪读数 (rad/s)
     * @param dt    距离上次预测的时间间隔 (s)
     */
    void predict(const Vector3f& gyro, float dt);

    /**
     * @brief 更新步 (Update) - 融合加速度计
     * @param accel 加速度计读数 (m/s^2 或 g)
     */
    void fuse_accel(const Vector3f& accel);

    /**
     * @brief 获取当前的欧拉角 (用于显示/控制)
     * @return Vector3f (Roll, Pitch, Yaw(暂时为0)) 单位: rad
     */
    Vector3f get_euler_angles();

    /**
     * @brief 获取估计的陀螺仪零偏 (用于调试)
     */
    Vector3f get_gyro_bias();

    // ==========================================
    // 参数调整接口 (供 Task 层调用)
    // ==========================================
    void set_process_noise(float q_angle, float q_bias);
    void set_measure_noise_accel(float r_accel);

private:
    // --- 核心矩阵 ---
    VectorX x; // 状态向量 [phi, theta, bgx, bgy]
    MatrixP P; // 协方差矩阵

    // --- 噪声矩阵 ---
    MatrixQ Q;
    MatrixR_Acc R_accel;

    // --- 辅助变量 ---
    bool is_initialized = false;
};