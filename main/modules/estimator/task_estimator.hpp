#pragma once

/**
 * @brief 启动状态估计任务 (EKF)
 * 职责：
 * 1. 订阅 IMU 数据 (200Hz)
 * 2. 执行 EKF 预测
 * 3. 融合 Accel/Mag (未来)
 * 4. 发布姿态数据 AttitudeData
 */
void start_estimator_task();