/**
 * @file debug_log.hpp
 * @brief 调试打印宏定义控制头文件
 *
 * 使用说明:
 * 1. 通过下方的宏定义开关控制各个模块的打印输出
 * 2. 在需要使用调试打印的文件中包含此头文件
 * 3. 使用 DEBUG_LOG_* 系列宏代替 ESP_LOGI
 */

#pragma once

#include "esp_log.h"

 // ============================================
 // 全局调试开关 (关闭后所有 DEBUG_LOG 都不输出)
 // ============================================
#define DEBUG_LOG_GLOBAL_ENABLE 1

// ============================================
// 各模块打印开关
// ============================================

// 传感器任务 (task_sensor.cpp)
#define DEBUG_SENSOR_ENABLE           0   // 传感器总开关
#define DEBUG_SENSOR_IMU              1   // IMU 数据打印
#define DEBUG_SENSOR_IMU_RAW          0   // IMU 原始数据打印
#define DEBUG_SENSOR_MAG              0   // 磁力计数据打印
#define DEBUG_SENSOR_BARO             0   // 气压计数据打印
#define DEBUG_SENSOR_I2C_SCAN         1   // I2C 扫描结果
#define DEBUG_SENSOR_CALIBRATION      1   // 校准参数加载
#define DEBUG_SENSOR_COORDINATE_MAP   0   // 坐标映射说明

// 姿态估计任务 (task_estimator.cpp)
#define DEBUG_ESTIMATOR_ENABLE        1   // 估计器总开关
#define DEBUG_ESTIMATOR_EULER         0   // 欧拉角输出
#define DEBUG_ESTIMATOR_GYRO          0   // 陀螺仪数据
#define DEBUG_ESTIMATOR_ACCEL         0   // 加速度计数据
#define DEBUG_ESTIMATOR_BIAS          0   // 陀螺仪零偏
#define DEBUG_ESTIMATOR_TIME          0   // 时间戳相关
#define DEBUG_ESTIMATOR_STATISTICS    0   // 统计计数器
#define DEBUG_ESTIMATOR_MAG           1   // 磁力计（即使未融合）
#define DEBUG_ESTIMATOR_SINGULARITY   0   // 欧拉角奇异性警告

// 打印频率控制 (每 N 次循环打印一次)
#define DEBUG_SENSOR_PRINT_INTERVAL   200  // 传感器任务打印间隔 (200次 = 1秒@200Hz)
#define DEBUG_ESTIMATOR_PRINT_INTERVAL_NORMAL   1000000  // 估计器正常打印间隔 (1Hz, 单位:us)
#define DEBUG_ESTIMATOR_PRINT_INTERVAL_FAST     200000  // 估计器快速打印间隔 (5Hz, 单位:us)

// ============================================
// 内部辅助宏 (不需要修改)
// ============================================

#define _DEBUG_LOG_PRINT(tag, fmt, ...) \
    do { \
        if (DEBUG_LOG_GLOBAL_ENABLE) { \
            ESP_LOGI(tag, fmt, ##__VA_ARGS__); \
        } \
    } while(0)

#define _DEBUG_LOG_CONDITION(condition, tag, fmt, ...) \
    do { \
        if (DEBUG_LOG_GLOBAL_ENABLE && (condition)) { \
            ESP_LOGI(tag, fmt, ##__VA_ARGS__); \
        } \
    } while(0)

// ============================================
// 传感器任务调试宏
// ============================================

#define DEBUG_SENSOR_INIT(tag) \
    static const char* _debug_tag = tag;

#define DEBUG_SENSOR_LOG(fmt, ...) \
    _DEBUG_LOG_CONDITION(DEBUG_SENSOR_ENABLE, _debug_tag, fmt, ##__VA_ARGS__)

#define DEBUG_SENSOR_IMU_LOG(fmt, ...) \
    _DEBUG_LOG_CONDITION((DEBUG_SENSOR_ENABLE && DEBUG_SENSOR_IMU), _debug_tag, fmt, ##__VA_ARGS__)

#define DEBUG_SENSOR_IMU_RAW_LOG(fmt, ...) \
    _DEBUG_LOG_CONDITION((DEBUG_SENSOR_ENABLE && DEBUG_SENSOR_IMU_RAW), _debug_tag, fmt, ##__VA_ARGS__)

#define DEBUG_SENSOR_MAG_LOG(fmt, ...) \
    _DEBUG_LOG_CONDITION((DEBUG_SENSOR_ENABLE && DEBUG_SENSOR_MAG), _debug_tag, fmt, ##__VA_ARGS__)

#define DEBUG_SENSOR_BARO_LOG(fmt, ...) \
    _DEBUG_LOG_CONDITION((DEBUG_SENSOR_ENABLE && DEBUG_SENSOR_BARO), _debug_tag, fmt, ##__VA_ARGS__)

#define DEBUG_SENSOR_I2C_SCAN_LOG(fmt, ...) \
    _DEBUG_LOG_CONDITION((DEBUG_SENSOR_ENABLE && DEBUG_SENSOR_I2C_SCAN), _debug_tag, fmt, ##__VA_ARGS__)

#define DEBUG_SENSOR_CALIBRATION_LOG(fmt, ...) \
    _DEBUG_LOG_CONDITION((DEBUG_SENSOR_ENABLE && DEBUG_SENSOR_CALIBRATION), _debug_tag, fmt, ##__VA_ARGS__)

#define DEBUG_SENSOR_COORDINATE_MAP_LOG(fmt, ...) \
    _DEBUG_LOG_CONDITION((DEBUG_SENSOR_ENABLE && DEBUG_SENSOR_COORDINATE_MAP), _debug_tag, fmt, ##__VA_ARGS__)

// ============================================
// 姿态估计任务调试宏
// ============================================

#define DEBUG_ESTIMATOR_INIT(tag) \
    static const char* _debug_tag = tag;

#define DEBUG_ESTIMATOR_LOG(fmt, ...) \
    _DEBUG_LOG_CONDITION(DEBUG_ESTIMATOR_ENABLE, _debug_tag, fmt, ##__VA_ARGS__)

#define DEBUG_ESTIMATOR_EULER_LOG(fmt, ...) \
    _DEBUG_LOG_CONDITION((DEBUG_ESTIMATOR_ENABLE && DEBUG_ESTIMATOR_EULER), _debug_tag, fmt, ##__VA_ARGS__)

#define DEBUG_ESTIMATOR_GYRO_LOG(fmt, ...) \
    _DEBUG_LOG_CONDITION((DEBUG_ESTIMATOR_ENABLE && DEBUG_ESTIMATOR_GYRO), _debug_tag, fmt, ##__VA_ARGS__)

#define DEBUG_ESTIMATOR_ACCEL_LOG(fmt, ...) \
    _DEBUG_LOG_CONDITION((DEBUG_ESTIMATOR_ENABLE && DEBUG_ESTIMATOR_ACCEL), _debug_tag, fmt, ##__VA_ARGS__)

#define DEBUG_ESTIMATOR_BIAS_LOG(fmt, ...) \
    _DEBUG_LOG_CONDITION((DEBUG_ESTIMATOR_ENABLE && DEBUG_ESTIMATOR_BIAS), _debug_tag, fmt, ##__VA_ARGS__)

#define DEBUG_ESTIMATOR_TIME_LOG(fmt, ...) \
    _DEBUG_LOG_CONDITION((DEBUG_ESTIMATOR_ENABLE && DEBUG_ESTIMATOR_TIME), _debug_tag, fmt, ##__VA_ARGS__)

#define DEBUG_ESTIMATOR_STATISTICS_LOG(fmt, ...) \
    _DEBUG_LOG_CONDITION((DEBUG_ESTIMATOR_ENABLE && DEBUG_ESTIMATOR_STATISTICS), _debug_tag, fmt, ##__VA_ARGS__)

#define DEBUG_ESTIMATOR_MAG_LOG(fmt, ...) \
    _DEBUG_LOG_CONDITION((DEBUG_ESTIMATOR_ENABLE && DEBUG_ESTIMATOR_MAG), _debug_tag, fmt, ##__VA_ARGS__)

#define DEBUG_ESTIMATOR_SINGULARITY_LOG(fmt, ...) \
    _DEBUG_LOG_CONDITION((DEBUG_ESTIMATOR_ENABLE && DEBUG_ESTIMATOR_SINGULARITY), _debug_tag, fmt, ##__VA_ARGS__)
