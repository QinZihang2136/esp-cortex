#include "task_estimator.hpp"
#include "ekf.hpp"          // 引用数学核心
#include "robot_bus.hpp"    // 引用总线
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <cmath>

static const char* TAG = "ESTIMATOR";

// [修改 1] 定义单位转换常量
static const float DEG_TO_RAD = 0.0174532925f;
// static const float RAD_TO_DEG = 57.2957795f; // 如果后面不用可以直接乘 57.3

void task_estimator_entry(void* arg)
{
    // 1. 实例化数学核心
    EspEKF ekf;

    // [修改 2] 初始化状态修正
    // 在 FRD (前-右-下) 坐标系中，飞机平放静止时：
    // 重力加速度 g 指向地心 (+Z)，但加速度计测量的是比力 (Specific Force) / 支持力。
    // 支持力是向上的 (指向天)，在 FRD 坐标系中，"上"是 -Z 方向。
    // 所以平放时，加速度计理论读数应该是 [0, 0, -9.8]。
    // 之前你写 9.8 导致 EKF 认为当前是倒扣状态 (Roll ~ 180)。
    ekf.init(Eigen::Vector3f(0.0f, 0.0f, 9.81f));

    // 2. 注册总线监听
    RobotBus::instance().imu.register_notifier();

    ImuData imu_data;
    uint32_t last_imu_gen = 0;

    // 注意：这里初始化时间建议给个非0值，或者在循环里处理第一次 dt
    uint64_t last_time_us = esp_timer_get_time();

    MagData mag_data_snapshot;
    uint32_t last_mag_gen = 0;

    ESP_LOGI(TAG, "Estimator Task Started (FRD Fixed).");

    while (1)
    {
        // 3. 等待通知 (阻塞式，最长等待 100ms)
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        // 4. 尝试获取最新 IMU 数据
        if (RobotBus::instance().imu.copy_if_updated(imu_data, last_imu_gen))
        {
            // 计算 dt (秒)
            uint64_t now_us = imu_data.timestamp_us;

            // 防御性编程：防止时间倒流或第一次运行 dt 异常
            if (now_us <= last_time_us)
            {
                last_time_us = now_us;
                continue;
            }

            float dt = (now_us - last_time_us) / 1000000.0f;
            last_time_us = now_us;

            // 限制 dt 最大值，防止断点调试或掉帧后积分爆炸
            if (dt > 0.1f) dt = 0.005f; // 默认给一个 200Hz 的时间步长

            // ==========================================
            // [关键修改 3] 坐标系映射与单位转换
            // ==========================================
            // 目标: FRD (前-右-下)
            // 根据你的测试现象: 
            // 1. Roll (X) 方向正确 -> 保持 X
            // 2. Pitch (Y) 抬头变负 (反了) -> 取反 Y
            // 3. Yaw (Z) 顺时针变小 (反了) -> 取反 Z
            // 4. Acc Z 静止读出 +9.8 (导致反转) -> 取反 Z 变为 -9.8

            // --- A. 陀螺仪处理 (Deg/s -> Rad/s) ---
            Eigen::Vector3f gyro;
            // gyro.x() = imu_data.gx * DEG_TO_RAD;
            // gyro.y() = -imu_data.gy * DEG_TO_RAD; // [修正] 取反
            // gyro.z() = imu_data.gz * DEG_TO_RAD; // [修正] 取反
            gyro.x() = imu_data.gx;
            gyro.y() = imu_data.gy; // [修正] 取反
            gyro.z() = imu_data.gz; // [修正] 取反

            // 执行预测
            ekf.predict(gyro, dt);

            // --- B. 加速度计处理 (保持 m/s^2) ---
            // 必须与 Gyro 的旋转方向一致
            Eigen::Vector3f accel;
            accel.x() = -imu_data.ax;
            accel.y() = -imu_data.ay; // [修正] 取反
            accel.z() = -imu_data.az; // [修正] 取反, 解决 -170 度问题



            // 融合判断
            float acc_norm = accel.norm();
            // 门限: 0.5g ~ 1.5g (约 5.0 ~ 15.0 m/s^2)
            if (acc_norm > 5.0f && acc_norm < 15.0f)
            {
                ekf.fuse_accel(accel);
            }

            // --- C. 磁力计处理 ---
            if (RobotBus::instance().mag.copy_if_updated(mag_data_snapshot, last_mag_gen))
            {
                // if (mag_data_snapshot.x != 0.0f || mag_data_snapshot.y != 0.0f)
                // {
                //     // 磁力计同样需要映射! 假设安装方向与 IMU 一致
                //     Eigen::Vector3f mag_vec;
                //     mag_vec.x() = mag_data_snapshot.x;
                //     mag_vec.y() = -mag_data_snapshot.y; // [修正] 取反
                //     mag_vec.z() = -mag_data_snapshot.z; // [修正] 取反
                //     ekf.fuse_mag(mag_vec);
                // }
            }

            // --- D. 发布姿态 ---
            Eigen::Vector3f euler = ekf.get_euler_angles();

            AttitudeData att;
            // 弧度转角度 (rad -> deg)
            att.roll = euler.x() * 57.29578f;
            att.pitch = euler.y() * 57.29578f;
            att.yaw = euler.z() * 57.29578f;

            // 规范化 Yaw 到 0~360 方便观察
            // if (att.yaw < 0) att.yaw += 360.0f;
            // if (att.yaw >= 360.0f) att.yaw -= 360.0f;

            RobotBus::instance().attitude.publish(att);

            // [调试]
            static int print_div = 0;
            if (print_div++ > 200) // 1Hz
            {
                print_div = 0;
                // 打印姿态以及转换后的加速度值，方便确认是否平放时为 [0, 0, -9.8]
                ESP_LOGI(TAG, "R: %6.1f P: %6.1f Y: %6.1f | AccZ: %5.2f",
                    att.roll, att.pitch, att.yaw, accel.z());
            }
        }
    }
}

void start_estimator_task()
{
    // 优先级设为 4 (比 Sensor 高一点，或持平)
    // 栈大小设为 8192 (Eigen 库比较吃栈，给大点安全)
    xTaskCreatePinnedToCore(task_estimator_entry, "Estimator", 8192, NULL, 4, NULL, 1);
}