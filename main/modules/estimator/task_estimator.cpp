#include "task_estimator.hpp"
#include "ekf.hpp"          // 引用数学核心
#include "robot_bus.hpp"    // 引用总线
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

static const char* TAG = "ESTIMATOR";

void task_estimator_entry(void* arg)
{
    // 1. 实例化数学核心
    // 这里是在栈上创建的，没问题，因为 Task 的栈足够大且一直存在
    EspEKF ekf;
    ekf.init();

    // 2. 注册总线监听
    // 我们需要告诉 RobotBus: "当 IMU 更新时，请叫醒我"
    // 注意：前提是 Topic 类实现了 register_notifier，如果没有实现，我们可以先用简单的轮询或延时
    // 假设你在 Topic.hpp 里已经实现了 register_notifier (之前代码里有)
    RobotBus::instance().imu.register_notifier();

    ImuData imu_data;
    uint32_t last_imu_gen = 0;
    uint64_t last_time_us = esp_timer_get_time();

    ESP_LOGI(TAG, "Estimator Task Started.");

    while (1)
    {
        // 3. 等待通知 (阻塞式，最长等待 100ms)
        // 当 RobotBus::imu.publish() 被调用时，它会发信号过来
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        // 4. 尝试获取最新 IMU 数据
        if (RobotBus::instance().imu.copy_if_updated(imu_data, last_imu_gen))
        {

            // 计算 dt (秒)
            uint64_t now_us = imu_data.timestamp_us;
            // 防御性编程：防止 dt 异常（比如第一帧）
            if (now_us <= last_time_us)
            {
                last_time_us = now_us;
                continue;
            }
            float dt = (now_us - last_time_us) / 1000000.0f;
            last_time_us = now_us;

            // 限制 dt 最大值，防止数据中断后重连导致积分爆炸
            if (dt > 0.1f) dt = 0.01f;

            // --- A. 核心预测步骤 ---
            Vector3f gyro(imu_data.gx, imu_data.gy, imu_data.gz);
            ekf.predict(gyro, dt);

            // --- B. 发布姿态 ---
            Vector3f euler = ekf.get_euler_angles();

            AttitudeData att;
            att.roll = euler.x() * 57.29578f;  // 弧度转角度 (rad -> deg)
            att.pitch = euler.y() * 57.29578f; // 弧度转角度
            att.yaw = 0.0f; // 暂时没有

            RobotBus::instance().attitude.publish(att);

            // [调试] 每隔 1 秒打印一次，证明我们在工作
            static int print_div = 0;
            if (print_div++ > 200)
            { // 200Hz * 1s
                print_div = 0;
                // 打印 Roll, Pitch 和 协方差矩阵 P 的第一个元素(Roll方差)
                // 如果 P 在变大，说明预测在生效（不确定性增加）
                // ESP_LOGI(TAG, "Roll: %.2f, Pitch: %.2f", att.roll, att.pitch);
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