#include "task_estimator.hpp"
#include "ekf.hpp"          // 引用数学核心
#include "robot_bus.hpp"    // 引用总线
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <cmath>

static const char* TAG = "ESTIMATOR";

// 角度单位换算（仅用于输出）
// 注意：当前 imu_data.gx/gy/gz 已经是 rad/s（你已取消再次转弧度），不要再乘 DEG_TO_RAD。
static const float RAD_TO_DEG = 57.29578f;

// =======================
// [新增] 调试辅助：角度 wrap / 差分
// =======================
static inline float wrap_deg(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static inline float delta_deg(float now, float prev)
{
    return wrap_deg(now - prev);
}

void task_estimator_entry(void* arg)
{
    // 1. 实例化数学核心
    EspEKF ekf;

    // [初始化说明 - 已修订]
    // 你的 SensorTask 已经把 IMU 数据映射到 FRD。
    // 但在本文件里，你对 accel 又做了取反（见下方 accel = -imu_data.a*），
    // 因此“送入 EKF fuse_accel() 的 accel”在平放静止时会接近 [0, 0, +9.8]。
    // 为了与当前这一约定一致，这里使用 +9.81 初始化。
    //
    // 如果你未来决定“不在这里取反 accel”（即直接用 FRD 下的原始比力，平放应为 [0,0,-9.8]），
    // 那么这里也应改为 ekf.init(Eigen::Vector3f(0.0f, 0.0f, -9.81f));
    ekf.init(Eigen::Vector3f(0.0f, 0.0f, 9.81f));

    // 2. 注册总线监听
    RobotBus::instance().imu.register_notifier();

    ImuData imu_data;
    uint32_t last_imu_gen = 0;

    // 注意：这里初始化时间建议给个非0值，或者在循环里处理第一次 dt
    uint64_t last_time_us = esp_timer_get_time();

    MagData mag_data_snapshot;
    uint32_t last_mag_gen = 0;

    ESP_LOGI(TAG, "Estimator Task Started (FRD).");

    // =======================
    // [新增] Debug 状态缓存
    // =======================
    static bool dbg_inited = false;
    static float last_yaw_deg = 0.0f;
    static uint64_t last_yaw_us = 0;

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
            // 坐标系说明
            // ==========================================
            // SensorTask 输出的 imu_data 已是 FRD（前-右-下）：
            //  - Gyro/Accel 的符号已按 FRD 右手系映射
            //  - 单位：gyro rad/s，accel m/s^2

            // --- A. 陀螺仪处理 (rad/s) ---
            Eigen::Vector3f gyro;
            gyro.x() = imu_data.gx;
            gyro.y() = imu_data.gy;
            gyro.z() = imu_data.gz;

            // 执行预测
            ekf.predict(gyro, dt);

            // --- B. 加速度计处理 (m/s^2) ---
            // 注意：这里目前是“整体取反”，目的是让平放时 accel.z ≈ +9.8，以匹配 EKF 内部当前重力方向约定。
            // 若你未来把 EKF 的重力方向改为与 FRD 比力一致（平放 -9.8），可取消这里的取反，并同步调整 init()。
            Eigen::Vector3f accel;
            accel.x() = -imu_data.ax;
            accel.y() = -imu_data.ay;
            accel.z() = -imu_data.az;

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
                // 目前你还没融合磁力计，因此 yaw 会慢漂是正常现象。
                // 如果要锁定航向，需要启用 fuse_mag（并处理软硬铁标定与倾角补偿策略）。
                //
                // if (mag_data_snapshot.x != 0.0f || mag_data_snapshot.y != 0.0f || mag_data_snapshot.z != 0.0f)
                // {
                //     Eigen::Vector3f mag_vec;
                //     // SensorTask 已做 FRD 映射，这里通常不需要再改符号
                //     mag_vec.x() = mag_data_snapshot.x;
                //     mag_vec.y() = mag_data_snapshot.y;
                //     mag_vec.z() = mag_data_snapshot.z;
                //     ekf.fuse_mag(mag_vec);
                // }
            }

            // --- D. 发布姿态 ---
            Eigen::Vector3f euler = ekf.get_euler_angles();

            AttitudeData att;
            // 弧度转角度 (rad -> deg)
            att.roll = euler.x() * RAD_TO_DEG;
            att.pitch = euler.y() * RAD_TO_DEG;
            att.yaw = euler.z() * RAD_TO_DEG;

            // 建议保持 yaw 在 [-180, 180]（便于差分和观察 wrap）
            att.yaw = wrap_deg(att.yaw);

            RobotBus::instance().attitude.publish(att);

            // ==========================================================
            // [新增] 高判定力 Debug 输出
            // ==========================================================
            // 1) 计算 yaw 角速度（差分后做 wrap，避免跨 -180/180 产生假尖峰）
            float yaw_rate_deg_s = 0.0f;
            if (!dbg_inited)
            {
                dbg_inited = true;
                last_yaw_deg = att.yaw;
                last_yaw_us = now_us;
            }
            else
            {
                float dt_yaw = (now_us - last_yaw_us) * 1e-6f;
                if (dt_yaw > 1e-4f && dt_yaw < 0.2f)
                {
                    yaw_rate_deg_s = delta_deg(att.yaw, last_yaw_deg) / dt_yaw;
                }
                last_yaw_deg = att.yaw;
                last_yaw_us = now_us;
            }

            // 2) 读取 EKF 内部估计的 gyro bias，并构造 omega
            Eigen::Vector3f bias = ekf.get_gyro_bias();
            Eigen::Vector3f omega = gyro - bias;

            // 3) 欧拉角奇异性判据（Pitch 接近 ±90° 时：Yaw/Roll 会变得不稳定，这是数学事实）
            // “前向轴投影到水平面”的长度 ~ |cos(pitch)|
            float pitch_rad = euler.y();
            float horiz = std::fabs(std::cos(pitch_rad)); // 越接近 0，航向越不可定义
            bool near_singularity = (std::fabs(att.pitch) > 80.0f);

            // 4) 打印策略：
            //    - 正常：1Hz
            //    - Pitch 接近 90°：5Hz，抓取欧拉角跳变的现场
            static uint64_t last_print_us = 0;
            uint64_t now_print_us = esp_timer_get_time();
            uint64_t period_us = near_singularity ? 200000 : 1000000;

            if (now_print_us - last_print_us > period_us)
            {
                last_print_us = now_print_us;

                ESP_LOGI(TAG,
                    "Euler(deg) R:%7.2f P:%7.2f Y:%7.2f | Ydot:%7.2f deg/s | dt:%7.4f | horiz:%6.3f%s",
                    att.roll, att.pitch, att.yaw,
                    yaw_rate_deg_s, dt, horiz,
                    near_singularity ? " [GIMBAL_LOCK_ZONE]" : "");

                ESP_LOGI(TAG,
                    "Gyro(rad/s) [%.5f %.5f %.5f] | Bias [%.5f %.5f %.5f] | Omega [%.5f %.5f %.5f] | AccNorm:%5.2f AccZ:%6.2f",
                    gyro.x(), gyro.y(), gyro.z(),
                    bias.x(), bias.y(), bias.z(),
                    omega.x(), omega.y(), omega.z(),
                    acc_norm, accel.z());
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
