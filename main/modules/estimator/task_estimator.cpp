#include "task_estimator.hpp"
#include "ekf.hpp"
#include "robot_bus.hpp"
#include "param_registry.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "debug_log.hpp"
#include <cmath>

static const char* TAG = "ESTIMATOR";
DEBUG_ESTIMATOR_INIT(TAG)

static constexpr float RAD_TO_DEG = 57.29578f;

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
    EspEKF ekf;

    // FRD specific force convention: flat rest is ~[0,0,-9.81].
    ekf.init(Eigen::Vector3f(0.0f, 0.0f, -9.81f));

    auto& bus = RobotBus::instance();
    auto& params = ParamRegistry::instance();

    // EKF runtime parameters (registered once; values loaded from NVS if available)
    static float ekf_q_angle = 0.001f;
    static float ekf_q_gyro_bias = 0.0001f;
    static float ekf_q_accel_bias = 0.0001f;
    static float ekf_r_accel_base = 0.20f;
    static float ekf_r_mag_base = 0.15f;
    static float ekf_mag_w_min = 0.10f;
    static float ekf_mag_gate_deg = 30.0f;
    static float ekf_acc_gate_ms2 = 3.5f;
    static float ekf_declin_deg = 4.5f;

    params.register_float("EKF_Q_ANGLE", &ekf_q_angle, ekf_q_angle);
    params.register_float("EKF_Q_GYRO_BIAS", &ekf_q_gyro_bias, ekf_q_gyro_bias);
    params.register_float("EKF_Q_ACCEL_BIAS", &ekf_q_accel_bias, ekf_q_accel_bias);
    params.register_float("EKF_R_ACCEL_BASE", &ekf_r_accel_base, ekf_r_accel_base);
    params.register_float("EKF_R_MAG_BASE", &ekf_r_mag_base, ekf_r_mag_base);
    params.register_float("EKF_MAG_W_MIN", &ekf_mag_w_min, ekf_mag_w_min);
    params.register_float("EKF_MAG_GATE_DEG", &ekf_mag_gate_deg, ekf_mag_gate_deg);
    params.register_float("EKF_ACC_GATE_MS2", &ekf_acc_gate_ms2, ekf_acc_gate_ms2);
    params.register_float("EKF_DECLIN_DEG", &ekf_declin_deg, ekf_declin_deg);

    ekf.set_process_noise(ekf_q_angle, ekf_q_gyro_bias, ekf_q_accel_bias);
    ekf.set_measure_noise_accel(ekf_r_accel_base);
    ekf.set_measure_noise_mag(ekf_r_mag_base);
    ekf.set_mag_declination(ekf_declin_deg);
    ekf.set_adaptive_fusion_config(ekf_mag_w_min, ekf_mag_gate_deg, ekf_acc_gate_ms2);

    bus.imu.register_notifier();
    bus.mag.register_notifier();

    ImuData imu_data{};
    uint32_t last_imu_gen = 0;

    MagData mag_data_snapshot{};
    uint32_t last_mag_gen = 0;

    DEBUG_ESTIMATOR_LOG("Estimator Task Started (FRD specific-force convention).");

    static bool dbg_inited = false;
    static float last_yaw_deg = 0.0f;
    static uint64_t last_yaw_us = 0;

    bool imu_time_inited = false;
    uint64_t last_imu_time_us = 0;

    bool wall_time_inited = false;
    uint64_t last_wall_us = 0;

    uint32_t cnt_imu_update = 0;
    uint32_t cnt_time_backwards = 0;
    uint32_t cnt_dt_capped = 0;
    uint32_t cnt_acc_reject = 0;
    uint32_t cnt_acc_accept = 0;
    uint32_t cnt_mag_reject = 0;
    uint32_t cnt_mag_accept = 0;
    uint32_t cnt_predict_call = 0;
    uint32_t cnt_acc_fuse_call = 0;
    uint32_t cnt_mag_fuse_call = 0;
    uint32_t win_imu_update = 0;
    uint32_t win_predict_call = 0;
    uint32_t win_acc_fuse_call = 0;
    uint32_t win_mag_fuse_call = 0;

    const float mag_min_valid = 0.05f;
    const float mag_max_valid = 2.00f;

    uint64_t last_param_sync_us = 0;
    uint64_t last_rate_window_us = 0;
    float last_rate_imu_hz = 0.0f;
    float last_rate_predict_hz = 0.0f;
    float last_rate_acc_fuse_hz = 0.0f;
    float last_rate_mag_fuse_hz = 0.0f;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        if (bus.imu.copy_if_updated(imu_data, last_imu_gen))
        {
            cnt_imu_update++;
            win_imu_update++;

            const uint64_t now_imu_us = imu_data.timestamp_us;
            const uint64_t now_wall_us = esp_timer_get_time();

            float dt_imu = 0.0f;
            float dt_wall = 0.0f;

            if (!imu_time_inited)
            {
                imu_time_inited = true;
                last_imu_time_us = now_imu_us;
                continue;
            }

            if (now_imu_us <= last_imu_time_us)
            {
                cnt_time_backwards++;
                last_imu_time_us = now_imu_us;
                continue;
            }
            dt_imu = (now_imu_us - last_imu_time_us) * 1e-6f;
            last_imu_time_us = now_imu_us;

            if (!wall_time_inited)
            {
                wall_time_inited = true;
                last_wall_us = now_wall_us;
            }
            else
            {
                if (now_wall_us > last_wall_us)
                {
                    dt_wall = (now_wall_us - last_wall_us) * 1e-6f;
                }
                last_wall_us = now_wall_us;
            }

            float dt = dt_imu;
            if (dt > 0.1f)
            {
                cnt_dt_capped++;
                dt = 0.005f;
            }

            if (now_wall_us - last_param_sync_us > 1000000ULL)
            {
                last_param_sync_us = now_wall_us;
                ekf.set_process_noise(ekf_q_angle, ekf_q_gyro_bias, ekf_q_accel_bias);
                ekf.set_measure_noise_accel(ekf_r_accel_base);
                ekf.set_measure_noise_mag(ekf_r_mag_base);
                ekf.set_mag_declination(ekf_declin_deg);
                ekf.set_adaptive_fusion_config(ekf_mag_w_min, ekf_mag_gate_deg, ekf_acc_gate_ms2);
            }

            Eigen::Vector3f gyro;
            gyro.x() = imu_data.gx;
            gyro.y() = imu_data.gy;
            gyro.z() = imu_data.gz;

            ekf.predict(gyro, dt);
            cnt_predict_call++;
            win_predict_call++;

            Eigen::Vector3f accel;
            accel.x() = imu_data.ax;
            accel.y() = imu_data.ay;
            accel.z() = imu_data.az;

            const float acc_norm = accel.norm();
            const bool acc_gate_ok = (acc_norm > 2.0f && acc_norm < 30.0f);

            if (acc_gate_ok)
            {
                cnt_acc_accept++;
                ekf.fuse_accel(accel, gyro.norm());
                cnt_acc_fuse_call++;
                win_acc_fuse_call++;
            }
            else
            {
                cnt_acc_reject++;
            }

            if (bus.mag.copy_if_updated(mag_data_snapshot, last_mag_gen))
            {
                const float mag_norm = std::sqrt(mag_data_snapshot.x * mag_data_snapshot.x +
                    mag_data_snapshot.y * mag_data_snapshot.y +
                    mag_data_snapshot.z * mag_data_snapshot.z);

                const bool mag_gate_ok = (mag_norm > mag_min_valid && mag_norm < mag_max_valid);

                if (mag_gate_ok)
                {
                    cnt_mag_accept++;
                    Eigen::Vector3f mag_vec;
                    mag_vec.x() = mag_data_snapshot.x;
                    mag_vec.y() = mag_data_snapshot.y;
                    mag_vec.z() = mag_data_snapshot.z;
                    ekf.fuse_mag(mag_vec);
                    cnt_mag_fuse_call++;
                    win_mag_fuse_call++;
                }
                else
                {
                    cnt_mag_reject++;
                }
            }

            Eigen::Vector3f euler = ekf.get_euler_angles();

            AttitudeData att;
            att.roll = euler.x() * RAD_TO_DEG;
            att.pitch = euler.y() * RAD_TO_DEG;
            att.yaw = wrap_deg(euler.z() * RAD_TO_DEG);
            bus.attitude.publish(att);

            auto accel_dbg = ekf.get_last_accel_debug();
            auto mag_dbg = ekf.get_last_mag_debug();

            EKFDebugData ekf_dbg{};
            ekf_dbg.yaw_measured = mag_dbg.yaw_measured;
            ekf_dbg.yaw_predicted = mag_dbg.yaw_predicted;
            ekf_dbg.yaw_residual = mag_dbg.yaw_residual;

            Eigen::Vector3f bias = ekf.get_gyro_bias();
            ekf_dbg.gyro_bias_x = bias.x();
            ekf_dbg.gyro_bias_y = bias.y();
            ekf_dbg.gyro_bias_z = bias.z();

            ekf_dbg.accel_residual_x = accel_dbg.innov.x();
            ekf_dbg.accel_residual_y = accel_dbg.innov.y();
            ekf_dbg.accel_residual_z = accel_dbg.innov.z();
            ekf_dbg.accel_used = accel_dbg.used;

            ekf_dbg.mag_world_x = mag_dbg.m_world.x();
            ekf_dbg.mag_world_y = mag_dbg.m_world.y();
            ekf_dbg.mag_world_z = mag_dbg.m_world.z();
            ekf_dbg.mag_body_x = mag_dbg.m_body.x();
            ekf_dbg.mag_body_y = mag_dbg.m_body.y();
            ekf_dbg.mag_body_z = mag_dbg.m_body.z();
            ekf_dbg.mag_used = mag_dbg.used;

            ekf_dbg.mag_weight = mag_dbg.mag_weight;
            ekf_dbg.acc_weight = accel_dbg.acc_weight;
            ekf_dbg.yaw_residual_std = mag_dbg.yaw_residual_std;
            ekf_dbg.acc_norm = accel_dbg.acc_norm;
            ekf_dbg.rate_imu_hz = last_rate_imu_hz;
            ekf_dbg.rate_predict_hz = last_rate_predict_hz;
            ekf_dbg.rate_acc_fuse_hz = last_rate_acc_fuse_hz;
            ekf_dbg.rate_mag_fuse_hz = last_rate_mag_fuse_hz;

            ekf_dbg.P_yaw = ekf.get_P_yaw();
            ekf_dbg.k_yaw_bias_z = ekf.get_last_mag_k_yaw_bias_z();
            ekf_dbg.timestamp_us = now_imu_us;
            bus.ekf_debug.publish(ekf_dbg);

            float yaw_rate_deg_s = 0.0f;
            if (!dbg_inited)
            {
                dbg_inited = true;
                last_yaw_deg = att.yaw;
                last_yaw_us = now_imu_us;
            }
            else
            {
                const float dt_yaw = (now_imu_us - last_yaw_us) * 1e-6f;
                if (dt_yaw > 1e-4f && dt_yaw < 0.2f)
                {
                    yaw_rate_deg_s = delta_deg(att.yaw, last_yaw_deg) / dt_yaw;
                }
                last_yaw_deg = att.yaw;
                last_yaw_us = now_imu_us;
            }

            Eigen::Vector3f omega = gyro - bias;

            const float pitch_rad = euler.y();
            const float horiz = std::fabs(std::cos(pitch_rad));
            const bool near_singularity = (std::fabs(att.pitch) > 80.0f);

            static uint64_t last_print_us = 0;
            const uint64_t now_print_us = esp_timer_get_time();
            const uint64_t period_us = near_singularity ? DEBUG_ESTIMATOR_PRINT_INTERVAL_FAST : DEBUG_ESTIMATOR_PRINT_INTERVAL_NORMAL;

            if (now_print_us - last_print_us > period_us)
            {
                last_print_us = now_print_us;
                const float dt_diff = (dt_wall > 0.0f) ? (dt_imu - dt_wall) : 0.0f;

                DEBUG_ESTIMATOR_EULER_LOG(
                    "Euler(deg) R:%7.2f P:%7.2f Y:%7.2f | Ydot:%7.2f deg/s | dt:%7.4f | horiz:%6.3f%s",
                    att.roll, att.pitch, att.yaw,
                    yaw_rate_deg_s, dt, horiz,
                    near_singularity ? " [GIMBAL_LOCK_ZONE]" : "");

                DEBUG_ESTIMATOR_GYRO_LOG(
                    "Gyro(rad/s) [%.5f %.5f %.5f] | Bias [%.5f %.5f %.5f] | Omega [%.5f %.5f %.5f]",
                    gyro.x(), gyro.y(), gyro.z(),
                    bias.x(), bias.y(), bias.z(),
                    omega.x(), omega.y(), omega.z());

                DEBUG_ESTIMATOR_ACCEL_LOG(
                    "Accel(FRD) [%.2f %.2f %.2f] | Norm:%5.2f | Gate:%s | W_acc:%.3f",
                    accel.x(), accel.y(), accel.z(),
                    acc_norm, acc_gate_ok ? "OK" : "REJ",
                    accel_dbg.acc_weight);

                DEBUG_ESTIMATOR_TIME_LOG(
                    "Time dt_imu:%0.4f dt_wall:%0.4f diff:%+0.4f",
                    dt_imu, dt_wall, dt_diff);

                DEBUG_ESTIMATOR_STATISTICS_LOG(
                    "Stats imu=%u back=%u cap=%u acc_ok=%u acc_rej=%u mag_ok=%u mag_rej=%u",
                    (unsigned)cnt_imu_update,
                    (unsigned)cnt_time_backwards,
                    (unsigned)cnt_dt_capped,
                    (unsigned)cnt_acc_accept,
                    (unsigned)cnt_acc_reject,
                    (unsigned)cnt_mag_accept,
                    (unsigned)cnt_mag_reject);

                DEBUG_ESTIMATOR_MAG_LOG("Mag(FRD) [%.3f %.3f %.3f] | W_mag:%.3f std:%.3f",
                    mag_data_snapshot.x, mag_data_snapshot.y, mag_data_snapshot.z,
                    mag_dbg.mag_weight, mag_dbg.yaw_residual_std);
            }

            // 频率统计窗口：每 1 秒输出一次实时频率（用于评估真实 EKF 运行频率）
            if (last_rate_window_us == 0)
            {
                last_rate_window_us = now_wall_us;
            }
            else if (now_wall_us - last_rate_window_us >= 1000000ULL)
            {
                const float win_sec = (now_wall_us - last_rate_window_us) * 1e-6f;
                const float imu_hz = (win_sec > 1e-3f) ? (win_imu_update / win_sec) : 0.0f;
                const float predict_hz = (win_sec > 1e-3f) ? (win_predict_call / win_sec) : 0.0f;
                const float acc_fuse_hz = (win_sec > 1e-3f) ? (win_acc_fuse_call / win_sec) : 0.0f;
                const float mag_fuse_hz = (win_sec > 1e-3f) ? (win_mag_fuse_call / win_sec) : 0.0f;
                last_rate_imu_hz = imu_hz;
                last_rate_predict_hz = predict_hz;
                last_rate_acc_fuse_hz = acc_fuse_hz;
                last_rate_mag_fuse_hz = mag_fuse_hz;

                DEBUG_ESTIMATOR_LOG(
                    "[RATE] win=%.2fs imu=%.1fHz predict=%.1fHz acc_fuse=%.1fHz mag_fuse=%.1fHz",
                    win_sec, imu_hz, predict_hz, acc_fuse_hz, mag_fuse_hz);

                // 重置窗口计数（保持原有总计数器不变）
                win_imu_update = 0;
                win_predict_call = 0;
                win_acc_fuse_call = 0;
                win_mag_fuse_call = 0;
                last_rate_window_us = now_wall_us;
            }
        }
    }
}

void start_estimator_task()
{
    xTaskCreatePinnedToCore(task_estimator_entry, "Estimator", 8192, NULL, 4, NULL, 1);
}
