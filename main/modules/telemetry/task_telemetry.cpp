#include "task_telemetry.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h" // 用于获取系统运行时间
#include "robot_bus.hpp"
#include "web_server.hpp"
#include "param_registry.hpp"
#include <math.h>
#include <stdio.h> // for snprintf

static const char* TAG = "TELEMETRY";
// 避免占用 Telemetry 任务栈（此前 1536B 局部数组是溢出诱因之一）
static char s_telem_json_buffer[1536];
static float s_telm_rate_hz_dash = 30.0f;
static float s_telm_rate_hz_safe = 20.0f;
static int32_t s_telm_auto_fallback_en = 1;

// 弧度转角度辅助函数
static float to_deg(float rad)
{
    return rad * 180.0f / M_PI;
}

void task_telemetry_entry(void* arg)
{
    auto& bus = RobotBus::instance();
    auto& params = ParamRegistry::instance();
    params.register_float("TELM_RATE_HZ_DASH", &s_telm_rate_hz_dash, s_telm_rate_hz_dash);
    params.register_float("TELM_RATE_HZ_SAFE", &s_telm_rate_hz_safe, s_telm_rate_hz_safe);
    params.register_int32("TELM_AUTO_FALLBACK_EN", &s_telm_auto_fallback_en, s_telm_auto_fallback_en);

    ESP_LOGI(TAG, "Telemetry Task Started on core %d. Broadcasting to Web...", xPortGetCoreID());

    uint64_t tx_window_start_us = esp_timer_get_time();
    uint32_t tx_window_packets = 0;
    uint32_t tx_window_ok = 0;
    uint32_t tx_window_fail = 0;
    uint32_t tx_window_json_max = 0;
    uint32_t tx_last_send_ok = 0;
    uint32_t tx_last_send_fail = 0;
    float tx_last_rate_hz = 0.0f;
    uint32_t tx_last_json_max = 0;
    uint32_t tx_last_json_len = 0;
    uint32_t tx_last_stack_hwm_bytes = 0;
    uint64_t stack_log_last_us = tx_window_start_us;
    uint32_t tx_last_sched_jitter_ms = 0;
    float tx_last_rate_target_hz = s_telm_rate_hz_dash;
    float tx_last_rate_active_hz = s_telm_rate_hz_dash;
    int tx_last_fallback_active = 0;
    uint64_t last_param_sync_us = tx_window_start_us;
    uint32_t fallback_bad_streak_sec = 0;
    uint32_t fallback_recover_sec = 0;
    bool fallback_active = false;
    float active_rate_hz = s_telm_rate_hz_dash;
    const float min_rate_hz = 5.0f;
    const float max_rate_hz = 60.0f;
    const uint32_t send_us_max_guard = 20000U;
    uint64_t send_cost_sum_us = 0;
    uint32_t send_cost_cnt = 0;
    uint32_t send_cost_max_us = 0;
    uint64_t fmt_cost_sum_us = 0;
    uint32_t fmt_cost_cnt = 0;
    uint32_t fmt_cost_max_us = 0;
    uint64_t loop_cost_sum_us = 0;
    uint32_t loop_cost_cnt = 0;
    uint32_t loop_cost_max_us = 0;

    // 周期化调度：相比 vTaskDelay 更稳定，能减少任务被抢占后的节拍漂移
    TickType_t last_wake_tick = xTaskGetTickCount();
    while (1)
    {
        const uint64_t now_for_rate_us = esp_timer_get_time();
        if (now_for_rate_us - last_param_sync_us >= 1000000ULL)
        {
            last_param_sync_us = now_for_rate_us;
            if (s_telm_rate_hz_dash < min_rate_hz) s_telm_rate_hz_dash = min_rate_hz;
            if (s_telm_rate_hz_dash > max_rate_hz) s_telm_rate_hz_dash = max_rate_hz;
            if (s_telm_rate_hz_safe < min_rate_hz) s_telm_rate_hz_safe = min_rate_hz;
            if (s_telm_rate_hz_safe > max_rate_hz) s_telm_rate_hz_safe = max_rate_hz;
            if (s_telm_rate_hz_safe > s_telm_rate_hz_dash) s_telm_rate_hz_safe = s_telm_rate_hz_dash;
            if (s_telm_auto_fallback_en == 0 && fallback_active)
            {
                fallback_active = false;
                fallback_bad_streak_sec = 0;
                fallback_recover_sec = 0;
            }
        }

        active_rate_hz = fallback_active ? s_telm_rate_hz_safe : s_telm_rate_hz_dash;
        const TickType_t loop_period_tick = pdMS_TO_TICKS((uint32_t)(1000.0f / active_rate_hz + 0.5f));
        vTaskDelayUntil(&last_wake_tick, (loop_period_tick > 0) ? loop_period_tick : 1);
        const uint64_t loop_begin_us = esp_timer_get_time();

        // 2. 从总线获取最新数据 (线程安全)
        // [说明] 依然获取原始 IMU/Mag 数据，用于发送给前端进行波形显示和校准
        ImuData imu = bus.imu.get();
        MagData mag = bus.mag.get();

        // [新增] 获取 EKF 发布的姿态数据 (已在 Estimator 任务中解算好)
        AttitudeData att = bus.attitude.get();

        // [新增] 获取EKF调试数据
        EKFDebugData ekf_dbg = bus.ekf_debug.get();

        // 3. 姿态解算 (Attitude Estimation)
        // [修改] 现在直接使用 EKF 的结果，不再手动计算
        float roll = att.roll;
        float pitch = att.pitch;
        float yaw = att.yaw;

        /* [旧逻辑保留] 原始的手动解算代码已注释，改用上面的 EKF 数据
           保留此处作为参考或对比
        // 目前仅使用加速度计 (Accelerometer Only) 计算 Roll/Pitch

        // 防止除以0
        if (imu.az != 0)
        {
            // Roll (横滚角): 绕 X 轴旋转
            // 利用重力分量在 Y 和 Z 轴的投影计算
            roll = to_deg(atan2(imu.ay, imu.az));

            // Pitch (俯仰角): 绕 Y 轴旋转
            // 利用重力分量在 X 轴的投影计算
            pitch = to_deg(atan2(-imu.ax, sqrt(imu.ay * imu.ay + imu.az * imu.az)));
        }

        // Yaw (偏航角): 简单使用磁力计
        // 注意：这里没有进行倾斜补偿 (Tilt Compensation)，所以只有水平放置时才准
        if (mag.x != 0 || mag.y != 0)
        {
            // atan2(y, x) 返回的是弧度，转成角度
            yaw = to_deg(atan2(mag.y, mag.x));
        }
        */

        // 4. 获取系统信息
        // 剩余内存 (KB)
        float free_heap_kb = (float)esp_get_free_heap_size() / 1024.0f;
        // 运行时间 (秒)
        uint32_t uptime_sec = (uint32_t)(esp_timer_get_time() / 1000000ULL);

        // 5. 模拟电池电压 (让界面好看点)
        // 实际项目中应读取 ADC
        float voltage = 12.0f + (float)(rand() % 10) / 100.0f;

        // 6. 打包 JSON
        // 格式必须严格匹配 app.js 中的 handleDataPacket

        // [安全检查] 确保EKF调试数据有效
        float yaw_meas_deg = 0.0f;
        float yaw_pred_deg = 0.0f;
        float yaw_res = 0.0f;
        float bias_x = 0.0f, bias_y = 0.0f, bias_z = 0.0f;
        float k_yaw_bias = 0.0f;
        int accel_used = 0;
        int mag_used = 0;
        float acc_weight = 0.0f;
        float mag_weight = 0.0f;
        float yaw_res_std = 0.0f;
        float acc_norm = 0.0f;
        float mag_wx = 0.0f, mag_wy = 0.0f, mag_wz = 0.0f;
        float p_yaw = 0.01f;
        float rate_imu_hz = 0.0f;
        float rate_predict_hz = 0.0f;
        float rate_acc_fuse_hz = 0.0f;
        float rate_mag_fuse_hz = 0.0f;

        // 只有当EKF数据有效时才使用（timestamp_us > 0 表示已初始化）
        if (ekf_dbg.timestamp_us > 0) {
            yaw_meas_deg = to_deg(ekf_dbg.yaw_measured);
            yaw_pred_deg = to_deg(ekf_dbg.yaw_predicted);
            yaw_res = ekf_dbg.yaw_residual;
            bias_x = ekf_dbg.gyro_bias_x;
            bias_y = ekf_dbg.gyro_bias_y;
            bias_z = ekf_dbg.gyro_bias_z;
            k_yaw_bias = ekf_dbg.k_yaw_bias_z;
            accel_used = ekf_dbg.accel_used ? 1 : 0;
            mag_used = ekf_dbg.mag_used ? 1 : 0;
            acc_weight = ekf_dbg.acc_weight;
            mag_weight = ekf_dbg.mag_weight;
            yaw_res_std = ekf_dbg.yaw_residual_std;
            acc_norm = ekf_dbg.acc_norm;
            mag_wx = ekf_dbg.mag_world_x;
            mag_wy = ekf_dbg.mag_world_y;
            mag_wz = ekf_dbg.mag_world_z;
            p_yaw = ekf_dbg.P_yaw;
            rate_imu_hz = ekf_dbg.rate_imu_hz;
            rate_predict_hz = ekf_dbg.rate_predict_hz;
            rate_acc_fuse_hz = ekf_dbg.rate_acc_fuse_hz;
            rate_mag_fuse_hz = ekf_dbg.rate_mag_fuse_hz;
        }

        const uint64_t fmt_begin_us = esp_timer_get_time();
        int len = snprintf(s_telem_json_buffer, sizeof(s_telem_json_buffer),
            "{\"type\":\"telem\",\"payload\":{"
            "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
            "\"voltage\":%.2f,"
            "\"sys\":{\"heap\":%.1f,\"time\":%lu},"
            "\"mag\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
            "\"imu\":{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f},"
            "\"ekf\":{"
                "\"yaw_measured\":%.2f,\"yaw_predicted\":%.2f,\"yaw_residual\":%.4f,"
                "\"bias\":{\"x\":%.5f,\"y\":%.5f,\"z\":%.5f},"
                "\"k_yaw_bias_z\":%.6f,"
                "\"accel_used\":%d,\"mag_used\":%d,"
                "\"acc_weight\":%.3f,\"mag_weight\":%.3f,"
                "\"yaw_residual_std\":%.4f,\"acc_norm\":%.3f,"
                "\"rate\":{\"imu_hz\":%.1f,\"predict_hz\":%.1f,\"acc_fuse_hz\":%.1f,\"mag_fuse_hz\":%.1f},"
                "\"tx\":{\"rate_hz\":%.1f,\"send_ok\":%u,\"send_fail\":%u,\"json_len\":%d,\"json_len_max\":%u,\"stack_hwm\":%u,\"target_hz\":%.1f,\"active_hz\":%.1f,\"fallback\":%d},"
                "\"mag_world\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
                "\"P_yaw\":%.6f"
            "}"
            "}}",
            roll, pitch, yaw, voltage,
            free_heap_kb, (unsigned long)uptime_sec,
            mag.x, mag.y, mag.z,
            imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz,
            yaw_meas_deg, yaw_pred_deg, yaw_res,
            bias_x, bias_y, bias_z,
            k_yaw_bias,
            accel_used, mag_used,
            acc_weight, mag_weight,
            yaw_res_std, acc_norm,
            rate_imu_hz, rate_predict_hz, rate_acc_fuse_hz, rate_mag_fuse_hz,
            tx_last_rate_hz, (unsigned)tx_last_send_ok, (unsigned)tx_last_send_fail, (int)tx_last_json_len, (unsigned)tx_last_json_max, (unsigned)tx_last_stack_hwm_bytes,
            tx_last_rate_target_hz, tx_last_rate_active_hz, tx_last_fallback_active,
            mag_wx, mag_wy, mag_wz,
            p_yaw
        );
        const uint64_t fmt_end_us = esp_timer_get_time();
        const uint32_t fmt_cost_us = (uint32_t)(fmt_end_us - fmt_begin_us);
        fmt_cost_sum_us += fmt_cost_us;
        fmt_cost_cnt++;
        if (fmt_cost_us > fmt_cost_max_us) fmt_cost_max_us = fmt_cost_us;

        // [调试] 检查buffer是否溢出
        if (len >= sizeof(s_telem_json_buffer)) {
            ESP_LOGE(TAG, "JSON buffer overflow! Needed %d, have %zu", len, sizeof(s_telem_json_buffer));
        }
        if (len > 0)
        {
            tx_last_json_len = (uint32_t)len;
            tx_window_packets++;
            if ((uint32_t)len > tx_window_json_max) tx_window_json_max = (uint32_t)len;
        }

        // 7. 发送给 Web 端
        // 只要 buffer 没溢出，就发送
        if (len > 0 && len < sizeof(s_telem_json_buffer))
        {
            const uint64_t send_begin_us = esp_timer_get_time();
            esp_err_t send_ret = WebServer::instance().send_ws_message(s_telem_json_buffer);
            const uint64_t send_end_us = esp_timer_get_time();
            if (send_ret == ESP_OK) tx_window_ok++;
            else tx_window_fail++;

            const uint32_t send_cost_us = (uint32_t)(send_end_us - send_begin_us);
            send_cost_sum_us += send_cost_us;
            send_cost_cnt++;
            if (send_cost_us > send_cost_max_us) send_cost_max_us = send_cost_us;
        }

        const uint64_t now_us = esp_timer_get_time();
        const uint32_t loop_cost_us = (uint32_t)(now_us - loop_begin_us);
        loop_cost_sum_us += loop_cost_us;
        loop_cost_cnt++;
        if (loop_cost_us > loop_cost_max_us) loop_cost_max_us = loop_cost_us;

        if (now_us - tx_window_start_us >= 1000000ULL)
        {
            const float win_sec = (now_us - tx_window_start_us) * 1e-6f;
            tx_last_rate_hz = (win_sec > 1e-3f) ? (tx_window_packets / win_sec) : 0.0f;
            tx_last_send_ok = tx_window_ok;
            tx_last_send_fail = tx_window_fail;
            tx_last_json_max = tx_window_json_max;
            tx_last_rate_target_hz = s_telm_rate_hz_dash;
            tx_last_rate_active_hz = active_rate_hz;
            tx_last_fallback_active = fallback_active ? 1 : 0;

            const uint32_t loop_avg_us = (loop_cost_cnt > 0) ? (uint32_t)(loop_cost_sum_us / loop_cost_cnt) : 0;
            const uint32_t fmt_avg_us = (fmt_cost_cnt > 0) ? (uint32_t)(fmt_cost_sum_us / fmt_cost_cnt) : 0;
            const uint32_t send_avg_us = (send_cost_cnt > 0) ? (uint32_t)(send_cost_sum_us / send_cost_cnt) : 0;
            tx_last_sched_jitter_ms = (loop_cost_max_us > loop_avg_us) ? ((loop_cost_max_us - loop_avg_us) / 1000U) : 0U;

            const bool bad_this_sec = (tx_last_send_fail > 0) || (send_cost_max_us > send_us_max_guard);
            if (s_telm_auto_fallback_en != 0)
            {
                if (!fallback_active)
                {
                    fallback_bad_streak_sec = bad_this_sec ? (fallback_bad_streak_sec + 1U) : 0U;
                    if (fallback_bad_streak_sec >= 3U)
                    {
                        fallback_active = true;
                        fallback_recover_sec = 0;
                        ESP_LOGW(TAG, "[GUARD] Fallback to safe rate %.1fHz (dash %.1fHz)", s_telm_rate_hz_safe, s_telm_rate_hz_dash);
                    }
                }
                else
                {
                    if (bad_this_sec)
                    {
                        fallback_recover_sec = 0;
                    }
                    else
                    {
                        fallback_recover_sec++;
                        if (fallback_recover_sec >= 5U)
                        {
                            fallback_active = false;
                            fallback_bad_streak_sec = 0;
                            fallback_recover_sec = 0;
                            ESP_LOGI(TAG, "[GUARD] Exit fallback, restore dash rate %.1fHz", s_telm_rate_hz_dash);
                        }
                    }
                }
            }
            else
            {
                fallback_bad_streak_sec = 0;
                fallback_recover_sec = 0;
                fallback_active = false;
            }

            ESP_LOGI(TAG,
                "[TX] rate=%.1fHz target/active=%.1f/%.1fHz fb=%d ok/fail=%u/%u json(cur/max)=%u/%u loop_us(avg/max)=%u/%u fmt_us(avg/max)=%u/%u send_us(avg/max)=%u/%u jitter=%ums",
                tx_last_rate_hz,
                tx_last_rate_target_hz, tx_last_rate_active_hz, tx_last_fallback_active,
                (unsigned)tx_last_send_ok, (unsigned)tx_last_send_fail,
                (unsigned)tx_last_json_len, (unsigned)tx_last_json_max,
                (unsigned)loop_avg_us, (unsigned)loop_cost_max_us,
                (unsigned)fmt_avg_us, (unsigned)fmt_cost_max_us,
                (unsigned)send_avg_us, (unsigned)send_cost_max_us,
                (unsigned)tx_last_sched_jitter_ms);

            tx_window_start_us = now_us;
            tx_window_packets = 0;
            tx_window_ok = 0;
            tx_window_fail = 0;
            tx_window_json_max = 0;

            loop_cost_sum_us = 0;
            loop_cost_cnt = 0;
            loop_cost_max_us = 0;
            fmt_cost_sum_us = 0;
            fmt_cost_cnt = 0;
            fmt_cost_max_us = 0;
            send_cost_sum_us = 0;
            send_cost_cnt = 0;
            send_cost_max_us = 0;
        }

        // 低频监控 Telemetry 任务剩余栈，方便现场确认是否仍有溢出风险
        if (now_us - stack_log_last_us >= 5000000ULL)
        {
            const uint32_t stack_hwm_bytes = uxTaskGetStackHighWaterMark(NULL);
            tx_last_stack_hwm_bytes = stack_hwm_bytes;
            ESP_LOGI(TAG, "[STACK] telemetry_hwm=%u bytes", (unsigned)stack_hwm_bytes);
            if (stack_hwm_bytes < 1024U)
            {
                ESP_LOGW(TAG, "[STACK] telemetry stack is low (%u bytes)", (unsigned)stack_hwm_bytes);
            }
            stack_log_last_us = now_us;
        }
    }
}

void start_telemetry_task()
{
    // 增加到 8192：JSON 组包 + snprintf + 浮点格式化在 ESP-IDF 上栈开销较大
    // pin 到 core0，降低与 Sensor/Estimator(core1) 的直接抢占竞争
    xTaskCreatePinnedToCore(task_telemetry_entry, "Telemetry", 8192, NULL, 3, NULL, 0);
}
