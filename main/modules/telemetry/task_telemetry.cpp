#include "task_telemetry.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h" // 用于获取系统运行时间
#include "robot_bus.hpp"
#include "web_server.hpp"
#include <math.h>
#include <stdio.h> // for snprintf

static const char* TAG = "TELEMETRY";

// 弧度转角度辅助函数
static float to_deg(float rad)
{
    return rad * 180.0f / M_PI;
}

void task_telemetry_entry(void* arg)
{
    // 准备一个足够大的缓冲区来存放 JSON 字符串
    char json_buffer[1024];  // 增大到1024字节以容纳EKF调试数据
    auto& bus = RobotBus::instance();

    ESP_LOGI(TAG, "Telemetry Task Started. Broadcasting to Web...");

    while (1)
    {
        // 1. 控制发送频率 (20Hz = 50ms)
        // 频率太高会阻塞 WiFi，太低网页看起来卡顿
        vTaskDelay(pdMS_TO_TICKS(50));

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
        float mag_wx = 0.0f, mag_wy = 0.0f, mag_wz = 0.0f;
        float p_yaw = 0.01f;

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
            mag_wx = ekf_dbg.mag_world_x;
            mag_wy = ekf_dbg.mag_world_y;
            mag_wz = ekf_dbg.mag_world_z;
            p_yaw = ekf_dbg.P_yaw;
        }

        int len = snprintf(json_buffer, sizeof(json_buffer),
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
            mag_wx, mag_wy, mag_wz,
            p_yaw
        );

        // [调试] 检查buffer是否溢出
        if (len >= sizeof(json_buffer)) {
            ESP_LOGE(TAG, "JSON buffer overflow! Needed %d, have %zu", len, sizeof(json_buffer));
        }

        // 7. 发送给 Web 端
        // 只要 buffer 没溢出，就发送
        if (len > 0 && len < sizeof(json_buffer))
        {
            WebServer::instance().send_ws_message(json_buffer);
        }
    }
}

void start_telemetry_task()
{
    // 优先级设为 3 (中等)，堆栈 4096 字节
    // 必须确保堆栈足够大，因为 snprintf 和 float 运算比较吃栈
    xTaskCreatePinnedToCore(task_telemetry_entry, "Telemetry", 4096, NULL, 3, NULL, 1);
}