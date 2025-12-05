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
    char json_buffer[512];
    auto& bus = RobotBus::instance();

    ESP_LOGI(TAG, "Telemetry Task Started. Broadcasting to Web...");

    while (1)
    {
        // 1. 控制发送频率 (20Hz = 50ms)
        // 频率太高会阻塞 WiFi，太低网页看起来卡顿
        vTaskDelay(pdMS_TO_TICKS(50));

        // 2. 从总线获取最新数据 (线程安全)
        ImuData imu = bus.imu.get();
        MagData mag = bus.mag.get();

        // 3. 姿态解算 (Attitude Estimation)
        // 目前仅使用加速度计 (Accelerometer Only) 计算 Roll/Pitch
        float roll = 0.0f;
        float pitch = 0.0f;
        float yaw = 0.0f;

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
        int len = snprintf(json_buffer, sizeof(json_buffer),
            "{\"type\":\"telem\",\"payload\":{"
            "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
            "\"voltage\":%.2f,"
            "\"sys\":{\"heap\":%.1f,\"time\":%lu},"
            "\"mag\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
            // [新增] 必须把 IMU 数据发给前端用于校准
            "\"imu\":{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f}"
            "}}",
            roll, pitch, yaw, voltage,
            free_heap_kb, (unsigned long)uptime_sec,
            mag.x, mag.y, mag.z,
            // 填入 IMU 数据
            imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz
        );

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