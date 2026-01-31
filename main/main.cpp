#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"          // NVS Flash 驱动 (WiFi 需要)
#include "robot_bus.hpp"
#include "task_sensor.h"
#include "param_registry.hpp"
#include "wifi_manager.hpp"     // WiFi 管理组件
#include "spiffs_manager.hpp"   // SPIFFS 文件系统组件
#include "task_telemetry.h"
// WebServer 头文件
#include "web_server.hpp"
#include "task_estimator.hpp" // <--- 引入头文件

static const char* TAG = "MAIN";

// =============================================================
// [测试用] 定义全局参数变量
// 这些变量的值会被参数系统管理：
// 1. 系统启动时，尝试从 Flash 读取旧值覆盖这里。
// 2. 网页修改时，这里的值会立即更新。
// =============================================================
static float  pid_pitch_p = 1.50f;
static float  pid_pitch_i = 0.05f;
static float  pid_pitch_d = 0.80f;
static float  mag_declination = 4.5f; // 磁偏角
static int32_t sys_log_level = 3;     // 日志等级 (Int 测试)
static int32_t motor_direction = 1;   // 电机转向 (Int 测试)

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "EspCortex System Starting...");

    // =============================================================
    // 0. 系统底层初始化 (NVS & Storage)
    // =============================================================

    // [NVS Flash] 初始化
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // [文件系统] 挂载 SPIFFS
    // 必须在 WebServer 启动前完成，因为 WebServer 需要读取里面的网页文件
    SpiffsManager::instance().init();

    // =============================================================
    // 1. 基础服务初始化
    // =============================================================

    // [参数系统] 初始化参数注册表
    ParamRegistry::instance().init();

    // -------------------------------------------------------------
    // 注册测试参数 (在这里把变量交给 Registry 接管)
    // -------------------------------------------------------------
    auto& params = ParamRegistry::instance();

    // 注册浮点数 (Float)
    params.register_float("PID_PITCH_P", &pid_pitch_p, 1.50f);
    params.register_float("PID_PITCH_I", &pid_pitch_i, 0.05f);
    params.register_float("PID_PITCH_D", &pid_pitch_d, 0.80f);
    params.register_float("MAG_DECLIN", &mag_declination, 4.5f);

    // 注册整数 (Int32)
    params.register_int32("SYS_LOG_LEVEL", &sys_log_level, 3);
    params.register_int32("MOTOR_DIR", &motor_direction, 1);

    ESP_LOGI(TAG, "Test Params Registered. P:%.2f I:%.2f D:%.2f", pid_pitch_p, pid_pitch_i, pid_pitch_d);
    // -------------------------------------------------------------

    // [网络服务] 启动 WiFi 管理器
    // 必须在 WebServer 启动前完成，因为 WebServer 依赖网络栈
    WifiManager::instance().init();

    // =============================================================
    // 启动 Web 服务器 (包含 mDNS)
    // =============================================================
    // 启动后，你可以在浏览器访问 http://espcortex.local 或 http://<IP地址>
    WebServer::instance().init();

    // [数据总线] 初始化 Topic 和 Queue
    RobotBus::instance().init();

    // =============================================================
    // 2. 任务与模块启动
    // =============================================================

    // [传感器任务]
    start_sensor_task();

    //启动 Estimator (它负责消费数据算姿态)
    start_estimator_task();

    // [遥测任务]
    start_telemetry_task();

    // =============================================================
    // 3. 系统体检 (可选)
    // =============================================================

    ParamRegistry::instance().print_unused();
    SpiffsManager::instance().print_usage();

    ESP_LOGI(TAG, "All Systems Go. Main task entering idle loop.");

    // 主任务进入空闲循环
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "System Heartbeat: Free Heap %lu bytes", (unsigned long)esp_get_free_heap_size());

    }
}
