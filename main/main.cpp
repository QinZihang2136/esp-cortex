#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "robot_bus.hpp"
#include "task_sensor.h"
#include "param_registry.hpp"

static const char* TAG = "MAIN";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "EspCortex System Starting...");

    // =============================================================
    // 1. 基础服务初始化
    // =============================================================

    // [参数系统] 最优先初始化，确保后续模块能读取到 NVS 中的配置
    ParamRegistry::instance().init();

    // [数据总线] 初始化 Topic 和 Queue
    RobotBus::instance().init();

    // =============================================================
    // 2. 任务与模块启动
    // =============================================================

    // [传感器任务]
    // 注意：传感器相关的参数 (如 MAG_OFFSET) 应在 start_sensor_task 内部进行注册
    start_sensor_task();

    // =============================================================
    // 3. 系统体检 (可选)
    // =============================================================

    // 打印当前系统中存在于 Flash 但未被代码使用的参数 (僵尸参数)
    // 帮助开发者发现废弃的配置
    ParamRegistry::instance().print_unused();

    ESP_LOGI(TAG, "All Systems Go. Main task entering idle loop.");

    // 主任务进入空闲循环，仅做心跳保活
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
        // 这里后续可以加一些系统级的监控，比如内存剩余量监控
        // ESP_LOGI(TAG, "System Heartbeat: Free Heap %lu bytes", esp_get_free_heap_size());
    }
}