#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"          // [新增] NVS Flash 驱动 (WiFi 需要)
#include "robot_bus.hpp"
#include "task_sensor.h"
#include "param_registry.hpp"
#include "wifi_manager.hpp"     // [新增] WiFi 管理组件
#include "spiffs_manager.hpp"   // [新增] SPIFFS 文件系统组件

static const char* TAG = "MAIN";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "EspCortex System Starting...");

    // =============================================================
    // 0. 系统底层初始化 (NVS & Storage)
    // =============================================================

    // [NVS Flash] 初始化
    // 注意：WiFi 驱动和 ParamRegistry 都依赖 NVS，必须最先初始化
    // 这里处理了 NVS 分区空间不足或版本不匹配的情况 (自动擦除重置)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // [文件系统] 挂载 SPIFFS
    // 负责挂载 'storage' 分区，以便后续读取 index.html 等网页文件
    SpiffsManager::instance().init();

    // =============================================================
    // 1. 基础服务初始化
    // =============================================================

    // [参数系统] 初始化参数注册表
    // 确保后续模块 (如 SensorTask) 启动时能读取到 NVS 中的配置
    ParamRegistry::instance().init();

    // [网络服务] 启动 WiFi 管理器
    // 逻辑：优先连接 "R&Q" (01200204)，连接失败则自动开启热点 "EspCortex"
    WifiManager::instance().init();

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

    // [新增] 打印文件系统使用情况 (方便检查网页文件是否烧录进去)
    SpiffsManager::instance().print_usage();

    ESP_LOGI(TAG, "All Systems Go. Main task entering idle loop.");

    // 主任务进入空闲循环，仅做心跳保活
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
        // 这里后续可以加一些系统级的监控，比如内存剩余量监控
        ESP_LOGI(TAG, "System Heartbeat: Free Heap %lu bytes", (unsigned long)esp_get_free_heap_size());
    }
}