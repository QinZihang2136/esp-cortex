#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "robot_bus.hpp"
#include "task_sensor.h"  // 只需要引用这个头文件

static const char* TAG = "MAIN";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "EspCortex System Starting...");

    // 1. 初始化数据总线
    RobotBus::instance().init();

    // 2. 启动各个模块
    start_sensor_task();

    // 未来：
    // start_control_task();
    // start_microros_task();

    ESP_LOGI(TAG, "All Systems Go.");
}