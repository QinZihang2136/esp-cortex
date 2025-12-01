#include "task_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

// 引入所有硬件驱动
#include "board_config.h"
#include "robot_bus.hpp"
#include "spi_bus.h"
#include "i2c_bus.h"
#include "icm42688.h"
#include "qmc5883l.h"
#include "icp20100.h" // 新增

static const char* TAG = "TASK_SENSOR";

// --- 静态硬件对象 ---
static SPIBus spi_bus(SPI2_HOST);
static ICM42688 imu(&spi_bus, PIN_SPI_CS_IMU);

// 假设 SCL=11, SDA=10 (请核对 board_config.h)
static I2CBus i2c_bus(I2C_NUM_0);
static QMC5883L mag(&i2c_bus);
static ICP20100 baro(&i2c_bus); // 新增

static SemaphoreHandle_t sem_imu_drdy = NULL;

// --- ISR ---
static void IRAM_ATTR imu_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(sem_imu_drdy, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

// --- 任务实体 ---
static void task_sensor_entry(void* arg)
{
    auto& bus = RobotBus::instance();
    ImuData imu_data = {};
    MagData mag_data = {};
    BaroData baro_data = {};

    int tick_counter = 0; // 用于分频计数

    // 1. 初始化 SPI & IMU
    spi_bus.begin(PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_CLK);
    vTaskDelay(pdMS_TO_TICKS(100));
    if (imu.begin() != ESP_OK) ESP_LOGE(TAG, "IMU Init Failed");

    // 2. 初始化 I2C & 磁力计 & 气压计
    // 初始化 I2C, 频率 400kHz
    i2c_bus.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    vTaskDelay(pdMS_TO_TICKS(50));

    // ================== I2C 扫描器开始 ==================
    ESP_LOGW(TAG, ">>> 开始 I2C 总线扫描 <<<");
    int devices_found = 0;
    for (int addr = 1; addr < 127; addr++)
    {
        // 尝试向该地址发送一个空写命令
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        // 发送命令，超时 50ms
        // 注意：这里需要传入 I2C 端口号，我们假设 I2CBus 类使用了 I2C_NUM_0
        // 如果你的 i2c_bus.cpp 里用的不是 0，请修改这里
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "发现 I2C 设备: 0x%02X", addr);
            devices_found++;

            // 简单判断一下是谁
            if (addr == 0x0D) ESP_LOGI(TAG, "  -> 可能是 QMC5883L");
            if (addr == 0x63) ESP_LOGI(TAG, "  -> 可能是 ICP-20100 (ADO=GND)");
            if (addr == 0x64) ESP_LOGI(TAG, "  -> 可能是 ICP-20100 (ADO=High)");
        }
    }
    if (devices_found == 0)
    {
        ESP_LOGE(TAG, "未发现任何 I2C 设备！请检查 SDA/SCL 接线！");
    }
    ESP_LOGW(TAG, ">>> 扫描结束，共发现 %d 个设备 <<<", devices_found);

    if (mag.begin() != ESP_OK) ESP_LOGE(TAG, "Mag Init Failed");
    if (baro.begin() != ESP_OK) ESP_LOGE(TAG, "Baro Init Failed");

    // 3. 开启 IMU 中断
    sem_imu_drdy = xSemaphoreCreateBinary();
    imu.enableInterrupt(PIN_IMU_INT, imu_isr_handler);

    ESP_LOGI(TAG, "All Sensors Initialized. Loop Start.");

    while (true)
    {
        // 等待 IMU 中断 (200Hz)
        if (xSemaphoreTake(sem_imu_drdy, pdMS_TO_TICKS(20)) == pdTRUE)
        {

            // --- A. 读取 IMU (每次都读) ---
            imu.getAccel(&imu_data.ax, &imu_data.ay, &imu_data.az);
            imu.getGyro(&imu_data.gx, &imu_data.gy, &imu_data.gz);
            imu_data.timestamp_us = esp_timer_get_time();
            bus.imu.publish(imu_data);

            // --- B. 分频读取磁力计 (100Hz -> 2分频) ---
            if (tick_counter % 2 == 0)
            {
                if (mag.isDataReady())
                {
                    mag.readData(&mag_data.x, &mag_data.y, &mag_data.z);
                    bus.mag.publish(mag_data);
                }
            }

            // --- C. 分频读取气压计 (25Hz -> 8分频) ---
            if (tick_counter % 8 == 0)
            {
                // ICP20100 没有 DataReady 引脚，直接读
                baro.readData(&baro_data.pressure, &baro_data.temperature);
                bus.baro.publish(baro_data);
            }

            // 调试打印 (1秒一次)
            if (tick_counter % 200 == 0)
            {
                ESP_LOGI(TAG, "IMU: %.2f | Mag: %.2f | Press: %.2f kPa",
                    imu_data.az, mag_data.z, baro_data.pressure);
            }

            tick_counter++;
        }
    }
}

void start_sensor_task()
{
    xTaskCreatePinnedToCore(task_sensor_entry, "Sensor", 4096, NULL, 5, NULL, 1);
}