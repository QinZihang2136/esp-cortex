#include "task_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "param_registry.hpp" // [必须] 引入参数系统
#include "debug_log.hpp"       // 调试打印控制 (来自 common 组件)
// 引入所有硬件驱动
#include "board_config.h"
#include "robot_bus.hpp"
#include "spi_bus.h"
#include "i2c_bus.h"
#include "icm42688.h"
#include "qmc5883l.h"
#include "icp20100.h"
#include "lps22hh.h" // [新增] 引入 LPS22HH 驱动头文件
#include <math.h> // [必须] 引入 math.h 用于 pow() 函数

static const char* TAG = "TASK_SENSOR";
DEBUG_SENSOR_INIT(TAG)

// --- 静态硬件对象 ---
static SPIBus spi_bus(SPI2_HOST);
static ICM42688 imu(&spi_bus, PIN_SPI_CS_IMU);

// 假设 SCL=11, SDA=10 (请核对 board_config.h)
static I2CBus i2c_bus(I2C_NUM_0);
static QMC5883L mag(&i2c_bus);

// [修改] 定义气压计对象和状态管理
static ICP20100 baro_icp(&i2c_bus); // 原有的 ICP20100 对象
static LPS22HH* baro_lps = nullptr; // [新增] LPS22HH 指针 (动态指向 5C 或 5D 实例)

// [新增] 活跃气压计类型标记
enum BaroType { BARO_NONE, BARO_ICP20100, BARO_LPS22HH };
static BaroType active_baro = BARO_NONE;

static SemaphoreHandle_t sem_imu_drdy = NULL;

// ==========================================
// [新增] 定义校准参数变量 (放在全局或静态区)
// ==========================================
// 默认 Offset = 0 (不偏移)
static float mag_offset_x = 0.0f;
static float mag_offset_y = 0.0f;
static float mag_offset_z = 0.0f;
// 默认 Scale = 1.0 (不缩放)
static float mag_scale_x = 1.0f;
static float mag_scale_y = 1.0f;
static float mag_scale_z = 1.0f;

// 2. 陀螺仪 (Gyro) - 只有零偏
static float gyro_offset_x = 0.0f;
static float gyro_offset_y = 0.0f;
static float gyro_offset_z = 0.0f;

// 3. 加速度计 (Accel) - 零偏 + 缩放
static float accel_offset_x = 0.0f;
static float accel_offset_y = 0.0f;
static float accel_offset_z = 0.0f;
static float accel_scale_x = 1.0f;
static float accel_scale_y = 1.0f;
static float accel_scale_z = 1.0f;

// 气压转海拔计算
static float calculate_altitude(float pressure_kpa)
{
    // 标准海平面气压 101.325 kPa
    const float P0 = 101.325f;
    if (pressure_kpa <= 0.0f) return 0.0f;

    // 公式: h = 44330 * (1 - (P / P0)^(1/5.255))
    // 注意：这里计算的是"气压高度"，即假设当前处于标准大气压环境下的相对高度。
    return 44330.0f * (1.0f - powf(pressure_kpa / P0, 0.1903f));
}

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
    auto& params = ParamRegistry::instance(); // 获取参数单例
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

    // ==========================================
    // [新增] 注册校准参数
    // ==========================================
    // 注册 Mag 参数
    params.register_float("MAG_OFFSET_X", &mag_offset_x, 0.0f);
    params.register_float("MAG_OFFSET_Y", &mag_offset_y, 0.0f);
    params.register_float("MAG_OFFSET_Z", &mag_offset_z, 0.0f);
    params.register_float("MAG_SCALE_X", &mag_scale_x, 1.0f);
    params.register_float("MAG_SCALE_Y", &mag_scale_y, 1.0f);
    params.register_float("MAG_SCALE_Z", &mag_scale_z, 1.0f);

    // 注册 Gyro 参数
    params.register_float("GYRO_OFFSET_X", &gyro_offset_x, 0.0f);
    params.register_float("GYRO_OFFSET_Y", &gyro_offset_y, 0.0f);
    params.register_float("GYRO_OFFSET_Z", &gyro_offset_z, 0.0f);

    // 注册 Accel 参数
    params.register_float("ACCEL_OFFSET_X", &accel_offset_x, 0.0f);
    params.register_float("ACCEL_OFFSET_Y", &accel_offset_y, 0.0f);
    params.register_float("ACCEL_OFFSET_Z", &accel_offset_z, 0.0f);
    params.register_float("ACCEL_SCALE_X", &accel_scale_x, 1.0f);
    params.register_float("ACCEL_SCALE_Y", &accel_scale_y, 1.0f);
    params.register_float("ACCEL_SCALE_Z", &accel_scale_z, 1.0f);

    ESP_LOGI(TAG, "Sensor Params Loaded.");

    // ================== I2C 扫描器 ==================
    DEBUG_SENSOR_I2C_SCAN_LOG(">>> 开始 I2C 总线扫描 <<<");
    int devices_found = 0;
    for (int addr = 1; addr < 127; addr++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK)
        {
            DEBUG_SENSOR_I2C_SCAN_LOG("发现 I2C 设备: 0x%02X", addr);
            devices_found++;
            if (addr == 0x0D) DEBUG_SENSOR_I2C_SCAN_LOG("  -> 可能是 QMC5883L");
            if (addr == 0x63) DEBUG_SENSOR_I2C_SCAN_LOG("  -> 可能是 ICP-20100 (ADO=GND)");
            if (addr == 0x64) DEBUG_SENSOR_I2C_SCAN_LOG("  -> 可能是 ICP-20100 (ADO=High)");
            if (addr == 0x5C) DEBUG_SENSOR_I2C_SCAN_LOG("  -> 可能是 LPS22HH (SA0=GND)");
            if (addr == 0x5D) DEBUG_SENSOR_I2C_SCAN_LOG("  -> 可能是 LPS22HH (SA0=VCC)");
        }
    }
    if (devices_found == 0) ESP_LOGE(TAG, "未发现任何 I2C 设备！");
    DEBUG_SENSOR_I2C_SCAN_LOG(">>> 扫描结束，共 %d 个设备 <<<", devices_found);

    // 初始化磁力计
    if (mag.begin() != ESP_OK) ESP_LOGE(TAG, "Mag Init Failed");

    // =======================================================
    // 气压计自动探测逻辑 (LPS22HH > ICP20100)
    // =======================================================
    static LPS22HH lps_5c(&i2c_bus, 0x5C);
    if (lps_5c.begin() == ESP_OK)
    {
        baro_lps = &lps_5c;
        active_baro = BARO_LPS22HH;
        ESP_LOGI(TAG, "Using Barometer: LPS22HH (Addr: 0x5C)");
    }
    else
    {
        static LPS22HH lps_5d(&i2c_bus, 0x5D);
        if (lps_5d.begin() == ESP_OK)
        {
            baro_lps = &lps_5d;
            active_baro = BARO_LPS22HH;
            ESP_LOGI(TAG, "Using Barometer: LPS22HH (Addr: 0x5D)");
        }
        else
        {
            if (baro_icp.begin() == ESP_OK)
            {
                active_baro = BARO_ICP20100;
                ESP_LOGI(TAG, "Using Barometer: ICP-20100");
            }
            else
            {
                active_baro = BARO_NONE;
                ESP_LOGE(TAG, "NO VALID BAROMETER FOUND!");
            }
        }
    }

    // 3. 开启 IMU 中断
    sem_imu_drdy = xSemaphoreCreateBinary();
    imu.enableInterrupt(PIN_IMU_INT, imu_isr_handler);

    ESP_LOGI(TAG, "All Sensors Initialized. Loop Start.");

    while (true)
    {
        // 等待 IMU 中断 (200Hz)
        if (xSemaphoreTake(sem_imu_drdy, pdMS_TO_TICKS(20)) == pdTRUE)
        {
            // --- A. 读取 IMU 原始数据 ---
            float ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
            imu.getAccel(&ax_raw, &ay_raw, &az_raw);
            imu.getGyro(&gx_raw, &gy_raw, &gz_raw);
            // =========================================================
            // [坐标系映射] SENSOR (Y前, X右, Z上) -> BODY FRD (X前, Y右, Z下)
            // =========================================================

            // 1. 机体 X (前) = 传感器 Y (前)
            float ax = ay_raw;
            float gx = gy_raw;

            // 2. 机体 Y (右) = 传感器 X (右)
            // 两个都向右，所以直接赋值，不需要负号
            float ay = ax_raw;
            float gy = gx_raw;

            // 3. 机体 Z (下) = -传感器 Z (上)
            // 必须取反，符合 FRD 右手系
            float az = -az_raw;
            float gz = -gz_raw;

            // =========================================================

            // 4. [关键] 应用校准 (在机体坐标系下进行)
            imu_data.ax = (ax - accel_offset_x) * accel_scale_x;
            imu_data.ay = (ay - accel_offset_y) * accel_scale_y;
            imu_data.az = (az - accel_offset_z) * accel_scale_z;

            imu_data.gx = gx - gyro_offset_x;
            imu_data.gy = gy - gyro_offset_y;
            imu_data.gz = gz - gyro_offset_z;

            imu_data.timestamp_us = esp_timer_get_time();
            bus.imu.publish(imu_data);

            // --- B. 分频读取磁力计 (100Hz -> 2分频) ---
            if (tick_counter % 2 == 0)
            {
                if (mag.isDataReady())
                {
                    float raw_x, raw_y, raw_z;
                    float mag_body_x, mag_body_y, mag_body_z;
                    mag.readData(&raw_x, &raw_y, &raw_z);

                    // 1. 坐标映射 FRD
                    // 与 IMU 保持一致
                    mag_body_x = raw_y;   // Y -> X
                    mag_body_y = raw_x;   // X -> Y
                    mag_body_z = -raw_z;  // Z -> -Z

                    // 2. 应用校准
                    mag_data.x = (mag_body_x - mag_offset_x) * mag_scale_x;
                    mag_data.y = (mag_body_y - mag_offset_y) * mag_scale_y;
                    mag_data.z = (mag_body_z - mag_offset_z) * mag_scale_z;
                    mag_data.timestamp_us = esp_timer_get_time();

                    bus.mag.publish(mag_data);
                }
            }

            // --- C. 分频读取气压计 (50Hz -> 4分频) ---
            if (tick_counter % 4 == 0)
            {
                esp_err_t ret = ESP_FAIL;
                if (active_baro == BARO_LPS22HH && baro_lps != nullptr)
                    ret = baro_lps->readData(&baro_data.pressure, &baro_data.temperature);
                else if (active_baro == BARO_ICP20100)
                    ret = baro_icp.readData(&baro_data.pressure, &baro_data.temperature);

                if (ret == ESP_OK) bus.baro.publish(baro_data);
            }

            // 调试打印 (由 DEBUG_SENSOR_PRINT_INTERVAL 控制频率)
            if (tick_counter % DEBUG_SENSOR_PRINT_INTERVAL == 0)
            {
                float alt = calculate_altitude(baro_data.pressure);
                DEBUG_SENSOR_IMU_LOG("IMU(FRD): ax=%.2f ay=%.2f az=%.2f",
                    imu_data.ax, imu_data.ay, imu_data.az);
                DEBUG_SENSOR_MAG_LOG("Mag(FRD): x=%.2f y=%.2f z=%.2f",
                    mag_data.x, mag_data.y, mag_data.z);
                DEBUG_SENSOR_BARO_LOG("Baro: pressure=%.2fkPa temp=%.1f°C alt=%.1fm",
                    baro_data.pressure, baro_data.temperature, alt);
            }

            tick_counter++;
        }
    }
}

void start_sensor_task()
{
    xTaskCreatePinnedToCore(task_sensor_entry, "Sensor", 4096, NULL, 5, NULL, 1);
}