#include "icm42688.h"
#include "esp_log.h"

static const char* TAG = "ICM42688";

// --- 寄存器地址映射 (参考 Datasheet Section 14) ---
#define REG_INT_CONFIG      0x14  // 中断引脚配置 [cite: 1572]
#define REG_INT_STATUS      0x2D  // 中断状态查询 [cite: 1712]
#define REG_PWR_MGMT0       0x4E  // 电源管理 [cite: 1838]
#define REG_INT_CONFIG1     0x64  // 中断时序配置 [cite: 1960]
#define REG_INT_SOURCE0     0x65  // 中断源路由 [cite: 1967]
#define REG_WHO_AM_I        0x75  // 设备ID [cite: 2025]
#define REG_ACCEL_DATA_X1   0x1F  // 加速度数据起始地址 [cite: 1609]
#define REG_GYRO_DATA_X1    0x25  // 陀螺仪数据起始地址 [cite: 1655]
// 【新增】频率配置寄存器
#define REG_GYRO_CONFIG0    0x4F  // [cite: 4031]
#define REG_ACCEL_CONFIG0   0x50  // [cite: 4041]
ICM42688::ICM42688(SPIBus* bus, int cs_pin) : _bus(bus), _cs_pin(cs_pin) {}

esp_err_t ICM42688::begin() {
    // 1. 将设备添加到 SPI 总线
    // 频率设为 10MHz (ICM42688 最大支持 24MHz)
    esp_err_t ret = _bus->addDevice(0, 10 * 1000 * 1000, _cs_pin, &_dev_handle);
    if (ret != ESP_OK) return ret;

    // 2. 读取 WHO_AM_I 寄存器验证芯片 ID
    uint8_t who;
    readReg(REG_WHO_AM_I, &who, 1);
    if (who != 0x47) { // Datasheet 规定默认 ID 为 0x47
        ESP_LOGE(TAG, "ID 错误: 0x%02X (应为 0x47)", who);
        return ESP_OK;
    }

    // 3. 电源管理配置 (REG_PWR_MGMT0)
    // Bit 3:2 (GYRO_MODE): 11 = Low Noise Mode (开启陀螺仪) [cite: 615]
    // Bit 1:0 (ACCEL_MODE): 11 = Low Noise Mode (开启加速度计) [cite: 615]
    // Value = 0b00001111 = 0x0F
    writeReg(REG_PWR_MGMT0, 0x0F); 
    writeReg(REG_GYRO_CONFIG0, 0x07);  // 设为 200Hz
    writeReg(REG_ACCEL_CONFIG0, 0x07); // 设为 200Hz

    // 4. 计算物理单位转换系数
    // 默认量程 (Reset Value): Accel ±16g, Gyro ±2000dps
    // Accel: 16g / 32768 * 9.81 (m/s^2)
    _accel_scale = 16.0f / 32768.0f * 9.81f; 
    // Gyro: 2000dps / 32768 * (PI / 180) (rad/s)
    _gyro_scale = 2000.0f / 32768.0f * (3.14159f / 180.0f); 

    ESP_LOGI(TAG, "初始化成功 (轮询模式就绪)");
    return ESP_OK;
}

bool ICM42688::isDataReady() {
    uint8_t status;
    // 读取 INT_STATUS 寄存器
    if (readReg(REG_INT_STATUS, &status, 1) == ESP_OK) {
        // Bit 3 是 DATA_RDY_INT [cite: 1712]
        // 如果为 1，说明数据寄存器已更新
        return (status & 0x08) != 0;
    }
    return false;
}

esp_err_t ICM42688::enableInterrupt(int gpio_pin, gpio_isr_t isr_handler) {
    // 1. 配置 ESP32 端的 GPIO
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE; // 上升沿触发 (因为我们后面配置传感器为 Active High)
    io_conf.pin_bit_mask = (1ULL << gpio_pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE; // 默认下拉，防止悬空干扰
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // 2. 注册 ESP32 的中断服务
    gpio_install_isr_service(0); // 允许失败(如果已安装)
    gpio_isr_handler_add((gpio_num_t)gpio_pin, isr_handler, NULL);

    // 3. 配置传感器端的寄存器来产生信号
    return configure_int_registers();
}

esp_err_t ICM42688::configure_int_registers() {
    esp_err_t ret;
    // A. INT_CONFIG (0x14): 配置引脚电气特性 [cite: 1572]
    // Bit 1 (DRIVE): 1 = Push-Pull (推挽输出，信号更强)
    // Bit 0 (POLARITY): 1 = Active High (高电平有效)
    ret = writeReg(REG_INT_CONFIG, 0x03); 
    if (ret != ESP_OK) return ret;

    // B. INT_CONFIG1 (0x64): 配置脉冲宽度 [cite: 1960]
    // 警告：Bit 4 (INT_ASYNC_RESET) 默认是 1，必须手动写 0！[cite: 1487]
    // 否则在脉冲模式下中断可能无法清除，导致只触发一次就死锁。
    ret = writeReg(REG_INT_CONFIG1, 0x00); 
    if (ret != ESP_OK) return ret;

    // C. INT_SOURCE0 (0x65): 开启数据就绪中断 [cite: 1967]
    // Bit 3 (UI_DRDY_INT1_EN): 1 = 将 Data Ready 信号路由到 INT1 引脚
    ret = writeReg(REG_INT_SOURCE0, 0x08);
    
    if (ret == ESP_OK) ESP_LOGI(TAG, "硬件中断已开启 (Push-Pull, Active High)");
    return ret;
}

void ICM42688::getAccel(float* x, float* y, float* z) {
    uint8_t buf[6];
    // 连读 6 个字节 (X_H, X_L, Y_H, Y_L, Z_H, Z_L)
    readReg(REG_ACCEL_DATA_X1, buf, 6);
    
    // 拼接高低位 (Big Endian: 高位在前) [cite: 885]
    int16_t raw_x = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t raw_y = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t raw_z = (int16_t)((buf[4] << 8) | buf[5]);
    
    // 转换为物理单位
    *x = raw_x * _accel_scale;
    *y = raw_y * _accel_scale;
    *z = raw_z * _accel_scale;
}

void ICM42688::getGyro(float* x, float* y, float* z) {
    uint8_t buf[6];
    readReg(REG_GYRO_DATA_X1, buf, 6);
    
    int16_t raw_x = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t raw_y = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t raw_z = (int16_t)((buf[4] << 8) | buf[5]);

    *x = raw_x * _gyro_scale;
    *y = raw_y * _gyro_scale;
    *z = raw_z * _gyro_scale;
}

// --- 内部使用的读写封装 ---
esp_err_t ICM42688::writeReg(uint8_t reg, uint8_t data) {
    return _bus->writeByte(_dev_handle, reg, data);
}
esp_err_t ICM42688::readReg(uint8_t reg, uint8_t* data, size_t len) {
    return _bus->readBytes(_dev_handle, reg, len, data);
}