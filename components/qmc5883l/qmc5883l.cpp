#include "qmc5883l.h"
#include "esp_log.h"

static const char* TAG = "QMC5883L";

// 寄存器定义
#define REG_XOUT_L 0x00
#define REG_STATUS 0x06
#define REG_CTRL1  0x09
#define REG_CTRL2  0x0A
#define REG_FBR    0x0B
#define REG_CHIP_ID 0x0D // Chip ID

QMC5883L::QMC5883L(I2CBus* bus, uint8_t addr) : _bus(bus), _addr(addr) {
    // 默认无校准
    _cal.offset[0] = _cal.offset[1] = _cal.offset[2] = 0.0f;
    _cal.scale[0] = _cal.scale[1] = _cal.scale[2] = 1.0f;
}

esp_err_t QMC5883L::begin() {
    // 1. 检查 ID (QMC5883L ID 是 0xFF)
    uint8_t id;
    _bus->readRegister(_addr, REG_CHIP_ID, &id);
    if (id != 0xFF) {
        ESP_LOGE(TAG, "ID Error: 0x%02X (Expected 0xFF)", id);
        return ESP_FAIL;
    }

    // 2. 软复位
    writeReg(REG_CTRL2, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));
    writeReg(REG_CTRL2, 0x00);

    // 3. 配置 FBR (推荐值)
    writeReg(REG_FBR, 0x01); 

    // 4. 配置控制寄存器 1 (关键！)
    // OSR=512 (00), Range=8G (01), ODR=100Hz (10), Mode=Continuous (01)
    // 00 | 01 | 10 | 01 = 0x1D
    // 8G Range => 3000 LSB/Gauss => Scale = 1/3000
    _range_scale = 1.0f / 3000.0f; 
    
    // 如果你想要 200Hz，ODR设为11 -> 0x1D | 0x0C = 0x1D? 不对
    // ODR 200Hz = 11b. 
    // 00(OSR) | 01(RNG) | 11(ODR) | 01(MODE) = 0x1D + 0x04 = 0x21 (假设 OSR=512)
    // 让我们用 100Hz (0x1D)，这是更稳妥的选择
    writeReg(REG_CTRL1, 0x1D); 

    ESP_LOGI(TAG, "Init Success (100Hz, 8G)");
    return ESP_OK;
}

bool QMC5883L::isDataReady() {
    uint8_t status;
    if (readReg(REG_STATUS, &status) == ESP_OK) {
        return (status & 0x01); // DRDY bit
    }
    return false;
}

esp_err_t QMC5883L::readData(float* x, float* y, float* z) {
    uint8_t buf[6];
    if (_bus->readRegisters(_addr, REG_XOUT_L, buf, 6) != ESP_OK) {
        return ESP_FAIL;
    }

    // QMC5883L 是 Little Endian
    int16_t raw_x = (int16_t)(buf[0] | (buf[1] << 8));
    int16_t raw_y = (int16_t)(buf[2] | (buf[3] << 8));
    int16_t raw_z = (int16_t)(buf[4] | (buf[5] << 8));

    // 1. 转换为物理单位 (Gauss)
    float gx = raw_x * _range_scale;
    float gy = raw_y * _range_scale;
    float gz = raw_z * _range_scale;

    // 2. 应用软磁/硬磁校准
    *x = (gx - _cal.offset[0]) * _cal.scale[0];
    *y = (gy - _cal.offset[1]) * _cal.scale[1];
    *z = (gz - _cal.offset[2]) * _cal.scale[2];

    return ESP_OK;
}

void QMC5883L::setCalibration(const CalibrationData& cal) {
    _cal = cal;
}

esp_err_t QMC5883L::writeReg(uint8_t reg, uint8_t val) {
    return _bus->writeRegister(_addr, reg, val);
}
esp_err_t QMC5883L::readReg(uint8_t reg, uint8_t* val) {
    return _bus->readRegister(_addr, reg, val);
}