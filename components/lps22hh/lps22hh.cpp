#include "lps22hh.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "LPS22HH";

// --- 寄存器定义 (参考 Datasheet Page 37) ---
#define REG_WHO_AM_I        0x0F
#define REG_CTRL_REG1       0x10
#define REG_CTRL_REG2       0x11
#define REG_STATUS          0x27
#define REG_PRESS_OUT_XL    0x28 
#define REG_PRESS_OUT_L     0x29
#define REG_PRESS_OUT_H     0x2A
#define REG_TEMP_OUT_L      0x2B 
#define REG_TEMP_OUT_H      0x2C

#define WHO_AM_I_VAL        0xB3 

LPS22HH::LPS22HH(I2CBus* bus, uint8_t addr) : _bus(bus), _addr(addr) {}

esp_err_t LPS22HH::begin()
{
    uint8_t id = 0;

    // 1. 检查设备 ID
    if (readReg(REG_WHO_AM_I, &id) != ESP_OK)
    {
        ESP_LOGE(TAG, "Read ID Failed!");
        return ESP_FAIL;
    }

    if (id != WHO_AM_I_VAL)
    {
        ESP_LOGE(TAG, "ID Mismatch! Exp: 0xB3, Got: 0x%02X", id);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Found LPS22HH (Addr: 0x%02X, ID: 0x%02X)", _addr, id);

    // 2. 软复位
    writeReg(REG_CTRL_REG2, 0x04);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 3. 配置 CTRL_REG2
    // Bit 4 (IF_ADD_INC): 1
    // Bit 1 (LOW_NOISE_EN): 1 (开启低噪声，前提是 ODR <= 75Hz)
    writeReg(REG_CTRL_REG2, 0x12);

    // 4. 配置 CTRL_REG1 (关键修改)
    // Bit 6-4 (ODR): 101 (75Hz) - 这是支持低噪声的最高频率 [cite: 932, 983]
    // Bit 3 (EN_LPFP): 1 (开启数字低通滤波)
    // Bit 1 (BDU): 1 (开启块数据更新保护)
    // Binary: 0101 1010 -> 0x5A
    esp_err_t ret = writeReg(REG_CTRL_REG1, 0x5A);

    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Init Success (75Hz, LowNoise: ON, BDU: ON)");
    }
    return ret;
}

esp_err_t LPS22HH::readData(float* pressure, float* temperature)
{
    uint8_t status = 0;
    if (readReg(REG_STATUS, &status) != ESP_OK) return ESP_FAIL;

    if ((status & 0x03) != 0x03)
    {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buf[5];
    if (readRegs(REG_PRESS_OUT_XL, buf, 5) != ESP_OK) return ESP_FAIL;

    int32_t raw_p = (int32_t)((buf[2] << 16) | (buf[1] << 8) | buf[0]);
    if (raw_p & 0x800000) raw_p |= 0xFF000000;

    // 转换为 kPa
    *pressure = (float)raw_p / 40960.0f;

    int16_t raw_t = (int16_t)((buf[4] << 8) | buf[3]);
    *temperature = (float)raw_t / 100.0f;

    return ESP_OK;
}
esp_err_t LPS22HH::writeReg(uint8_t reg, uint8_t val) { return _bus->writeRegister(_addr, reg, val); }
esp_err_t LPS22HH::readReg(uint8_t reg, uint8_t* val) { return _bus->readRegister(_addr, reg, val); }
esp_err_t LPS22HH::readRegs(uint8_t reg, uint8_t* val, size_t len) { return _bus->readRegisters(_addr, reg, val, len); }