#include "qmc5883l.h"
#include "esp_log.h"

static const char* TAG = "QMC5883L";

// --- 寄存器定义 (参考 Datasheet [cite: 2085, 2094, 2144]) ---
#define REG_XOUT_L  0x00
#define REG_STATUS  0x06
#define REG_CTRL1   0x09
#define REG_CTRL2   0x0A
#define REG_FBR     0x0B
#define REG_CHIP_ID 0x0D 

QMC5883L::QMC5883L(I2CBus* bus, uint8_t addr) : _bus(bus), _addr(addr)
{
    // 初始化校准参数，避免除以0
    _cal.offset[0] = _cal.offset[1] = _cal.offset[2] = 0.0f;
    _cal.scale[0] = _cal.scale[1] = _cal.scale[2] = 1.0f;
    // 给 _range_scale 一个默认值，防止未调用 begin 直接读取出错
    _range_scale = 1.0f / 3000.0f;
}

esp_err_t QMC5883L::begin()
{
    // 0. 启动前先小睡一下，避开 I2C 扫描的锋芒
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t id = 0;
    esp_err_t err = ESP_FAIL;

    // --- 1. 检查 ID (增加重试机制) ---
    // 尝试读取 5 次，只要有一次成功就行
    for (int i = 0; i < 5; i++)
    {
        err = _bus->readRegister(_addr, REG_CHIP_ID, &id);
        if (err == ESP_OK)
        {
            break; // 读到了！跳出循环
        }
        // 如果失败，打印一个调试信息（可选），然后等 10ms 再试
        // ESP_LOGW(TAG, "ID Read Attempt %d failed, retrying...", i + 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 如果 5 次全挂了，那才是真的挂了
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C Bus Read Failed after 5 retries! Check wiring and address (0x%02X)", _addr);
        return ESP_FAIL;
    }

    // --- ID 逻辑判断 (保持不变) ---
    if (id == 0xFF)
    {
        ESP_LOGI(TAG, "Found QMC5883L (ID: 0xFF)");
    }
    else if (id == 0xD0)
    {
        ESP_LOGW(TAG, "Found DA5883 (ID: 0xD0) - Compatible clone. Proceeding.");
    }
    else
    {
        ESP_LOGW(TAG, "Unknown Chip ID: 0x%02X (Expected 0xFF or 0xD0). Forcing init...", id);
    }

    // --- 2. 软复位 ---
    writeReg(REG_CTRL2, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));
    writeReg(REG_CTRL2, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));

    // --- 3. 配置 FBR ---
    writeReg(REG_FBR, 0x01);

    // --- 4. 配置控制寄存器 1 ---
    // OSR=512, RNG=8G, ODR=100Hz, Mode=Continuous
    writeReg(REG_CTRL1, 0x1D);

    // --- 5. 设置数据转换比例 ---
    _range_scale = 1.0f / 3000.0f; // 8G 量程

    ESP_LOGI(TAG, "Init Success. Scale set for 8G Range.");
    return ESP_OK;
}
bool QMC5883L::isDataReady()
{
    uint8_t status;
    if (readReg(REG_STATUS, &status) == ESP_OK)
    {
        // Bit 0 是 DRDY (Data Ready) [cite: 2107]
        return (status & 0x01);
    }
    return false;
}

esp_err_t QMC5883L::readData(float* x, float* y, float* z)
{
    uint8_t buf[6];
    // 必须一次性读取 6 个字节 (X_L, X_H, Y_L, Y_H, Z_L, Z_H) [cite: 2085]
    if (_bus->readRegisters(_addr, REG_XOUT_L, buf, 6) != ESP_OK)
    {
        // I2C 读取失败
        return ESP_FAIL;
    }

    // 转换为 16 位有符号整数 (Little Endian)
    int16_t raw_x = (int16_t)(buf[0] | (buf[1] << 8));
    int16_t raw_y = (int16_t)(buf[2] | (buf[3] << 8));
    int16_t raw_z = (int16_t)(buf[4] | (buf[5] << 8));

    // 1. 转换为物理单位 (Gauss)
    // 如果 _range_scale 未被初始化，这里结果会是 0
    float gx = (float)raw_x * _range_scale;
    float gy = (float)raw_y * _range_scale;
    float gz = (float)raw_z * _range_scale;

    // 2. 应用软磁/硬磁校准
    *x = (gx - _cal.offset[0]) * _cal.scale[0];
    *y = (gy - _cal.offset[1]) * _cal.scale[1];
    *z = (gz - _cal.offset[2]) * _cal.scale[2];

    return ESP_OK;
}

void QMC5883L::setCalibration(const CalibrationData& cal)
{
    _cal = cal;
}

esp_err_t QMC5883L::writeReg(uint8_t reg, uint8_t val)
{
    return _bus->writeRegister(_addr, reg, val);
}
esp_err_t QMC5883L::readReg(uint8_t reg, uint8_t* val)
{
    return _bus->readRegister(_addr, reg, val);
}