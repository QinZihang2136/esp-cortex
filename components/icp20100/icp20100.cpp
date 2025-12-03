#include "icp20100.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "ICP20100";

// --- 寄存器地址 ---
#define REG_TRIM1_MSB       0x05
#define REG_TRIM2_LSB       0x06
#define REG_TRIM2_MSB       0x07
#define REG_DEVICE_ID       0x0C 
#define REG_OTP_CONFIG1     0xAC
#define REG_OTP_MR_LSB      0xAD
#define REG_OTP_MR_MSB      0xAE
#define REG_OTP_MRA_LSB     0xAF
#define REG_OTP_MRA_MSB     0xB0
#define REG_OTP_MRB_LSB     0xB1
#define REG_OTP_MRB_MSB     0xB2
#define REG_OTP_ADDRESS     0xB5
#define REG_OTP_COMMAND     0xB6
#define REG_OTP_RDATA       0xB8
#define REG_OTP_STATUS      0xB9
#define REG_OTP_DBG2        0xBC
#define REG_MASTER_LOCK     0xBE
#define REG_OTP_STATUS2     0xBF
#define REG_MODE_SELECT     0xC0
#define REG_INTERRUPT_MASK  0xC2
#define REG_FIFO_CONFIG     0xC3
#define REG_FIFO_FILL       0xC4
#define REG_PRESS_DATA_0    0xFA

ICP20100::ICP20100(I2CBus* bus, uint8_t addr) : _bus(bus), _addr(addr) {}

// 辅助函数：确保至少延时 1 个 tick
static inline void delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms) > 0 ? pdMS_TO_TICKS(ms) : 1);
}

esp_err_t ICP20100::begin()
{
    // ============================================================
    // 1. 强力唤醒序列 (根据 Datasheet 4.1)
    // ============================================================
    // 现象：I2C 扫描能扫到 0x7E (I3C入口) 但扫不到 0x63。
    // 原因：I2C 接口处于休眠状态，需要特定的 Dummy Write 激活。

    ESP_LOGI(TAG, "Waking up ICP-20100...");

    // 尝试发送 "Dummy Write to 0xEE" 3次
    // 注意：这里可能会返回失败(NACK)，这是正常的，我们的目的是产生时钟信号
    for (int i = 0; i < 3; i++)
    {
        // 向寄存器 0xEE 写 0x00 (数据手册第17页建议)
        _bus->writeRegister(_addr, 0xEE, 0x00);
        delay_ms(5);
    }

    // --- 2. 软复位 (确保状态机复位) ---
    // 写入 Standby 模式 (0x00)
    if (writeReg(REG_MODE_SELECT, 0x00) != ESP_OK)
    {
        // 如果这里还失败，说明唤醒没成功，再试一次强力唤醒
        ESP_LOGW(TAG, "Soft Reset NACK, retrying wakeup...");
        _bus->writeRegister(_addr, 0xEE, 0x00);
        delay_ms(10);
        writeReg(REG_MODE_SELECT, 0x00);
    }
    delay_ms(5);

    // --- 3. 检查 ID (带重试) ---
    uint8_t id = 0;
    bool id_ok = false;
    for (int i = 0; i < 5; i++)
    {
        if (readReg(REG_DEVICE_ID, &id) == ESP_OK)
        {
            if (id == 0x63)
            {
                id_ok = true;
                break;
            }
        }
        // 如果读不到，打印个点，不刷屏
        // printf("."); 
        delay_ms(10);
    }

    if (!id_ok)
    {
        // 只有 5 次全挂了才报错
        ESP_LOGE(TAG, "ID Read Failed after retries. Last ID: 0x%02X", id);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Device ID Found: 0x%02X", id);

    // --- 4. 智能 OTP 启动 (防止重复校准导致漂移) ---
    // 检查是否已经 Boot 过 (Status2 的 bit 0)
    uint8_t boot_status = 0;
    readReg(REG_OTP_STATUS2, &boot_status);

    // 如果 Status2 最低位是 1，说明已经初始化过了
    if (boot_status & 0x01)
    {
        ESP_LOGI(TAG, "Sensor already booted (Skipping OTP reload).");
    }
    else
    {
        // 未初始化，执行 OTP 序列
        if (bootSequence() != ESP_OK)
        {
            ESP_LOGE(TAG, "OTP Boot Failed");
            return ESP_FAIL;
        }
    }

    // --- 5. 清空 FIFO ---
    writeReg(REG_FIFO_FILL, 0x80);
    delay_ms(10);

    // --- 6. 配置工作模式 ---
    // Mode 2: Low Noise (BW=10Hz, ODR=40Hz)
    writeReg(REG_MODE_SELECT, 0x48);

    // --- 7. 等待滤波器稳定 (约 500ms) ---
    // 丢弃前 20 个不稳定的样本
    for (int i = 0; i < 20; i++)
    {
        delay_ms(25);
        float dummy_p, dummy_t;
        readData(&dummy_p, &dummy_t);
    }
    // 再次清空，准备开始正式工作
    writeReg(REG_FIFO_FILL, 0x80);

    ESP_LOGI(TAG, "Init Success (Mode 2: 40Hz) - Ready");
    return ESP_OK;
}
esp_err_t ICP20100::readData(float* pressure, float* temperature)
{
    // --- 关键修复：读取前先检查 FIFO 是否有数据 ---
    uint8_t fifo_status = 0;
    if (readReg(REG_FIFO_FILL, &fifo_status) != ESP_OK) return ESP_FAIL;

    // Datasheet 13.24: FIFO_LEVEL is bits [4:0] [cite: 2019]
    // 0: Empty, 1: 1/16 full...
    uint8_t level = fifo_status & 0x1F;

    // 如果 FIFO 为空，直接返回，不要读！
    if (level == 0)
    {
        // 返回特定错误码，让上层知道这次没数据，不要更新显示
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buf[9];
    // 读取 6 字节 (Pressure High/Mid/Low + Temp High/Mid/Low)
    // 必须一次性 Burst Read，才能保证数据一致性 [cite: 1750]
    if (readRegs(REG_PRESS_DATA_0, buf, 6) != ESP_OK) return ESP_FAIL;

    // 数据组装 (20-bit 有效，放在 24-bit 容器里)
    int32_t raw_p = (int32_t)((buf[0] << 16) | (buf[1] << 8) | buf[2]);
    int32_t raw_t = (int32_t)((buf[3] << 16) | (buf[4] << 8) | buf[5]);

    // 符号扩展 (虽然气压通常为正，但这是一个有符号 24 位整数)
    if (raw_p & 0x800000) raw_p |= 0xFF000000;
    if (raw_t & 0x800000) raw_t |= 0xFF000000;

    // 转换公式 [cite: 1673, 1691]
    *pressure = ((float)raw_p / 131072.0f) * 40.0f + 70.0f;
    *temperature = ((float)raw_t / 262144.0f) * 65.0f + 25.0f;

    return ESP_OK;
}

// === OTP 启动序列 ===
esp_err_t ICP20100::bootSequence()
{
    ESP_LOGI(TAG, "Running OTP Boot Sequence...");

    // 1. Power Mode
    writeReg(REG_MODE_SELECT, 0x04);
    delay_ms(4); // Min 4ms required [cite: 1578]

    // 2. Unlock
    writeReg(REG_MASTER_LOCK, 0x1F);

    // 3. Enable OTP
    writeReg(REG_OTP_CONFIG1, 0x03);
    delay_ms(1);

    // 4. Toggle Reset
    writeReg(REG_OTP_DBG2, 0x80);
    delay_ms(1);
    writeReg(REG_OTP_DBG2, 0x00);
    delay_ms(1);

    // 5. Redundant Read Config
    writeReg(REG_OTP_MRA_LSB, 0x04);
    writeReg(REG_OTP_MRA_MSB, 0x04);
    writeReg(REG_OTP_MRB_LSB, 0x21);
    writeReg(REG_OTP_MRB_MSB, 0x20);
    writeReg(REG_OTP_MR_LSB, 0x10);
    writeReg(REG_OTP_MR_MSB, 0x80);

    // 6. Read/Write Offset (0xF8)
    uint8_t offset;
    if (otpRead(0xF8, &offset) != ESP_OK) return ESP_FAIL;
    writeReg(REG_TRIM1_MSB, offset & 0x3F);

    // 7. Read/Write Gain (0xF9)
    uint8_t gain;
    if (otpRead(0xF9, &gain) != ESP_OK) return ESP_FAIL;
    writeReg(REG_TRIM2_MSB, (gain & 0x07) << 4);

    // 8. Read/Write HFosc (0xFA)
    uint8_t hfosc;
    if (otpRead(0xFA, &hfosc) != ESP_OK) return ESP_FAIL;
    writeReg(REG_TRIM2_LSB, hfosc & 0x7F);

    // 9. Disable OTP
    writeReg(REG_OTP_CONFIG1, 0x00);
    delay_ms(1);

    // 10. Lock
    writeReg(REG_MASTER_LOCK, 0x00);

    // 11. Standby
    writeReg(REG_MODE_SELECT, 0x00);

    // 12. Mark as Booted [cite: 1653]
    // 这一步很重要，防止下次误判
    writeReg(REG_OTP_STATUS2, 0x01);

    return ESP_OK;
}

esp_err_t ICP20100::otpRead(uint8_t otp_addr, uint8_t* data)
{
    writeReg(REG_OTP_ADDRESS, otp_addr);
    writeReg(REG_OTP_COMMAND, 0x10); // Read Command

    for (int i = 0; i < 10; i++)
    {
        uint8_t status;
        readReg(REG_OTP_STATUS, &status);
        if ((status & 0x01) == 0) return readReg(REG_OTP_RDATA, data);
        delay_ms(1);
    }
    return ESP_FAIL;
}

// 基础读写函数保持不变...
esp_err_t ICP20100::writeReg(uint8_t reg, uint8_t val) { return _bus->writeRegister(_addr, reg, val); }
esp_err_t ICP20100::readReg(uint8_t reg, uint8_t* val) { return _bus->readRegister(_addr, reg, val); }
esp_err_t ICP20100::readRegs(uint8_t reg, uint8_t* val, size_t len) { return _bus->readRegisters(_addr, reg, val, len); }