#include "icp20100.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "ICP20100";

// --- 寄存器地址 ---
#define REG_TRIM1_MSB       0x05
#define REG_TRIM2_LSB       0x06
#define REG_TRIM2_MSB       0x07
#define REG_DEVICE_ID       0x0C // Device ID
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

esp_err_t ICP20100::begin()
{
    // --- 步骤 1: I2C 总线激活 (Dummy Write) ---
    // Datasheet 4.1: "After reset... needs 10 clock cycles... no communication is possible before this"
    // 我们向一个只读寄存器(0x0C)写一次 0x00，目的是为了产生时钟信号唤醒它
    // 忽略这次操作的返回值，因为可能会 NACK
    writeReg(REG_DEVICE_ID, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10)); // 等它醒来

    // --- 步骤 2: 软复位 (Soft Reset) ---
    // 为了确保状态干净，我们先让它进入 Standby 模式
    // Mode Select (0xC0) = 0x00 (Standby)
    if (writeReg(REG_MODE_SELECT, 0x00) != ESP_OK)
    {
        ESP_LOGW(TAG, "Bus not ready yet...");
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // 清空 FIFO
    writeReg(REG_FIFO_FILL, 0x80); // Flush FIFO

    // 屏蔽所有中断
    writeReg(REG_INTERRUPT_MASK, 0xFF);
    writeReg(REG_FIFO_CONFIG, 0x00);

    // --- 步骤 3: 检查 ID ---
    uint8_t id = 0;
    // 尝试读取 3 次，防止刚上电读失败
    for (int i = 0; i < 3; i++)
    {
        if (readReg(REG_DEVICE_ID, &id) == ESP_OK)
        {
            if (id == 0x63) break; // 读对了就跳出
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (id != 0x63)
    {
        // 如果这里打印 0xFF，说明总线通了但芯片没回数据；0x00 说明线没通
        ESP_LOGE(TAG, "ID Error: 0x%02X (Expected 0x63)", id);
        return ESP_FAIL;
    }

    // --- 步骤 4: 执行 OTP 启动序列 (加载校准参数) ---
    // 这是一个必须执行的复杂流程，否则读出来的压力值是错的
    if (bootSequence() != ESP_OK)
    {
        ESP_LOGE(TAG, "OTP Boot Failed");
        return ESP_FAIL;
    }

    // --- 步骤 5: 配置工作模式 ---
    // 设置为 Mode 2: Low Noise (BW=10Hz, ODR=40Hz)
    // 配合我们 25Hz (200Hz/8) 的读取策略绰绰有余
    // Bit 3 (MEAS_MODE): 1 = Continuous
    // Bit 2 (POWER_MODE): 0 = Normal
    // Bit 7:5 (MEAS_CONFIG): 010 = Mode 2
    // Value = 0b01001000 = 0x48
    writeReg(REG_MODE_SELECT, 0x48);

    ESP_LOGI(TAG, "Init Success (Mode 2: 40Hz)");
    return ESP_OK;
}

esp_err_t ICP20100::readData(float* pressure, float* temperature)
{
    uint8_t buf[9];
    // 一次性读取 6 字节 (Pressure High/Mid/Low + Temp High/Mid/Low)
    // 0xFA 是 FIFO_DATA 或 直接数据寄存器基地址
    if (readRegs(REG_PRESS_DATA_0, buf, 6) != ESP_OK) return ESP_FAIL;

    // 数据组装 (20-bit 有效，高位对齐)
    int32_t raw_p = (buf[0] << 16) | (buf[1] << 8) | buf[2];
    int32_t raw_t = (buf[3] << 16) | (buf[4] << 8) | buf[5];

    // 转换公式 (参考 Datasheet 6.7)
    // P = (P_out / 2^17) * 40kPa + 70kPa
    // 原始数据是 2's complement，但在 30-110kPa 范围内通常是正数
    *pressure = ((float)raw_p / 131072.0f) * 40.0f + 70.0f;

    // T = (T_out / 2^18) * 65C + 25C
    *temperature = ((float)raw_t / 262144.0f) * 65.0f + 25.0f;

    return ESP_OK;
}

// === OTP 启动序列 (参考 PX4 Driver & Datasheet Section 6.5) ===
esp_err_t ICP20100::bootSequence()
{
    uint8_t val;
    // 1. 检查是否已经 Boot 过
    readReg(REG_OTP_STATUS2, &val);
    if (val & 0x01)
    {
        ESP_LOGI(TAG, "Already Booted");
        return ESP_OK;
    }

    // 2. 进入 Power Mode (激活 OTP 电源域)
    writeReg(REG_MODE_SELECT, 0x04); // Power Mode = 1
    vTaskDelay(pdMS_TO_TICKS(5));

    // 3. 解锁主寄存器
    writeReg(REG_MASTER_LOCK, 0x1F);

    // 4. 开启 OTP
    writeReg(REG_OTP_CONFIG1, 0x03); // OTP_ENABLE=1, WRITE_SWITCH=1
    vTaskDelay(pdMS_TO_TICKS(1));

    // 5. 切换 Reset 引脚 (Toggle Reset)
    writeReg(REG_OTP_DBG2, 0x80);
    vTaskDelay(pdMS_TO_TICKS(1));
    writeReg(REG_OTP_DBG2, 0x00);
    vTaskDelay(pdMS_TO_TICKS(1));

    // 6. 配置冗余读取参数 (Redundant Read)
    writeReg(REG_OTP_MRA_LSB, 0x04);
    writeReg(REG_OTP_MRA_MSB, 0x04);
    writeReg(REG_OTP_MRB_LSB, 0x21);
    writeReg(REG_OTP_MRB_MSB, 0x20);
    writeReg(REG_OTP_MR_LSB, 0x10);
    writeReg(REG_OTP_MR_MSB, 0x80);

    // 7. 读取并回写 Offset (Addr 0xF8)
    uint8_t offset;
    otpRead(0xF8, &offset);
    writeReg(REG_TRIM1_MSB, offset & 0x3F);

    // 8. 读取并回写 Gain (Addr 0xF9)
    uint8_t gain;
    otpRead(0xF9, &gain);
    writeReg(REG_TRIM2_MSB, (gain & 0x07) << 4);

    // 9. 读取并回写 HFosc (Addr 0xFA)
    uint8_t hfosc;
    otpRead(0xFA, &hfosc);
    writeReg(REG_TRIM2_LSB, hfosc & 0x7F);

    // 10. 关闭 OTP
    writeReg(REG_OTP_CONFIG1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(1));

    // 11. 重新上锁
    writeReg(REG_MASTER_LOCK, 0x00);

    // 12. 回到 Standby
    writeReg(REG_MODE_SELECT, 0x00);

    // 13. 标记为已启动 (可选，有些版本需要)
    // writeReg(REG_OTP_STATUS2, 0x01); 

    return ESP_OK;
}

esp_err_t ICP20100::otpRead(uint8_t otp_addr, uint8_t* data)
{
    writeReg(REG_OTP_ADDRESS, otp_addr);
    writeReg(REG_OTP_COMMAND, 0x10); // Read Command

    // 等待 BUSY 位清零
    for (int i = 0; i < 10; i++)
    {
        uint8_t status;
        readReg(REG_OTP_STATUS, &status);
        if ((status & 0x01) == 0) break;
        vTaskDelay(1);
    }
    return readReg(REG_OTP_RDATA, data);
}

// --- 基础读写封装 ---
esp_err_t ICP20100::writeReg(uint8_t reg, uint8_t val)
{
    return _bus->writeRegister(_addr, reg, val);
}
esp_err_t ICP20100::readReg(uint8_t reg, uint8_t* val)
{
    return _bus->readRegister(_addr, reg, val);
}
esp_err_t ICP20100::readRegs(uint8_t reg, uint8_t* val, size_t len)
{
    return _bus->readRegisters(_addr, reg, val, len);
}