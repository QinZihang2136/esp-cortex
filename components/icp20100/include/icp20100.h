#pragma once
#include "i2c_bus.h"

class ICP20100
{
public:
    /**
     * @brief 构造函数
     * @param bus I2CBus 指针
     * @param addr 设备地址 (默认 0x63) [cite: 3565]
     */
    ICP20100(I2CBus* bus, uint8_t addr = 0x64);

    /**
     * @brief 初始化
     * 执行复杂的 OTP 启动序列，并配置为 Mode 2 (Low Noise)
     */
    esp_err_t begin();

    /**
     * @brief 读取压力和温度
     * @param pressure 输出压力 (kPa)
     * @param temperature 输出温度 (°C)
     */
    esp_err_t readData(float* pressure, float* temperature);

private:
    I2CBus* _bus;
    uint8_t _addr;

    // 内部辅助函数
    esp_err_t writeReg(uint8_t reg, uint8_t val);
    esp_err_t readReg(uint8_t reg, uint8_t* val);
    esp_err_t readRegs(uint8_t reg, uint8_t* val, size_t len);

    // OTP 操作函数
    esp_err_t otpRead(uint8_t otp_addr, uint8_t* data);
    esp_err_t bootSequence();
};