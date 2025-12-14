#pragma once
#include "i2c_bus.h"

class LPS22HH
{
public:
    /**
     * @brief 构造函数
     * @param bus I2CBus 实例指针
     * @param addr 设备地址 (默认为 0x5C, SA0=GND)
     */
    LPS22HH(I2CBus* bus, uint8_t addr = 0x5C);

    /**
     * @brief 初始化传感器
     * 执行软复位、检查ID、开启 BDU/低噪声模式/低通滤波，设置 ODR=50Hz
     */
    esp_err_t begin();

    /**
     * @brief 读取数据
     * @param pressure 输出压力 (kPa) - 注意单位已转换为 kPa 以匹配系统标准
     * @param temperature 输出温度 (°C)
     */
    esp_err_t readData(float* pressure, float* temperature);

private:
    I2CBus* _bus;
    uint8_t _addr;

    // 寄存器辅助函数
    esp_err_t writeReg(uint8_t reg, uint8_t val);
    esp_err_t readReg(uint8_t reg, uint8_t* val);
    esp_err_t readRegs(uint8_t reg, uint8_t* val, size_t len);
};