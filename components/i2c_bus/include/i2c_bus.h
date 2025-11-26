#pragma once
#include "driver/i2c.h"
#include "esp_err.h"

class I2CBus {
public:
    /**
     * @brief 构造函数
     * @param port I2C 端口号 (I2C_NUM_0 或 I2C_NUM_1)
     */
    I2CBus(i2c_port_t port);
    ~I2CBus();

    /**
     * @brief 初始化 I2C 主机
     * @param sda_pin SDA 引脚
     * @param scl_pin SCL 引脚
     * @param clk_speed 时钟频率 (Hz)，通常 100000 或 400000
     */
    esp_err_t begin(int sda_pin, int scl_pin, uint32_t clk_speed = 400000);
    esp_err_t close();

    // --- 寄存器读写接口 ---
    esp_err_t writeRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
    esp_err_t readRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);
    esp_err_t readRegisters(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);

    // --- 提供底层接口访问 ---
    i2c_port_t getPort() const { return _port; }

private:
    i2c_port_t _port;
};