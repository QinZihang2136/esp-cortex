#pragma once
#include "spi_bus.h"
#include "driver/gpio.h"

class ICM42688 {
public:
    /**
     * @brief 构造函数
     * @param bus 指向已初始化的 SPIBus 对象的指针
     * @param cs_pin 传感器连接的片选(CS)引脚号
     */
    ICM42688(SPIBus* bus, int cs_pin);

    /**
     * @brief 传感器基础初始化
     * 检查 ID，配置电源模式，设置量程。
     * 无论是轮询还是中断模式，都必须先调用这个。
     */
    esp_err_t begin();

    /**
     * @brief [可选] 开启硬件中断模式
     * 配置 ESP32 的 GPIO 接收中断，并配置传感器在数据就绪时拉高引脚。
     * @param gpio_pin ESP32 连接 INT1 的引脚号
     * @param isr_handler 中断回调函数 (ISR)
     */
    esp_err_t enableInterrupt(int gpio_pin, gpio_isr_t isr_handler);

    /**
     * @brief [可选] 查询数据是否就绪 (用于轮询模式)
     * 通过 SPI 读取 INT_STATUS 寄存器。
     * @return true: 有新数据, false: 无新数据
     */
    bool isDataReady();

    // 读取加速度 (单位: m/s^2)
    void getAccel(float* x, float* y, float* z);
    // 读取角速度 (单位: rad/s)
    void getGyro(float* x, float* y, float* z);

private:
    SPIBus* _bus;
    spi_device_handle_t _dev_handle = nullptr;
    int _cs_pin;
    
    // 灵敏度比例系数 (Raw -> 物理单位)
    float _accel_scale;
    float _gyro_scale;

    // 内部辅助函数
    esp_err_t writeReg(uint8_t reg, uint8_t data);
    esp_err_t readReg(uint8_t reg, uint8_t* data, size_t len);
    esp_err_t configure_int_registers();
};