#pragma once
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

/**
 * @brief SPI 总线封装类
 * 负责管理 ESP32 的 SPI 主机外设，支持多设备挂载
 */
class SPIBus {
public:
    /**
     * @brief 构造函数
     * @param host SPI 主机号 (SPI2_HOST 或 SPI3_HOST)
     */
    SPIBus(spi_host_device_t host);
    ~SPIBus();
    
    /**
     * @brief 初始化 SPI 总线 (GPIO 映射)
     * @param mosi_pin 主出从入引脚
     * @param miso_pin 主入从出引脚
     * @param sclk_pin 时钟引脚
     * @return esp_err_t 初始化结果
     */
    esp_err_t begin(int mosi_pin, int miso_pin, int sclk_pin);
    esp_err_t close();

    /**
     * @brief 向总线挂载一个新设备
     * @param mode SPI 模式 (0-3), ICM42688 通常支持模式 0 or 3
     * @param clock_speed_hz 时钟频率 (例如 10MHz = 10000000)
     * @param cs_pin 片选引脚
     * @param handle [输出] 返回的设备句柄，后续读写都要用它
     */
    esp_err_t addDevice(uint8_t mode, uint32_t clock_speed_hz, int cs_pin, spi_device_handle_t *handle);
    esp_err_t removeDevice(spi_device_handle_t handle);
    
    // --- 底层读写接口 ---
    // writeBytes: 写寄存器 (自动处理地址位的写标志)
    esp_err_t writeBytes(spi_device_handle_t handle, uint8_t regAddr, size_t length, const uint8_t *data);
    // readBytes: 读寄存器 (自动处理地址位的读标志)
    esp_err_t readBytes(spi_device_handle_t handle, uint8_t regAddr, size_t length, uint8_t *data);
    
    // --- 便捷接口 (单字节操作) ---
    esp_err_t writeByte(spi_device_handle_t handle, uint8_t regAddr, uint8_t data);
    esp_err_t readByte(spi_device_handle_t handle, uint8_t regAddr, uint8_t *data);

private:
    spi_host_device_t _host; // 记录是 SPI2 还是 SPI3
};