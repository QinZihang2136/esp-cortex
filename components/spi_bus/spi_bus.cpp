#include "spi_bus.h"
#include <cstring>

// SPI 读写位掩码定义
// 很多传感器(包括ICM42688)规定：
// 读取时，寄存器地址最高位(MSB)必须是 1
// 写入时，寄存器地址最高位(MSB)必须是 0
#define SPI_READ_BIT  0x80 
#define SPI_WRITE_BIT 0x7F

SPIBus::SPIBus(spi_host_device_t host) : _host(host) {}
SPIBus::~SPIBus() { close(); }

esp_err_t SPIBus::begin(int mosi_pin, int miso_pin, int sclk_pin) {
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = mosi_pin;
    buscfg.miso_io_num = miso_pin;
    buscfg.sclk_io_num = sclk_pin;
    buscfg.quadwp_io_num = -1; // 不使用四线模式的写保护引脚
    buscfg.quadhd_io_num = -1; // 不使用四线模式的HOLD引脚
    buscfg.max_transfer_sz = 4096; // 单次最大传输字节数 (DMA相关)

    // 初始化总线，启用自动 DMA 通道分配
    // DMA 对于大批量数据传输很重要，虽然读传感器数据量小，但用了也不亏
    return spi_bus_initialize(_host, &buscfg, SPI_DMA_CH_AUTO);
}

esp_err_t SPIBus::close() { return spi_bus_free(_host); }

esp_err_t SPIBus::addDevice(uint8_t mode, uint32_t clock_speed_hz, int cs_pin, spi_device_handle_t *handle) {
    spi_device_interface_config_t devcfg = {};
    devcfg.command_bits = 0;
    devcfg.address_bits = 8; // 关键：告诉 ESP32 发送数据前先发 8位(1字节) 的寄存器地址
    devcfg.mode = mode;      // SPI 模式 (CPOL, CPHA)
    devcfg.clock_speed_hz = clock_speed_hz;
    devcfg.spics_io_num = cs_pin; // 片选引脚
    devcfg.queue_size = 1;   // 事务队列深度，我们是同步传输，1就够了
    
    return spi_bus_add_device(_host, &devcfg, handle);
}

esp_err_t SPIBus::removeDevice(spi_device_handle_t handle) { return spi_bus_remove_device(handle); }

// 写入数据
esp_err_t SPIBus::writeBytes(spi_device_handle_t handle, uint8_t regAddr, size_t length, const uint8_t *data) {
    spi_transaction_t t = {};
    // 将寄存器地址与 0x7F 进行与操作，确保最高位为 0 (表示写入)
    t.addr = regAddr & SPI_WRITE_BIT; 
    t.length = length * 8;            // 长度单位是 bit (位)，不是字节
    t.tx_buffer = data;               // 数据源指针
    return spi_device_transmit(handle, &t); // 阻塞式传输，直到发送完毕
}

// 读取数据
esp_err_t SPIBus::readBytes(spi_device_handle_t handle, uint8_t regAddr, size_t length, uint8_t *data) {
    spi_transaction_t t = {};
    // 将寄存器地址与 0x80 进行或操作，确保最高位为 1 (表示读取)
    t.addr = regAddr | SPI_READ_BIT;  
    t.length = length * 8;
    t.rxlength = length * 8;
    t.rx_buffer = data;               // 接收缓冲区指针
    return spi_device_transmit(handle, &t);
}

// 便捷封装：写单字节
esp_err_t SPIBus::writeByte(spi_device_handle_t handle, uint8_t regAddr, uint8_t data) {
    return writeBytes(handle, regAddr, 1, &data);
}

// 便捷封装：读单字节
esp_err_t SPIBus::readByte(spi_device_handle_t handle, uint8_t regAddr, uint8_t *data) {
    return readBytes(handle, regAddr, 1, data);
}