#include "i2c_bus.h"
#include "esp_log.h"

static const char* TAG = "I2C_BUS";

I2CBus::I2CBus(i2c_port_t port) : _port(port) {}

I2CBus::~I2CBus() { close(); }

esp_err_t I2CBus::begin(int sda_pin, int scl_pin, uint32_t clk_speed) {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_pin;
    conf.scl_io_num = scl_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE; // 内部上拉通常不够，但开着无妨
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = clk_speed;
    
    esp_err_t err = i2c_param_config(_port, &conf);
    if (err != ESP_OK) return err;

    return i2c_driver_install(_port, conf.mode, 0, 0, 0);
}

esp_err_t I2CBus::close() {
    return i2c_driver_delete(_port);
}

esp_err_t I2CBus::writeRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(_port, dev_addr, write_buf, sizeof(write_buf), pdMS_TO_TICKS(100));
}

esp_err_t I2CBus::readRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data) {
    return i2c_master_write_read_device(_port, dev_addr, &reg_addr, 1, data, 1, pdMS_TO_TICKS(100));
}

esp_err_t I2CBus::readRegisters(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(_port, dev_addr, &reg_addr, 1, data, len, pdMS_TO_TICKS(100));
}