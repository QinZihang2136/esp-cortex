#pragma once
#include "i2c_bus.h"

// 默认地址 (0x0D)
#define QMC5883L_ADDR 0x0D

class QMC5883L {
public:
    struct CalibrationData {
        float offset[3]; // X, Y, Z 偏移
        float scale[3];  // X, Y, Z 比例
    };

    /**
     * @brief 构造函数
     * @param bus 指向 I2CBus 对象的指针
     * @param addr I2C 设备地址
     */
    QMC5883L(I2CBus* bus, uint8_t addr = QMC5883L_ADDR);

    /**
     * @brief 初始化传感器
     * 默认配置: Continuous Mode, 100Hz, 8G Range, OSR=512
     */
    esp_err_t begin();

    /**
     * @brief 检查数据是否就绪 (DRDY)
     * 用于轮询判断
     */
    bool isDataReady();

    /**
     * @brief 读取磁场数据 (单位: Gauss)
     * 已包含校准运算
     */
    esp_err_t readData(float* x, float* y, float* z);

    // 设置校准参数
    void setCalibration(const CalibrationData& cal_data);

private:
    I2CBus* _bus;
    uint8_t _addr;
    CalibrationData _cal;
    float _range_scale; // 毫高斯转换系数

    esp_err_t writeReg(uint8_t reg, uint8_t val);
    esp_err_t readReg(uint8_t reg, uint8_t* val);
};