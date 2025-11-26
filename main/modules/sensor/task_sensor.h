#pragma once

/**
 * @brief 启动传感器任务
 * 负责初始化 SPI 总线、IMU 传感器，并开始周期性采集数据发布到 DataBus
 */
void start_sensor_task();