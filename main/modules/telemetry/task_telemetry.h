#pragma once

/**
 * @brief 启动遥测任务
 * * 职责：
 * 1. 定期 (20Hz) 从 RobotBus 获取传感器数据
 * 2. 进行简单的姿态解算 (Accel -> Euler Angles)
 * 3. 获取系统状态 (Heap, Uptime)
 * 4. 打包 JSON 并通过 WebSocket 发送
 */
void start_telemetry_task();