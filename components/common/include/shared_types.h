#pragma once // 这是一个指令，告诉编译器：这个文件只允许被包含一次，防止重复定义出错
#include <cstdint> // 引入标准整数类型，比如 uint64_t (64位无符号整数)

// ==========================================
// 1. 状态数据 (State) - 对应“白板”上的数据
// ==========================================

// 传感器原始数据 (IMU)
// 就像是传感器员看到的仪表盘读数
struct ImuData
{
    float ax, ay, az;     // 三轴加速度 (单位: g 或 m/s^2)
    float gx, gy, gz;     // 三轴角速度 (单位: rad/s 或 deg/s)
    uint64_t timestamp_us; // 时间戳 (微秒)，记录这组数据是什么时候采集的
};

struct MagData
{
    float x, y, z; // Gauss
    uint64_t timestamp_us; // 时间戳 (微秒)，记录这组数据是什么时候采集的
};

// 【新增】气压计数据
struct BaroData
{
    float pressure;    // kPa
    float temperature; // Celsius
    uint64_t timestamp_us; // 时间戳 (微秒)，记录这组数据是什么时候采集的
};

// 姿态解算结果
// 这是经过数学计算后，机器人的“身体姿态”
struct AttitudeData
{
    float roll;  // 横滚角 (左右倾斜)
    float pitch; // 俯仰角 (前后倾斜)
    float yaw;   // 偏航角 (车头朝向)
    // 后面如果你学深了，还可以加四元数 float q[4];
};

struct BatteryState
{
    float voltage;      // 电压，例如 12.6
    float current;      // 电流，例如 1.5
    uint8_t percentage; // 百分比，例如 80
};

// ==========================================
// 2. 命令数据 (Command) - 对应“信箱”里的信件
// ==========================================

// 运动控制指令
// 上级(电脑)发给下级(ESP32)的命令
struct MotionCommand
{
    float vx;    // 前进速度 (m/s)
    float vy;    // 横移速度 (麦克纳姆轮才用到，普通车是0)
    float omega; // 旋转速度 (rad/s)
};