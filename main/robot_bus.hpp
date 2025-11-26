#pragma once
#include "topic.hpp"
#include "command_queue.hpp"
#include "shared_types.h"  // 记得包含那个标准表格

/**
 * RobotBus (机器人总线)
 * 这是一个单例类 (Singleton)。
 */
class RobotBus
{
public:
    // ================= 定义所有的白板 =================
    Topic<ImuData> imu;            // IMU 数据的白板
    Topic<AttitudeData> attitude;  // 姿态数据的白板
    Topic<MagData> mag;

    // ================= 定义所有的信箱 =================
    CommandQueue<MotionCommand> motion_cmd;  // 运动指令的信箱

    // ================= 单例模式核心代码 =================
    // 这是为了保证全世界只有一个 RobotBus 实例
    static RobotBus& instance()
    {
        static RobotBus instance;  // 静态变量，只会创建一次
        return instance;
    }

    // 初始化函数
    // 主要是为了把信箱建好(分配内存)
    void init()
    {
        motion_cmd.init(10);  // 创建一个深度为10的队列
    }

private:
    // 私有的构造函数
    // 意思是：除了我自己，谁都不准创建 RobotBus
    // 这样就强制大家只能通过 instance() 来使用它
    RobotBus() = default;
};