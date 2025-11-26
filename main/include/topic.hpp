#pragma once
#include <mutex>            // C++标准互斥锁 (用来防止打架)
#include <atomic>           // 原子操作 (用来做版本号计数)
#include <freertos/FreeRTOS.h> // 引入FreeRTOS系统
#include <freertos/task.h>     // 引入任务管理

/**
 * 模板类 Topic<T>
 * "template <typename T>" 的意思是：这块白板是通用的。
 * T 可以是 ImuData，也可以是 AttitudeData。
 * 你给我什么类型，我就变成存放什么类型的白板。
 */
template<typename T>
class Topic {
private:
    T _data{};                      // 真正存放数据的地方 (白板的板面)
    std::atomic<uint32_t> _gen{0};  // 版本号 (每写一次就+1)
    mutable std::mutex _mutex;      // 互斥锁 (写数据时的“请勿打扰”牌子)
    TaskHandle_t _notify_task = nullptr; // 需要被叫醒的任务 (控制员的工号)

public:
    Topic() = default; // 默认构造函数

    /**
     * @brief 发布数据 (写白板)
     * @param msg 要写入的数据
     * 注意：不能在中断(ISR)里调用这个函数，因为用了锁！
     */
    void publish(const T& msg) {
        {
            // 1. 上锁 (Lock)
            // std::lock_guard 是一个保镖。
            // 当它出生时(构造)，自动上锁；
            // 当它消失时(大括号结束)，自动解锁。
            // 这样防止你忘记解锁导致死机。
            std::lock_guard<std::mutex> lock(_mutex);
            
            // 2. 写入数据
            _data = msg;
            
            // 3. 版本号 +1
            // 告诉大家：我有新数据啦！
            _gen++; 
        } // <--- 在这里，保镖消失，锁自动解开

        // 4. 如果有人注册了“叫醒服务”，就去叫醒他
        if (_notify_task != nullptr) {
            // FreeRTOS 的函数：发送一个信号给指定任务
            xTaskNotifyGive(_notify_task);
        }
    }

    /**
     * @brief 检查并复制更新 (看白板)
     * @param out 用来接收数据的容器
     * @param last_gen 我上次看到的版本号
     * @return true(有新数据), false(没更新)
     */
    bool copy_if_updated(T& out, uint32_t& last_gen) {
        // 1. 快速检查：现在的版本号 等不等于 我上次记的版本号？
        uint32_t current_gen = _gen.load();
        if (current_gen == last_gen) {
            return false; // 没变化，直接走人，不浪费时间上锁
        }

        // 2. 既然有新数据，那就上锁，准备抄写
        {
            std::lock_guard<std::mutex> lock(_mutex);
            out = _data; // 抄写数据
        }

        // 3. 更新我手中的版本号
        last_gen = current_gen;
        return true; // 告诉调用者：我拿到新货了
    }

    /**
     * @brief 注册“叫醒服务”
     * 谁调用这个函数，以后白板更新时就叫醒谁。
     */
    void register_notifier() {
        // 获取当前正在运行的任务句柄(ID)，存下来
        _notify_task = xTaskGetCurrentTaskHandle();
    }
};