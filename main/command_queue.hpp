#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h> // 引入 FreeRTOS 的队列功能

/**
 * 模板类 CommandQueue<T>
 * 同样也是通用的。T 可以是 MotionCommand。
 */
template<typename T>
class CommandQueue {
private:
    QueueHandle_t _q = nullptr; // 这是 FreeRTOS 原生的队列句柄(Handle)

public:
    // 初始化信箱大小
    // length = 10 表示信箱最多能塞 10 封信，再多就塞不进去了
    void init(size_t length = 10) {
        // 创建队列，每封信的大小是 sizeof(T)
        _q = xQueueCreate(length, sizeof(T));
    }

    // 投递信件 (发送)
    bool send(const T& cmd, TickType_t timeout = 0) {
        if(!_q) return false; // 如果信箱没建好，发送失败
        
        // xQueueSend: 往队列尾部塞数据
        // timeout = 0 表示：如果信箱满了，我不等，直接返回失败（不堵塞）
        return xQueueSend(_q, &cmd, timeout) == pdPASS;
    }

    // 取出信件 (接收)
    bool receive(T& out, TickType_t timeout = portMAX_DELAY) {
        if(!_q) return false;

        // xQueueReceive: 从队列头部拿数据
        // portMAX_DELAY 表示：如果信箱是空的，我就死等，直到有信来为止（堵塞）
        return xQueueReceive(_q, &out, timeout) == pdPASS;
    }
};