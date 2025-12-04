#pragma once

#include <vector>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h" // 引入信号量/互斥锁
#include <string>       // <--- 【必须添加这一行】
/**
 * @brief 支持的参数类型
 */
enum class ParamType
{
    INT32,
    FLOAT
};

/**
 * @brief 参数节点
 * 警告：name 字段必须指向静态字符串常量，不能是局部变量！
 */
struct ParamNode
{
    const char* name;       // 参数名 (Key)
    ParamType type;         // 类型
    void* ptr;              // 内存地址
    union
    {
        int32_t i;
        float f;
    } default_val;          // 默认值
};

/**
 * @brief 参数注册表组件 (Singleton)
 * 功能：提供线程安全的参数注册、读取、修改和持久化存储。
 */
class ParamRegistry
{
public:
    // 获取单例
    static ParamRegistry& instance()
    {
        static ParamRegistry _instance;
        return _instance;
    }

    /**
     * @brief 初始化参数系统
     * 1. 初始化 NVS Flash
     * 2. 创建互斥锁
     * 必须在 app_main 最开始调用
     */
    void init();

    // ================== Float 接口 ==================

    /**
     * @brief 注册浮点参数
     * @param name 参数名 (必须是静态字符串，如 "PID_P")
     * @param ptr 变量地址
     * @param default_val 默认值
     */
    void register_float(const char* name, float* ptr, float default_val);

    /**
     * @brief 设置浮点参数并保存
     * @return true 成功, false 失败
     */
    bool set_float(const char* name, float new_value);


    // ================== Int32 接口 ==================

    /**
     * @brief 注册整型参数 (用于状态、计数、枚举)
     */
    void register_int32(const char* name, int32_t* ptr, int32_t default_val);

    /**
     * @brief 设置整型参数并保存
     */
    bool set_int32(const char* name, int32_t new_value);


    // ================== 调试工具 ==================
    void print_all();

    /**
     * @brief 打印所有“僵尸参数”
     * 遍历 NVS Flash，列出那些存在于 Flash 中，但当前代码没有注册(未使用)的参数。
     */
    void print_unused();

    /**
     * @brief 清理“僵尸参数”
     * 删除所有当前代码未注册的参数，释放 Flash 空间。
     * 建议仅在开发者确信需要清理时调用。
     */
    void remove_unused();
    void debug_dump_nvs();

    // ================== 序列化与反序列化接口 ==================

/**
 * @brief 生成所有参数的 JSON 字符串
 * 格式: {"type":"param_list", "payload": [{"name":"PID_P", "type":1, "val":0.5}, ...]}
 * @return std::string 包含完整的 JSON 数据
 */
    std::string dump_to_json_string();

    /**
     * @brief 通用参数更新接口
     * @param key 参数名
     * @param val 新数值 (统一由 float 传入，内部自动判断类型并强转)
     * @return true 更新成功, false 找不到参数
     */
    bool set_param_by_key(const char* key, float val);

private:
    ParamRegistry() = default; // 私有构造

    // 内部参数列表
    std::vector<ParamNode> _params;

    // 互斥锁：保证多线程安全
    SemaphoreHandle_t _mutex = nullptr;

    // 内部查找函数 (不加锁，由调用者加锁)
    ParamNode* find_node_unsafe(const char* name);

    // NVS 配置
    const size_t PARAM_WARN_THRESHOLD = 200;


};