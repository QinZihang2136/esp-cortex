/**
 * @file param_registry.cpp
 * @brief 参数注册表组件实现
 * * 负责管理系统参数的注册、读写、持久化存储以及垃圾回收。
 * 兼容 ESP-IDF v5.x NVS API。
 */

#include "param_registry.hpp"
#include <cstring>
#include <vector>  // 【修复】必须包含，用于存储参数列表
#include <string>  // 【修复】必须包含，用于垃圾回收时的临时Key存储
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <cstdio> // for snprintf
static const char* TAG = "PARAM";
static const char* NVS_NS = "robot_param"; // NVS 命名空间，相当于文件夹名

/**
 * @brief 系统初始化
 * 必须在系统启动最早期调用
 */
void ParamRegistry::init()
{
    // 1. 创建互斥锁 (Mutex)
    // 保护 _params 列表不被多线程同时修改导致崩溃
    if (_mutex == nullptr)
    {
        _mutex = xSemaphoreCreateMutex();
    }

    // 2. 初始化 NVS Flash
    esp_err_t err = nvs_flash_init();

    // 如果 NVS 分区没有空页或版本不匹配 (通常发生在固件升级或分区表变动后)
    // 需要执行擦除并重新初始化
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS Partition truncated/version mismatch, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err); // 如果这里报错，说明 Flash 硬件可能损坏或分区表严重错误

    ESP_LOGI(TAG, "Component Initialized (NVS + Mutex Ready)");
}

/**
 * @brief 内部查找函数 (不加锁)
 * @note 调用此函数前必须在外部上锁！
 */
ParamNode* ParamRegistry::find_node_unsafe(const char* name)
{
    for (auto& node : _params)
    {
        if (strcmp(node.name, name) == 0) return &node;
    }
    return nullptr;
}

// =================================================================================
//                               Float 类型实现
// =================================================================================

void ParamRegistry::register_float(const char* name, float* ptr, float default_val)
{
    // 1. 上锁：进入临界区
    xSemaphoreTake(_mutex, portMAX_DELAY);

    // 2. 查重：防止同一个 Key 被注册两次
    if (find_node_unsafe(name) != nullptr)
    {
        ESP_LOGW(TAG, "Duplicate float '%s' ignored", name);
        xSemaphoreGive(_mutex); // 退出前必须解锁
        return;
    }

    // 3. 构建节点数据
    ParamNode node;
    node.name = name;
    node.type = ParamType::FLOAT;
    node.ptr = (void*)ptr; // 擦除类型信息，存为通用指针
    node.default_val.f = default_val;

    // 4. 尝试从 NVS 读取旧值
    nvs_handle_t handle;
    bool loaded = false;

    // 以只读模式打开 NVS
    if (nvs_open(NVS_NS, NVS_READONLY, &handle) == ESP_OK)
    {
        size_t size = sizeof(float);
        // 使用 Blob (二进制块) 读写浮点数，避免字符串转换带来的精度损失
        if (nvs_get_blob(handle, name, ptr, &size) == ESP_OK)
        {
            loaded = true; // 读取成功，*ptr 的值已经被改写为 Flash 里的值
        }
        nvs_close(handle); // 记得关闭句柄
    }

    // 5. 结果处理
    if (!loaded)
    {
        // 没读到（第一次运行），使用默认值初始化变量
        *ptr = default_val;
    }
    else
    {
        // 读到了，打印一下告知用户
        ESP_LOGI(TAG, "Load Float: %-15s = %.3f", name, *ptr);
    }

    // 6. 加入列表并解锁
    _params.push_back(node);
    xSemaphoreGive(_mutex);
}

bool ParamRegistry::set_float(const char* name, float new_value)
{
    xSemaphoreTake(_mutex, portMAX_DELAY);

    // 1. 查找节点
    ParamNode* node = find_node_unsafe(name);
    // 检查节点是否存在，且类型是否匹配
    if (!node || node->type != ParamType::FLOAT)
    {
        ESP_LOGE(TAG, "Set Float Failed: '%s' not registered!", name);
        xSemaphoreGive(_mutex);
        return false;
    }

    // 2. 更新 RAM (立即生效，控制循环立马就能用到新值)
    *(float*)node->ptr = new_value;

    // 3. 更新 Flash (持久化)
    nvs_handle_t handle;
    if (nvs_open(NVS_NS, NVS_READWRITE, &handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS Open Failed");
        xSemaphoreGive(_mutex);
        return false;
    }

    // 写入 Blob
    nvs_set_blob(handle, name, &new_value, sizeof(float));
    // 提交 (Commit) 是必须的，否则掉电丢失
    nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "Set Float:  %-15s -> %.3f", name, new_value);

    xSemaphoreGive(_mutex);
    return true;
}

// =================================================================================
//                               Int32 类型实现
// =================================================================================

void ParamRegistry::register_int32(const char* name, int32_t* ptr, int32_t default_val)
{
    xSemaphoreTake(_mutex, portMAX_DELAY);

    if (find_node_unsafe(name) != nullptr)
    {
        ESP_LOGW(TAG, "Duplicate int '%s' ignored", name);
        xSemaphoreGive(_mutex);
        return;
    }

    ParamNode node;
    node.name = name;
    node.type = ParamType::INT32;
    node.ptr = (void*)ptr;
    node.default_val.i = default_val;

    nvs_handle_t handle;
    bool loaded = false;

    if (nvs_open(NVS_NS, NVS_READONLY, &handle) == ESP_OK)
    {
        int32_t flash_val = 0;
        // Int32 有 NVS 原生支持的接口，不需要用 Blob
        if (nvs_get_i32(handle, name, &flash_val) == ESP_OK)
        {
            *ptr = flash_val;
            loaded = true;
        }
        nvs_close(handle);
    }

    if (!loaded)
    {
        *ptr = default_val;
    }
    else
    {
        ESP_LOGI(TAG, "Load Int:   %-15s = %ld", name, (long)*ptr);
    }

    _params.push_back(node);
    xSemaphoreGive(_mutex);
}

bool ParamRegistry::set_int32(const char* name, int32_t new_value)
{
    xSemaphoreTake(_mutex, portMAX_DELAY);

    ParamNode* node = find_node_unsafe(name);
    if (!node || node->type != ParamType::INT32)
    {
        ESP_LOGE(TAG, "Set Int Failed: '%s' not registered!", name);
        xSemaphoreGive(_mutex);
        return false;
    }

    *(int32_t*)node->ptr = new_value;

    nvs_handle_t handle;
    if (nvs_open(NVS_NS, NVS_READWRITE, &handle) != ESP_OK)
    {
        xSemaphoreGive(_mutex);
        return false;
    }

    nvs_set_i32(handle, name, new_value);
    nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "Set Int:    %-15s -> %ld", name, (long)new_value);

    xSemaphoreGive(_mutex);
    return true;
}

// =================================================================================
//                          打印与清理 (兼容 ESP-IDF v5.x)
// =================================================================================

void ParamRegistry::print_all()
{
    xSemaphoreTake(_mutex, portMAX_DELAY);

    ESP_LOGI(TAG, "=== System Params Dump (Total: %d) ===", (int)_params.size());
    for (const auto& node : _params)
    {
        if (node.type == ParamType::FLOAT)
        {
            ESP_LOGI(TAG, "  [F] %-15s : %.3f", node.name, *(float*)node.ptr);
        }
        else if (node.type == ParamType::INT32)
        {
            ESP_LOGI(TAG, "  [I] %-15s : %ld", node.name, (long)*(int32_t*)node.ptr);
        }
    }
    ESP_LOGI(TAG, "=======================================");

    xSemaphoreGive(_mutex);
}

void ParamRegistry::print_unused()
{
    xSemaphoreTake(_mutex, portMAX_DELAY);
    ESP_LOGI(TAG, "--- Checking for Unused Parameters (Zombies) ---");

    // 【修复】ESP-IDF v5.x 迭代器 API 写法
    nvs_iterator_t it = nullptr;
    // 参数1: 分区名("nvs")，参数2: 命名空间(NVS_NS)，参数3: 类型(ANY)，参数4: 迭代器指针
    esp_err_t res = nvs_entry_find("nvs", NVS_NS, NVS_TYPE_ANY, &it);

    int count = 0;
    while (res == ESP_OK)
    {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info); // 获取当前条目信息

        // 拿着 Flash 里的 Key 去 RAM 表里找
        if (find_node_unsafe(info.key) == nullptr)
        {
            ESP_LOGW(TAG, "  [Unused] Key: %s (Found in Flash but not registered)", info.key);
            count++;
        }

        // 移动到下一个条目
        res = nvs_entry_next(&it);
    }
    // 释放迭代器内存
    nvs_release_iterator(it);

    if (count == 0)
    {
        ESP_LOGI(TAG, "  Good! No unused parameters found.");
    }
    else
    {
        ESP_LOGW(TAG, "  Found %d unused parameters.", count);
    }

    xSemaphoreGive(_mutex);
}
void ParamRegistry::debug_dump_nvs()
{
    xSemaphoreTake(_mutex, portMAX_DELAY);
    ESP_LOGW(TAG, "=== DEBUG: DUMPING ALL NVS KEYS ===");

    nvs_iterator_t it = nullptr;
    esp_err_t res = nvs_entry_find("nvs", NVS_NS, NVS_TYPE_ANY, &it);

    while (res == ESP_OK)
    {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);

        // 无论是否注册，都打印出来
        ESP_LOGI(TAG, "  Key in Flash: %s", info.key);

        res = nvs_entry_next(&it);
    }
    nvs_release_iterator(it);
    ESP_LOGW(TAG, "=== DUMP FINISHED ===");
    xSemaphoreGive(_mutex);
}
void ParamRegistry::remove_unused()
{
    xSemaphoreTake(_mutex, portMAX_DELAY);
    ESP_LOGW(TAG, "--- STARTING GARBAGE COLLECTION ---");

    // 1. 收集阶段
    // 我们不能一边遍历迭代器一边删除 Key，这会导致迭代器失效崩溃。
    // 所以先用 vector 把所有僵尸 Key 存下来。
    std::vector<std::string> keys_to_delete;

    nvs_iterator_t it = nullptr;
    esp_err_t res = nvs_entry_find("nvs", NVS_NS, NVS_TYPE_ANY, &it);

    while (res == ESP_OK)
    {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);

        if (find_node_unsafe(info.key) == nullptr)
        {
            // 存入待删除列表
            keys_to_delete.push_back(std::string(info.key));
        }
        res = nvs_entry_next(&it);
    }
    nvs_release_iterator(it);

    // 2. 删除阶段
    if (keys_to_delete.empty())
    {
        ESP_LOGI(TAG, "Nothing to delete.");
        xSemaphoreGive(_mutex);
        return;
    }

    nvs_handle_t handle;
    if (nvs_open(NVS_NS, NVS_READWRITE, &handle) == ESP_OK)
    {
        for (const auto& key : keys_to_delete)
        {
            // 执行物理删除
            esp_err_t err = nvs_erase_key(handle, key.c_str());
            if (err == ESP_OK)
            {
                ESP_LOGI(TAG, "  Deleted: %s", key.c_str());
            }
            else
            {
                ESP_LOGE(TAG, "  Failed to delete: %s", key.c_str());
            }
        }
        // 必须提交
        nvs_commit(handle);
        nvs_close(handle);

        // 强制转换为 int 避免 size_t 格式化警告
        ESP_LOGW(TAG, "Cleanup Done. Deleted %d parameters.", (int)keys_to_delete.size());
    }
    else
    {
        ESP_LOGE(TAG, "Failed to open NVS for deletion.");
    }

    xSemaphoreGive(_mutex);
}

// =================================================================================
//                          JSON 序列化与通用更新
// =================================================================================

std::string ParamRegistry::dump_to_json_string()
{
    xSemaphoreTake(_mutex, portMAX_DELAY);

    // 预估一个大小，避免频繁 realloc (假设平均每个参数 60 字节)
    std::string json;
    json.reserve(512 + _params.size() * 60);

    // 头部
    json += "{\"type\":\"param_list\",\"payload\":[";

    char buffer[128];
    for (size_t i = 0; i < _params.size(); i++)
    {
        const auto& node = _params[i];

        // 获取当前值
        float val_f = 0.0f;
        int type_code = 0; // 0=INT, 1=FLOAT (对应 app.js 里的定义)

        if (node.type == ParamType::FLOAT)
        {
            val_f = *(float*)node.ptr;
            type_code = 1;
        }
        else
        {
            val_f = (float)(*(int32_t*)node.ptr); // 转成 float 方便统一传输
            type_code = 0;
        }

        // 格式化单个对象: {"name":"xxx", "type":1, "val":1.23}
        // %.3f 保留3位小数，对于整数它会显示 x.000，app.js 会处理
        int len = snprintf(buffer, sizeof(buffer),
            "{\"name\":\"%s\",\"type\":%d,\"val\":%.4f}",
            node.name, type_code, val_f);

        json.append(buffer, len);

        // 如果不是最后一个，加逗号
        if (i < _params.size() - 1)
        {
            json += ",";
        }
    }

    // 尾部
    json += "]}";

    xSemaphoreGive(_mutex);
    return json;
}

bool ParamRegistry::set_param_by_key(const char* key, float val)
{
    // 复用 set_float 和 set_int32 的逻辑，但这里不仅是查找，还要根据类型分发
    // 为了避免死锁，我们这里重新上锁，并直接操作内存/NVS，或者调用 specific set 函数
    // 最简单的方法是调用现有的 set_float/set_int32，但它们会再次上锁。
    // *最佳实践*：直接在这里复用逻辑。

    xSemaphoreTake(_mutex, portMAX_DELAY);

    ParamNode* node = find_node_unsafe(key);
    if (!node)
    {
        ESP_LOGE(TAG, "Update failed: Key '%s' not found", key);
        xSemaphoreGive(_mutex);
        return false;
    }

    esp_err_t err = ESP_FAIL;
    nvs_handle_t handle;

    // 打开 NVS
    if (nvs_open(NVS_NS, NVS_READWRITE, &handle) != ESP_OK)
    {
        xSemaphoreGive(_mutex);
        return false;
    }

    if (node->type == ParamType::FLOAT)
    {
        // 更新 RAM
        *(float*)node->ptr = val;
        // 更新 NVS
        err = nvs_set_blob(handle, key, &val, sizeof(float));
        ESP_LOGI(TAG, "Web Update Float: %s -> %.3f", key, val);
    }
    else if (node->type == ParamType::INT32)
    {
        int32_t i_val = (int32_t)val; // 强转回整数
        // 更新 RAM
        *(int32_t*)node->ptr = i_val;
        // 更新 NVS
        err = nvs_set_i32(handle, key, i_val);
        ESP_LOGI(TAG, "Web Update Int:   %s -> %ld", key, (long)i_val);
    }

    if (err == ESP_OK) nvs_commit(handle);
    nvs_close(handle);

    xSemaphoreGive(_mutex);
    return (err == ESP_OK);
}