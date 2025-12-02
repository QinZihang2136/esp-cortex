#pragma once
#include "esp_err.h"

class SpiffsManager
{
public:
    // 单例访问点
    static SpiffsManager& instance()
    {
        static SpiffsManager instance;
        return instance;
    }

    /**
     * @brief 初始化并挂载 SPIFFS
     * @param partition_label 分区表里的名字，默认是 "storage"
     * @param mount_point 虚拟文件系统的路径，默认是 "/spiffs"
     */
    void init(const char* partition_label = "storage", const char* mount_point = "/spiffs");

    // 检查是否已挂载
    bool is_mounted() const { return _is_mounted; }

    // 打印当前磁盘使用情况 (Total / Used)
    void print_usage();

private:
    SpiffsManager() = default;
    bool _is_mounted = false;
    const char* _partition_label = nullptr;
};