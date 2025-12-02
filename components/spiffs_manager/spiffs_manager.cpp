#include "spiffs_manager.hpp"
#include "esp_spiffs.h"
#include "esp_log.h"

static const char* TAG = "SPIFFS";

void SpiffsManager::init(const char* partition_label, const char* mount_point)
{
    if (_is_mounted)
    {
        ESP_LOGW(TAG, "SPIFFS already mounted!");
        return;
    }

    ESP_LOGI(TAG, "Initializing SPIFFS...");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = mount_point,
      .partition_label = partition_label,
      .max_files = 5,    // 同时打开文件的最大数量
      .format_if_mount_failed = true // 如果挂载失败（比如第一次），自动格式化
    };

    // 1. 注册并挂载文件系统
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    // 2. 检查分区信息
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(partition_label, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information");
    }
    else
    {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    _is_mounted = true;
    _partition_label = partition_label;
    ESP_LOGI(TAG, "SPIFFS mounted at %s", mount_point);
}

void SpiffsManager::print_usage()
{
    if (!_is_mounted) return;
    size_t total = 0, used = 0;
    if (esp_spiffs_info(_partition_label, &total, &used) == ESP_OK)
    {
        ESP_LOGI(TAG, "Storage Usage: %d / %d bytes", used, total);
    }
}