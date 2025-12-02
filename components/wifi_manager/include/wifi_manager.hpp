#pragma once
#include "esp_wifi.h"
#include "esp_event.h"

class WifiManager
{
public:
    static WifiManager& instance()
    {
        static WifiManager instance;
        return instance;
    }

    void init();
    bool is_ap_mode() const { return _is_ap_mode; }

private:
    WifiManager() = default;
    bool try_connect_sta(const char* ssid, const char* password, int retries = 3);
    void start_ap(const char* ssid, const char* password);
    bool _is_ap_mode = false;
};