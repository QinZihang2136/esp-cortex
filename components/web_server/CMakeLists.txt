idf_component_register(
    SRCS "web_server.cpp"
    INCLUDE_DIRS "include"
    # 关键点：添加 REQUIRES 或 PRIV_REQUIRES
    # esp_http_server: 用于 Web 服务
    # spiffs_manager: 你的代码里引用了 spiffs_manager.hpp，如果它也是个组件，这里要加
    # mdns: 下面我们要加的 mDNS 功能
    REQUIRES esp_http_server mdns json param_registry
)