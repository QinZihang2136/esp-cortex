#pragma once

#include "esp_http_server.h"
#include "esp_err.h"
class WebServer
{
public:
    // 单例模式获取实例
    static WebServer& instance() { static WebServer i; return i; }
    void send_ws_message(const char* msg);

    /**
     * @brief 初始化 Web 服务器和 mDNS 服务
     * 启动后可通过 IP 或 http://espcortex.local 访问
     */
    void init();

private:
    WebServer() = default;
    httpd_handle_t server = NULL;

    // --- HTTP 请求处理函数 ---
    static esp_err_t index_get_handler(httpd_req_t* req);
    static esp_err_t app_js_handler(httpd_req_t* req);
    static esp_err_t style_css_handler(httpd_req_t* req);
    static esp_err_t ws_handler(httpd_req_t* req);

};