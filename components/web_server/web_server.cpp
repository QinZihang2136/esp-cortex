#include "web_server.hpp"
#include "esp_log.h"
#include "mdns.h"       // 引入 mDNS 头文件
#include <stdio.h>      // 用于文件操作 fopen 等
#include <sys/stat.h>
#include "cJSON.h"
#include "param_registry.hpp"
#include <string>       // [必须] 引入 string 支持

static const char* TAG = "WEB_SERVER";

// [新增] 全局变量：记录当前连接的 WebSocket 客户端 ID (Socket File Descriptor)
// 作用：当 telemetry 任务想要主动推送数据时，它需要知道推给谁。
// 限制：这是一个简化实现，只支持“最后一个连接的客户端”。如果多个浏览器同时开，只有最后一个能收到遥测。
static int g_client_fd = -1;

// 辅助结构体 (暂时保留，虽然目前简化逻辑没用到)
struct AsyncRespArg
{
    httpd_handle_t hd;
    int fd;
};

// ==========================================
// 辅助函数：从 SPIFFS 读取文件并发送给浏览器
// ==========================================
static esp_err_t send_file_from_spiffs(httpd_req_t* req, const char* filename, const char* type)
{
    // 1. 尝试打开文件
    FILE* f = fopen(filename, "r");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file: %s", filename);
        // 如果文件找不到，返回 404 Not Found
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // 2. 设置 Content-Type (告诉浏览器这是 HTML 还是 JS，否则浏览器不解析)
    httpd_resp_set_type(req, type);
    // 避免浏览器缓存旧版前端资源，确保每次刷新拿到最新 app.js/index.html
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    httpd_resp_set_hdr(req, "Expires", "0");

    // 3. 分块读取并发送 (避免一次性读入大文件导致 ESP32 内存耗尽)
    char chunk[1024];
    size_t chunksize;
    while ((chunksize = fread(chunk, 1, sizeof(chunk), f)) > 0)
    {
        // 发送这一块数据
        if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK)
        {
            fclose(f);
            ESP_LOGE(TAG, "File sending failed!");
            return ESP_FAIL;
        }
    }

    // 4. 发送结束标志 (空块) 并关闭文件
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// ==========================================
// mDNS 服务启动函数
// ==========================================
static void start_mdns_service()
{
    // 初始化 mDNS
    ESP_ERROR_CHECK(mdns_init());

    // 设置主机名 -> 之后可用 http://espcortex.local 访问
    // 注意：Windows 可能需要安装 Bonjour 服务才能解析 .local
    ESP_ERROR_CHECK(mdns_hostname_set("espcortex"));

    // 设置实例名 (描述性文字，用于服务发现)
    ESP_ERROR_CHECK(mdns_instance_name_set("EspCortex Robot Control"));

    // 添加 HTTP 服务记录，让 Bonjour/Zeroconf 扫描器能发现设备
    mdns_txt_item_t serviceTxtData[1] = {
        {"board", "esp32"}
    };
    ESP_ERROR_CHECK(mdns_service_add("EspCortex-Web", "_http", "_tcp", 80, serviceTxtData, 1));

    ESP_LOGI(TAG, "mDNS started. You can access via: http://espcortex.local");
}

// ==========================================
// URL 处理函数实现 (路由)
// ==========================================
esp_err_t WebServer::index_get_handler(httpd_req_t* req)
{
    return send_file_from_spiffs(req, "/spiffs/index.html", "text/html");
}

esp_err_t WebServer::app_js_handler(httpd_req_t* req)
{
    return send_file_from_spiffs(req, "/spiffs/app.js", "application/javascript");
}

esp_err_t WebServer::style_css_handler(httpd_req_t* req)
{
    return send_file_from_spiffs(req, "/spiffs/style.css", "text/css");
}

// ==========================================
// WebSocket 核心处理逻辑
// ==========================================
esp_err_t WebServer::ws_handler(httpd_req_t* req)
{
    // 1. 握手阶段 (Handshake)
    if (req->method == HTTP_GET)
    {
        ESP_LOGI(TAG, "Handshake done");
        // 这里可以获取 session_id，但为了简单我们使用 socket_fd
        return ESP_OK;
    }

    // 2. 数据接收准备
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    // 第一次调用：仅获取数据长度 (不读数据)
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) return ret;

    if (ws_pkt.len > 0)
    {
        // 分配内存
        uint8_t* buf = (uint8_t*)calloc(1, ws_pkt.len + 1);
        if (!buf) return ESP_ERR_NO_MEM;

        ws_pkt.payload = buf;
        // 第二次调用：真正读取数据到 buf
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) { free(buf); return ret; }

        // ====================================================
        // [关键] 记住是谁发的消息
        // ====================================================
        // 获取当前请求的底层 Socket ID。
        // 我们把它存入全局变量，这样 send_ws_message (遥测任务) 才知道数据该推给谁。
        g_client_fd = httpd_req_to_sockfd(req);

        ESP_LOGI(TAG, "RX from FD %d: %s", g_client_fd, (char*)ws_pkt.payload);

        // ====================================================
        // 3. 解析 JSON 指令 (业务逻辑)
        // ====================================================
        cJSON* root = cJSON_Parse((char*)ws_pkt.payload);
        if (root)
        {
            cJSON* cmd_item = cJSON_GetObjectItem(root, "cmd");

            if (cJSON_IsString(cmd_item))
            {
                const char* cmd = cmd_item->valuestring;

                // --- 场景 A: 网页请求获取参数列表 (Request-Response) ---
                if (strcmp(cmd, "get_params") == 0)
                {
                    // 1. 调用 Registry 生成大字符串
                    std::string json_str = ParamRegistry::instance().dump_to_json_string();

                    // 2. [稳妥] 直接使用 req 句柄原地回复
                    // 对于这种“一问一答”的场景，直接使用 httpd_ws_send_frame 是最可靠的，
                    // 因为 req 句柄当前是有效的，肯定能发回给提问者。
                    httpd_ws_frame_t resp_pkt;
                    memset(&resp_pkt, 0, sizeof(httpd_ws_frame_t));
                    resp_pkt.type = HTTPD_WS_TYPE_TEXT;
                    resp_pkt.payload = (uint8_t*)json_str.c_str();
                    resp_pkt.len = json_str.length();

                    esp_err_t send_ret = httpd_ws_send_frame(req, &resp_pkt);
                    if (send_ret == ESP_OK)
                    {
                        ESP_LOGI(TAG, "Sent params list: %d bytes", json_str.length());
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Failed to send params: %d", send_ret);
                    }
                }

                // --- 场景 B: 网页请求修改参数 (Command) ---
                else if (strcmp(cmd, "set_param") == 0)
                {
                    cJSON* key_item = cJSON_GetObjectItem(root, "key");
                    cJSON* val_item = cJSON_GetObjectItem(root, "val");

                    if (cJSON_IsString(key_item) && cJSON_IsNumber(val_item))
                    {
                        // 直接丢给 Registry，它会自动判断类型并更新 NVS
                        ParamRegistry::instance().set_param_by_key(key_item->valuestring, (float)val_item->valuedouble);
                    }
                }

                // --- 场景 C: 传感器标定指令 (预留) ---
                else if (strcmp(cmd, "calib_imu_side") == 0)
                {
                    // 这里未来可以调用 IMU 标定逻辑
                    ESP_LOGI(TAG, "Calibration command received");
                }
            }
            cJSON_Delete(root); // 必须释放 JSON 对象内存！
        }
        else
        {
            ESP_LOGE(TAG, "JSON Parse Error");
        }

        free(buf);
    }
    return ESP_OK;
}

// ==========================================
// [新增实现] 发送数据给浏览器 (主动推送/遥测)
// ==========================================
esp_err_t WebServer::send_ws_message(const char* msg)
{
    // 1. 安全检查：如果服务器没启动，或者还没人连上来过，就不发
    if (!server || g_client_fd < 0) return ESP_ERR_INVALID_STATE;

    // 2. 构造数据包
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*)msg;
    ws_pkt.len = strlen(msg);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    // 3. 异步发送 (Async Send)
    // 这里不能用 httpd_ws_send_frame，因为它需要 req 句柄。
    // Telemetry 任务没有 req 句柄，只能用 httpd_ws_send_frame_async 配合 socket fd。
    esp_err_t ret = httpd_ws_send_frame_async(server, g_client_fd, &ws_pkt);

    // 如果发送失败（比如客户端断开了），重置 fd 防止继续发
    if (ret != ESP_OK)
    {
        // 只有当错误不是“稍后重试”时才认为是断开
        if (ret != ESP_ERR_HTTPD_INVALID_REQ)
        {
            // g_client_fd = -1; // 可选：如果很严格可以置 -1，但通常不需要
        }
    }
    return ret;
}

// ==========================================
// 初始化入口
// ==========================================
void WebServer::init()
{
    // 1. 先启动 mDNS
    start_mdns_service();

    // 2. 配置并启动 HTTP Server
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 12; // 增加 URI 处理上限
    config.stack_size = 8192;     // 增加栈大小，防止复杂操作 (如 JSON 解析) 爆栈

    ESP_LOGI(TAG, "Starting HTTP Server on port: %d", config.server_port);

    if (httpd_start(&server, &config) == ESP_OK)
    {

        // 注册 "/" -> index.html
        httpd_uri_t index_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = index_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &index_uri);

        // 注册 "/app.js" -> app.js
        httpd_uri_t js_uri = {
            .uri = "/app.js",
            .method = HTTP_GET,
            .handler = app_js_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &js_uri);

        // 注册 "/style.css" -> style.css
        httpd_uri_t css_uri = {
            .uri = "/style.css",
            .method = HTTP_GET,
            .handler = style_css_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &css_uri);

        // 注册 WebSocket 路由
        httpd_uri_t ws_uri = {
            .uri = "/ws",
            .method = HTTP_GET,
            .handler = ws_handler,
            .user_ctx = NULL,
            .is_websocket = true   // <--- 关键！开启 WebSocket 支持 (需 menuconfig 开启)
        };
        httpd_register_uri_handler(server, &ws_uri);

        ESP_LOGI(TAG, "Web Server Handlers registered.");
    }
    else
    {
        ESP_LOGE(TAG, "Error starting server!");
    }
}
