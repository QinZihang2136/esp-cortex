# 参数管理系统 (Parameter Registry)

## 概述

参数管理系统提供**远程在线调参**和**持久化存储**功能。支持通过 Web 界面实时修改系统参数，修改后立即生效并自动保存至 NVS (Non-Volatile Storage)。

## 核心特性

- **动态注册**: 模块化设计，支持运行时注册参数
- **类型支持**: `float`, `int32_t`, `bool`
- **持久化存储**: 基于 ESP-IDF NVS，掉电不丢失
- **Web 调参**: 通过 Web 界面实时修改参数
- **默认值管理**: 首次使用自动写入默认值
- **参数分组**: 支持按模块分组显示

## 架构设计

```
┌─────────────────────────────────────────────────────┐
│              ParamRegistry (单例)                    │
└─────────────────────────────────────────────────────┘
                          │
         ┌────────────────┼────────────────┐
         │                │                │
    ┌────▼────┐     ┌─────▼─────┐    ┌───▼────┐
    │   NVS   │     │   RAM    │    │  Web   │
    │ (Flash) │     │  Cache   │    │  API   │
    │         │     │          │    │        │
    └─────────┘     └──────────┘    └────────┘
                            │
                    ┌───────┴────────┐
                    │                │
               ┌────▼────┐      ┌───▼────┐
               │ Sensor  │      │  EKF   │
               │ Params  │      │ Params │
               └─────────┘      └────────┘
```

## 数据结构

### 参数定义

**位置**: `components/common/include/shared_types.h`

```cpp
struct ParamInfo {
    const char* name;        // 参数名称 (唯一)
    const char* group;       // 分组名称
    ParamType type;          // 参数类型
    void* ptr;              // 指向 RAM 变量的指针
    float min_val;          // 最小值 (仅 float)
    float max_val;          // 最大值 (仅 float)
    float default_val;      // 默认值
};

enum class ParamType {
    FLOAT,
    INT32,
    BOOL
};
```

### 预定义参数列表

```cpp
// 传感器校准参数
#define PARAM_ACCEL_BIAS_X  "ACCEL_BIAS_X"
#define PARAM_ACCEL_BIAS_Y  "ACCEL_BIAS_Y"
#define PARAM_ACCEL_BIAS_Z  "ACCEL_BIAS_Z"
#define PARAM_ACCEL_SCALE_X "ACCEL_SCALE_X"
#define PARAM_ACCEL_SCALE_Y "ACCEL_SCALE_Y"
#define PARAM_ACCEL_SCALE_Z "ACCEL_SCALE_Z"

#define PARAM_MAG_BIAS_X    "MAG_BIAS_X"
#define PARAM_MAG_BIAS_Y    "MAG_BIAS_Y"
#define PARAM_MAG_BIAS_Z    "MAG_BIAS_Z"
#define PARAM_MAG_SCALE_X   "MAG_SCALE_X"
#define PARAM_MAG_SCALE_Y   "MAG_SCALE_Y"
#define PARAM_MAG_SCALE_Z   "MAG_SCALE_Z"

// EKF 滤波器参数
#define PARAM_EKF_Q_ANGLE   "EKF_Q_ANGLE"
#define PARAM_EKF_Q_BIAS    "EKF_Q_BIAS"
#define PARAM_EKF_R_ACCEL   "EKF_R_ACCEL"
#define PARAM_EKF_R_MAG     "EKF_R_MAG"

// WiFi 配置
#define PARAM_WIFI_SSID     "WIFI_SSID"
#define PARAM_WIFI_PASSWORD "WIFI_PASSWORD"
```

## 使用方法

### 注册参数

**步骤 1**: 定义 RAM 变量

```cpp
// main/modules/estimator/task_estimator.cpp

static float ekf_q_angle = 0.0005f;
static float ekf_q_bias = 0.00001f;
static float ekf_r_accel = 0.05f;
static float ekf_r_mag = 0.1f;
```

**步骤 2**: 注册到 ParamRegistry

```cpp
void register_ekf_params() {
    auto& params = ParamRegistry::instance();

    params.register_param("EKF_Q_ANGLE", "EKF", ParamType::FLOAT,
                          &ekf_q_angle, 0.0f, 1.0f, 0.0005f);

    params.register_param("EKF_Q_BIAS", "EKF", ParamType::FLOAT,
                          &ekf_q_bias, 0.0f, 0.1f, 0.00001f);

    params.register_param("EKF_R_ACCEL", "EKF", ParamType::FLOAT,
                          &ekf_r_accel, 0.0f, 1.0f, 0.05f);

    params.register_param("EKF_R_MAG", "EKF", ParamType::FLOAT,
                          &ekf_r_mag, 0.0f, 1.0f, 0.1f);
}
```

**步骤 3**: 在初始化时调用

```cpp
void start_estimator_task() {
    register_ekf_params();  // 注册参数
    // ...
}
```

### 读取参数

**方法 1**: 直接访问 RAM 变量 (最快)

```cpp
void ekf_update() {
    ekf.set_process_noise(ekf_q_angle, ekf_q_bias);
    ekf.set_measure_noise_accel(ekf_r_accel);
}
```

**方法 2**: 通过 ParamRegistry 查询

```cpp
auto& params = ParamRegistry::instance();

float q_angle = 0.0f;
if (params.get_param("EKF_Q_ANGLE", q_angle)) {
    ESP_LOGI(TAG, "Q_ANGLE = %.6f", q_angle);
}
```

### 修改参数

**方法 1**: 代码中修改 (自动保存)

```cpp
auto& params = ParamRegistry::instance();

float new_value = 0.001f;
params.set_param("EKF_Q_ANGLE", new_value);
```

**方法 2**: Web 界面修改

1. 打开 Web 控制台 → 参数管理
2. 找到 `EKF_Q_ANGLE` 参数
3. 输入新值: `0.001`
4. 点击 **修改** 按钮
5. 参数立即生效并保存至 NVS

### 使用参数值

```cpp
// 在 EKF 初始化时使用参数
void init_ekf() {
    ekf.set_process_noise(ekf_q_angle, ekf_q_bias);
    ekf.set_measure_noise_accel(ekf_r_accel);
    ekf.set_measure_noise_mag(ekf_r_mag);
}

// 在运行时动态调整
void runtime_tune_ekf() {
    // 如果检测到震动过大，增加加速度计噪声
    if (vibration_detected) {
        ekf_r_accel = 0.2f;
        params.set_param("EKF_R_ACCEL", ekf_r_accel);
    }
}
```

## NVS 存储机制

### NVS 命名空间

所有参数存储在 `params` 命名空间下：

```
NVS Namespace: "params"
├── Key: "ACCEL_BIAS_X"    → Value: 0.012
├── Key: "ACCEL_BIAS_Y"    → Value: -0.023
├── Key: "ACCEL_BIAS_Z"    → Value: 0.987
├── Key: "MAG_BIAS_X"      → Value: 0.123
├── Key: "EKF_Q_ANGLE"     → Value: 0.0005
└── ...
```

### 首次初始化流程

```
系统启动
    ↓
扫描 NVS 命名空间 "params"
    ↓
参数是否已存在?
    ├─ 是 → 读取值到 RAM Cache
    └─ 否 → 写入默认值到 NVS
```

**代码示例**:
```cpp
// components/param_registry/param_registry.cpp

void ParamRegistry::load_params_from_nvs() {
    nvs_handle_t handle;
    nvs_open("params", NVS_READWRITE, &handle);

    for (auto& param : param_list) {
        // 尝试读取
        esp_err_t err = ESP_OK;
        if (param.type == ParamType::FLOAT) {
            float* ptr = (float*)param.ptr;
            err = nvs_get_float(handle, param.name, ptr);
            if (err == ESP_ERR_NVS_NOT_FOUND) {
                // 首次使用，写入默认值
                *ptr = param.default_val;
                nvs_set_float(handle, param.name, param.default_val);
            }
        }
        // ... INT32, BOOL 类似处理
    }

    nvs_commit(handle);  // 提交到 Flash
    nvs_close(handle);
}
```

### 持久化保存流程

```
Web 界面修改参数
    ↓
WebSocket 消息 → Web Server
    ↓
ParamRegistry::set_param(new_value)
    ↓
更新 RAM Cache
    ↓
写入 NVS
    ↓
nvs_commit() (提交到 Flash)
    ↓
返回 ACK
```

## API 接口

### C++ API

**注册参数**:
```cpp
esp_err_t register_param(const char* name,
                         const char* group,
                         ParamType type,
                         void* ptr,
                         float min_val,
                         float max_val,
                         float default_val);
```

**获取参数**:
```cpp
esp_err_t get_param(const char* name, float& val);
esp_err_t get_param(const char* name, int32_t& val);
esp_err_t get_param(const char* name, bool& val);
```

**设置参数**:
```cpp
esp_err_t set_param(const char* name, float val);
esp_err_t set_param(const char* name, int32_t val);
esp_err_t set_param(const char* name, bool val);
```

**导出参数列表 (用于 Web)**:
```cpp
std::vector<ParamInfo> get_all_params();
std::vector<ParamInfo> get_params_by_group(const char* group);
```

### Web API

**获取所有参数**:
```
GET /api/params
```

响应:
```json
{
  "params": [
    {
      "name": "EKF_Q_ANGLE",
      "group": "EKF",
      "type": "float",
      "value": 0.0005,
      "min": 0.0,
      "max": 1.0,
      "default": 0.0005
    },
    ...
  ]
}
```

**修改参数**:
```
POST /api/params
Content-Type: application/json

{
  "name": "EKF_Q_ANGLE",
  "value": 0.001
}
```

响应:
```json
{
  "status": "success",
  "name": "EKF_Q_ANGLE",
  "value": 0.001
}
```

## 使用示例

### 示例 1: 添加新的 PID 参数

```cpp
// main/modules/control/task_control.cpp

static float pid_kp = 1.0f;
static float pid_ki = 0.1f;
static float pid_kd = 0.01f;

void register_control_params() {
    auto& params = ParamRegistry::instance();

    params.register_param("PID_KP", "Control", ParamType::FLOAT,
                          &pid_kp, 0.0f, 10.0f, 1.0f);
    params.register_param("PID_KI", "Control", ParamType::FLOAT,
                          &pid_ki, 0.0f, 5.0f, 0.1f);
    params.register_param("PID_KD", "Control", ParamType::FLOAT,
                          &pid_kd, 0.0f, 1.0f, 0.01f);
}

void pid_control() {
    // 直接使用 RAM 变量
    float output = pid_kp * error +
                   pid_ki * error_integral +
                   pid_kd * error_derivative;
}
```

### 示例 2: 动态调整 EKF 噪声参数

```cpp
// main/modules/estimator/task_estimator.cpp

void adaptive_tuning() {
    auto& params = ParamRegistry::instance();

    // 检测到震动 (加速度计残差大)
    if (accel_residual_norm > 0.5f) {
        // 临时增大加速度计噪声
        ekf_r_accel = 0.3f;
        params.set_param("EKF_R_ACCEL", ekf_r_accel);
        ekf.set_measure_noise_accel(ekf_r_accel);
    }
    // 恢复正常
    else if (accel_residual_norm < 0.1f) {
        ekf_r_accel = 0.05f;
        params.set_param("EKF_R_ACCEL", ekf_r_accel);
        ekf.set_measure_noise_accel(ekf_r_accel);
    }
}
```

### 示例 3: 批量导出/导入参数

```cpp
// 导出所有参数到 JSON
void export_params_to_json() {
    auto& params = ParamRegistry::instance();
    auto all_params = params.get_all_params();

    cJSON* root = cJSON_CreateArray();
    for (auto& p : all_params) {
        cJSON* item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "name", p.name);
        if (p.type == ParamType::FLOAT) {
            float val = *(float*)p.ptr;
            cJSON_AddNumberToObject(item, "value", val);
        }
        cJSON_AddItemToArray(root, item);
    }

    char* json_str = cJSON_Print(root);
    ESP_LOGI(TAG, "Params: %s", json_str);

    free(json_str);
    cJSON_Delete(root);
}
```

## 调试与监控

### 查看参数值

```cpp
// 打印所有参数
void dump_all_params() {
    auto& params = ParamRegistry::instance();
    auto all_params = params.get_all_params();

    ESP_LOGI(TAG, "=== Parameter Dump ===");
    for (auto& p : all_params) {
        if (p.type == ParamType::FLOAT) {
            float val = *(float*)p.ptr;
            ESP_LOGI(TAG, "[%s] %s = %.6f (default: %.6f)",
                     p.group, p.name, val, p.default_val);
        }
    }
}
```

### Web 界面查看

1. 打开 Web 控制台
2. 切换到 **参数管理** 标签
3. 查看所有参数列表
4. 可按分组筛选

## 故障排查

### Q: 参数修改后重启丢失？

**可能原因**:
1. `nvs_commit()` 未调用
2. NVS 分区损坏
3. Flash 写入失败

**解决方法**:
1. 检查 `set_param()` 返回值
2. 擦除 NVS 分区重新初始化
3. 检查 Flash 剩余空间

### Q: 参数读取失败？

**可能原因**:
1. 参数名称拼写错误
2. 参数未注册
3. NVS 键名冲突

**解决方法**:
1. 确认参数名称与注册时一致
2. 检查是否在 `main.cpp` 中调用注册函数
3. 使用 `nvs_partition_tool` 查看分区内容

### Q: 参数值超出范围？

**可能原因**:
1. NVS 中存储的旧值超出 `min_val`/`max_val`
2. 直接修改 RAM 变量绕过了范围检查

**解决方法**:
1. 在 `set_param()` 中增加范围检查
2. 擦除 NVS 重新写入默认值

## 最佳实践

### 1. 参数命名规范

```cpp
// 格式: <MODULE>_<CATEGORY>_<NAME>

// ✅ 好的命名
#define PARAM_EKF_Q_ANGLE     "EKF_Q_ANGLE"
#define PARAM_ACCEL_BIAS_X    "ACCEL_BIAS_X"
#define PARAM_WIFI_SSID       "WIFI_SSID"

// ❌ 不好的命名
#define PARAM_TEMP1           "temp1"        // 不清晰
#define PARAM_BiasX           "BiasX"        // 大小写混用
```

### 2. 合理设置默认值

```cpp
// ✅ 好的默认值 (基于实测或理论计算)
params.register_param("EKF_Q_ANGLE", ..., 0.0005f);  // 经过验证的值

// ❌ 不好的默认值 (随意设置)
params.register_param("EKF_Q_ANGLE", ..., 1.0f);     // 过大，会导致不稳定
```

### 3. 分组管理

```cpp
// 按功能模块分组
params.register_param("EKF_Q_ANGLE", "EKF", ...);          // EKF 滤波器参数
params.register_param("PID_KP", "Control", ...);           // 控制器参数
params.register_param("ACCEL_BIAS_X", "Sensor", ...);     // 传感器校准参数
```

### 4. 范围保护

```cpp
// 设置合理的 min/max 值
params.register_param("EKF_Q_ANGLE", ..., 0.0f, 0.1f, 0.0005f);
//                                    ^^^^  ^^^^
//                                    min   max
```

## 相关文件

- **参数系统**: [components/param_registry/](../components/param_registry/)
- **参数定义**: [components/common/include/shared_types.h](../components/common/include/shared_types.h)
- **Web API**: [components/web_server/web_server.cpp](../components/web_server/web_server.cpp)

## 参考资料

- **ESP-IDF NVS**: ESP-IDF 官方 Non-Volatile Storage API 文档
- **Parameter Server**: ROS 参数服务器设计理念
