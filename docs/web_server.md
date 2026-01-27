# Web 服务器与通信模块 (Web Server Module)

## 概述

Web 服务器模块提供了一个**零依赖的 Web 可视化控制台**，用户可以通过浏览器实时监控设备状态、调整参数和校准传感器。采用 **HTTP + WebSocket** 双协议架构。

## 系统架构

```
┌────────────────────────────────────────────────────────┐
│                   Web Server (Core 0)                  │
│            Priority: 2  |  Stack: 8192                 │
└────────────────────────────────────────────────────────┘
                          │
         ┌────────────────┼────────────────┐
         │                │                │
    ┌────▼────┐     ┌─────▼─────┐    ┌───▼────┐
    │   HTTP  │     │ WebSocket │    │ SPIFFS │
    │  Server │     │   Server  │    │  File  │
    │  (port  │     │  (实时通信) │    │ System │
    │   80)   │     │            │    │        │
    └────┬────┘     └─────┬─────┘    └────────┘
         │                │
         │                └──────> Telemetry Task
         │                        (20Hz 数据推送)
         │
    ┌────▼────┐
    │ Browser │
    │ Client  │
    └─────────┘
```

## 核心组件

### 1. HTTP 服务器

**功能**: 提供静态文件服务和 RESTful API

**端口**: 80

**支持接口**:
- `GET /` - 返回主页 (index.html)
- `GET /api/params` - 获取所有参数
- `POST /api/params` - 更新参数
- `POST /api/reboot` - 重启设备

**文件位置**: `components/web_server/`

### 2. WebSocket 服务器

**功能**: 实时双向通信

**端口**: 80 (与 HTTP 复用)

**消息格式**: JSON 文本

**通信方向**:
- **设备 → 浏览器**: 遥测数据 (20Hz)
- **浏览器 → 设备**: 参数修改指令

### 3. SPIFFS 文件系统

**挂载点**: `/spiffs`

**存储内容**:
- `/index.html` - 主页面
- `/app.js` - 前端逻辑
- `/bootstrap.min.css` - 样式框架
- `/favicon.ico` - 图标

**分区大小**: 2 MB (可在 `partition.csv` 调整)

## 数据通信协议

### 遥测数据 (Device → Browser)

**频率**: 20 Hz (每 50ms 一帧)

**JSON 格式**:
```json
{
  "type": "telem",
  "payload": {
    "roll": 1.23,
    "pitch": 2.34,
    "yaw": 45.67,
    "voltage": 12.34,
    "sys": {
      "heap": 234.5,
      "time": 12345
    },
    "mag": {
      "x": 0.234,
      "y": -0.123,
      "z": 0.456
    },
    "imu": {
      "ax": 0.012,
      "ay": 0.023,
      "az": 9.810,
      "gx": 0.00123,
      "gy": -0.00045,
      "gz": 0.00012
    },
    "ekf": {
      "yaw_measured": 45.67,
      "yaw_predicted": 45.12,
      "yaw_residual": 0.0034,
      "bias": {
        "x": 0.00001,
        "y": 0.00002,
        "z": 0.00003
      },
      "k_yaw_bias_z": 0.002345,
      "accel_used": 1,
      "mag_used": 1,
      "mag_world": {
        "x": 0.234,
        "y": 0.123,
        "z": -0.345
      },
      "P_yaw": 0.001234
    }
  }
}
```

**数据说明**:
- `roll/pitch/yaw`: 姿态角 (度)
- `voltage`: 电池电压 (V)
- `sys.heap`: 剩余堆内存 (KB)
- `sys.time`: 运行时间 (秒)
- `mag`: 磁力计原始数据 (Gauss)
- `imu`: IMU 原始数据 (加速度 m/s², 角速度 rad/s)
- `ekf`: EKF 调试数据 (新增)

**发送代码**: [main/modules/telemetry/task_telemetry.cpp](../main/modules/telemetry/task_telemetry.cpp)

### 参数修改 (Browser → Device)

**请求格式**:
```json
{
  "type": "set_param",
  "param_name": "MAG_BIAS_X",
  "value": 0.123
}
```

**设备响应**:
```json
{
  "type": "param_ack",
  "status": "success",
  "param_name": "MAG_BIAS_X",
  "value": 0.123
}
```

## 前端架构

### 技术栈

- **HTML5**: 页面结构
- **Bootstrap 5**: UI 框架
- **Three.js**: 3D 可视化
- **Chart.js**: 实时波形图 (可选)
- **Vanilla JavaScript**: 逻辑控制

### 主要模块

#### 1. WebSocket 管理器

**文件**: `data/app.js`

**初始化**:
```javascript
const ws = new WebSocket(`ws://${window.location.hostname}/`);
ws.onopen = function() {
    console.log("WebSocket 连接成功");
};
ws.onmessage = function(event) {
    const msg = JSON.parse(event.data);
    handleMessage(msg);
};
```

#### 2. 数据处理中心

```javascript
function handleMessage(msg) {
    if (msg.type === 'telem') {
        updateDashboard(msg.payload);
        updateVectorView(msg.payload);
        updateEKFDebug(msg.payload);
        updateWaveform(msg.payload);
    }
}
```

#### 3. 3D 姿态可视化

**Three.js 场景**:
```javascript
// 飞机模型
const planeGeometry = new THREE.BufferGeometry();
// ... 加载飞机顶点数据

const planeMaterial = new THREE.MeshBasicMaterial({
    color: 0x00ff00,
    wireframe: true
});

const planeMesh = new THREE.Mesh(planeGeometry, planeMaterial);
scene.add(planeMesh);

// 更新姿态
function updatePlaneOrientation(roll, pitch, yaw) {
    const quaternion = new THREE.Quaternion();
    quaternion.setFromEuler(
        new THREE.Euler(pitch, roll, -yaw, 'YXZ')
    );
    planeMesh.quaternion.copy(quaternion);
}
```

#### 4. 向量视图

**坐标系箭头**:
- 红色箭头: X 轴 (前)
- 绿色箭头: Y 轴 (右)
- 蓝色箭头: Z 轴 (下)

**新增 EKF 箭头**:
- 红色箭头 (vecArrowYawMeasured): 磁力计实测航向
- 青色箭头 (vecArrowYawPredicted): 姿态预测航向

```javascript
// 实测航向箭头
vecArrowYawMeasured = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0, 0, 0),
    1.5,
    0xdc3545  // 红色
);

// 预测航向箭头
vecArrowYawPredicted = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0, 0, 0),
    1.5,
    0x0dcaf0  // 青色
);
```

#### 5. EKF 调试面板

**更新逻辑**:
```javascript
function updateEKFDebug(ekf) {
    // 更新航向数据
    document.getElementById("ekf-yaw-measured").innerText =
        ekf.yaw_measured.toFixed(1) + "°";
    document.getElementById("ekf-yaw-predicted").innerText =
        ekf.yaw_predicted.toFixed(1) + "°";

    // 更新残差 (带颜色标识)
    const residualDeg = ekf.yaw_residual * 180.0 / Math.PI;
    const residualElem = document.getElementById("ekf-yaw-residual");
    residualElem.innerText = residualDeg.toFixed(2) + "°";

    if (Math.abs(residualDeg) > 5.0) {
        residualElem.className = "h5 text-danger";
    } else if (Math.abs(residualDeg) > 2.0) {
        residualElem.className = "h5 text-warning";
    } else {
        residualElem.className = "h5 text-success";
    }

    // 更新零偏
    document.getElementById("ekf-bias").innerHTML =
        `X: ${ekf.bias.x.toFixed(5)}<br>` +
        `Y: ${ekf.bias.y.toFixed(5)}<br>` +
        `Z: ${ekf.bias.z.toFixed(5)}`;

    // 更新融合状态徽章
    const accelBadge = document.getElementById("ekf-accel-used");
    accelBadge.className = ekf.accel_used ?
        "badge bg-success" : "badge bg-secondary";
    accelBadge.innerText = ekf.accel_used ? "融合中" : "待机";

    // 更新协方差和卡尔曼增益
    document.getElementById("ekf-p-yaw").innerText = ekf.P_yaw.toFixed(6);
    document.getElementById("ekf-k-yaw-bias-z").innerText =
        ekf.k_yaw_bias_z.toFixed(6);

    // 异常警告
    const statusBadge = document.getElementById("ekf-status");
    if (residualAbs > 10.0 || pYaw > 1.0) {
        statusBadge.className = "badge bg-danger";
        statusBadge.innerText = "严重异常";
    } else if (residualAbs > 5.0 || pYaw > 0.1) {
        statusBadge.className = "badge bg-warning";
        statusBadge.innerText = "警告";
    } else {
        statusBadge.className = "badge bg-success";
        statusBadge.innerText = "正常";
    }
}
```

## 使用方法

### 访问控制台

1. **连接 WiFi**:
   - 设备自动连接已配置的路由器
   - 或连接设备 AP: `EspCortex` (密码: `12345678`)

2. **打开浏览器**:
   ```
   http://espcortex.local
   或
   http://<device-ip>
   ```

3. **查看状态**:
   - 仪表盘: 实时姿态、传感器数据
   - 向量视图: 3D 可视化
   - EKF 调试: 融合状态监控
   - 参数管理: 在线调参
   - 传感器标定: 校准向导

### 修改参数

1. 打开 **参数管理** 标签
2. 找到目标参数 (如 `MAG_BIAS_X`)
3. 输入新值
4. 点击 **修改** 按钮
5. 参数立即生效并自动保存至 NVS

### 传感器校准

**加速度计六面校准**:
1. 打开 **传感器标定** 标签
2. 点击 **开始加速度计校准**
3. 按照 3D 飞机图标指示，依次完成六面放置
4. 校准完成后参数自动保存

**磁力计椭球拟合**:
1. 点击 **开始磁力计校准**
2. 缓慢旋转设备 (画 8 字)
3. 观察进度条，收集足够样本
4. 点击 **完成校准**

## 性能优化

### WebSocket 性能

**当前配置**:
- 发送频率: 20 Hz
- JSON 大小: ~600 字节/帧
- 带宽占用: ~12 KB/s

**优化建议**:
- 降低频率: 修改 `vTaskDelay(pdMS_TO_TICKS(50))` → `vTaskDelay(pdMS_TO_TICKS(100))` (10 Hz)
- 压缩数据: 移除不必要的字段

### SPIFFS 性能

**缓存策略**:
```cpp
// components/web_server/web_server.cpp

static const char* index_html_cache = nullptr;

void serve_index(httpd_req_t *req) {
    if (index_html_cache == nullptr) {
        // 首次读取并缓存
        // ...
    }
    httpd_send(req, index_html_cache, strlen(index_html_cache));
}
```

## 故障排查

### Q: 无法访问 Web 界面？

**检查清单**:
1. 设备是否成功连接 WiFi (查看串口输出)
2. 浏览器输入 IP 是否正确
3. 尝试访问 `http://espcortex.local`
4. 检查防火墙是否拦截

### Q: WebSocket 连接失败？

**可能原因**:
1. 浏览器不支持 WebSocket
2. 网络中断
3. 设备内存不足

**解决方法**:
1. 更新浏览器 (Chrome/Firefox/Edge)
2. 检查 `sys.heap` 是否充足 (> 100 KB)
3. 重启设备

### Q: 数据不更新？

**检查清单**:
1. 打开浏览器控制台 (F12)，查看是否有报错
2. 检查 `ws.onmessage` 是否正常触发
3. 查看串口是否有 Telemetry 日志
4. 确认 JSON 格式正确

### Q: 3D 视图不显示？

**可能原因**:
1. WebGL 不支持
2. Three.js 加载失败

**解决方法**:
1. 更新显卡驱动
2. 检查浏览器控制台错误
3. 确认 `app.js` 正确加载

## 扩展开发

### 添加新的遥测数据

**后端 (task_telemetry.cpp)**:
```cpp
int len = snprintf(json_buffer, sizeof(json_buffer),
    "{\"type\":\"telem\",\"payload\":{"
    "\"new_field\":%.2f"  // 添加新字段
    "}}",
    new_value
);
```

**前端 (app.js)**:
```javascript
function updateDashboard(payload) {
    if (payload.new_field !== undefined) {
        document.getElementById("new-field").innerText =
            payload.new_field.toFixed(2);
    }
}
```

### 添加新的 REST API

**后端 (web_server.cpp)**:
```cpp
static esp_err_t my_api_handler(httpd_req_t *req) {
    // 解析请求
    // ...
    // 返回响应
    const char* resp = "{\"status\":\"ok\"}";
    httpd_send(req, resp, strlen(resp));
    return ESP_OK;
}

// 注册 URI
{
    uri = "/api/my_endpoint",
    method = HTTP_POST,
    handler = my_api_handler
}
```

**前端调用**:
```javascript
fetch("/api/my_endpoint", {
    method: "POST",
    body: JSON.stringify({key: "value"})
}).then(res => res.json())
.then(data => console.log(data));
```

## 相关文件

- **Web 服务器**: [components/web_server/](../components/web_server/)
- **遥测任务**: [main/modules/telemetry/task_telemetry.cpp](../main/modules/telemetry/task_telemetry.cpp)
- **前端页面**: [data/index.html](../data/index.html)
- **前端逻辑**: [data/app.js](../data/app.js)
- **文件系统**: [components/spiffs_manager/](../components/spiffs_manager/)

## 参考资料

- **ESP-IDF HTTP Server**: ESP-IDF 官方文档
- **WebSocket Protocol**: RFC 6455
- **Three.js Documentation**: https://threejs.org/docs/
- **Bootstrap 5**: https://getbootstrap.com/docs/5.3/
