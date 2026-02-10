# Telemetry 30Hz 更新说明

## 1. 更新目标

本次更新目标是解决网页端“刷新卡顿”问题，在保证稳定性的前提下提升交互流畅度：

- Telemetry 推流从 20Hz 提升到 30Hz
- Dashboard 前端渲染节流同步提升到约 30FPS
- 增加发送链路保护机制，避免提频后不稳定

## 2. 关键改动

### 2.1 Telemetry 推流提频

- 发送循环由固定 50ms（20Hz）调整为按目标频率动态计算周期
- 默认目标频率 `TELM_RATE_HZ_DASH = 30`

### 2.2 自动回退保护

新增自动回退机制（默认启用）：

- 连续 3 秒检测到异常：
  - 发送失败 (`send_fail > 0`) 或
  - 发送耗时峰值过高 (`send_us_max > 20000us`)
- 自动降到安全频率 `TELM_RATE_HZ_SAFE`（默认 20Hz）
- 连续 5 秒稳定后自动恢复到目标频率

### 2.3 前端节流对齐

- Dashboard 渲染节流由 `80ms` 调整为 `33ms`
- 避免出现“后端已提频、前端仍低频渲染”的瓶颈

### 2.4 诊断信息增强

`payload.ekf.tx` 新增字段：

- `target_hz`：当前目标频率
- `active_hz`：当前实际生效频率（可能处于回退）
- `fallback`：是否处于回退（0/1）

## 3. 相关参数

可在参数页面在线修改：

- `TELM_RATE_HZ_DASH`（默认 30）
- `TELM_RATE_HZ_SAFE`（默认 20）
- `TELM_AUTO_FALLBACK_EN`（默认 1）

建议起步配置：

- `TELM_RATE_HZ_DASH = 30`
- `TELM_RATE_HZ_SAFE = 20`
- `TELM_AUTO_FALLBACK_EN = 1`

## 4. 验证方法

## 4.1 串口日志

观察 `TELEMETRY` 日志：

```text
[TX] rate=29.8Hz target/active=30.0/30.0Hz fb=0 ok/fail=30/0 ...
```

关注项：

- `rate` 是否稳定在 `28~31Hz`
- `ok/fail` 是否保持 `fail=0`
- `fb` 是否长期为 `0`（正常工况）

## 4.2 网页端

- Dashboard 动画与数值更新更连贯
- 浏览器控制台 `[PERF]` 中 `ws` 与 `fps(dash)` 匹配改善

## 4.3 压测观察

当网络瞬时变差或浏览器忙时，若触发回退：

- `target/active` 出现 `30/20`
- `fb=1`

恢复后应自动回到 `fb=0`。

## 5. 已知边界

- Monitor 页面的 Plotly/3D 更新频率仍独立节流，当前未随 Dashboard 一起提频。
- 若后续继续提频（>30Hz），建议先做按页面消息分层，避免无效高频数据占用链路。
