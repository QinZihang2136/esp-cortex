# 注册组件
idf_component_register(
    SRCS "param_registry.cpp"       # 源文件
    INCLUDE_DIRS "include"          # 头文件目录
    # 依赖项：
    # nvs_flash: 用于存储
    # esp_common: 包含基础类型
    # log: 用于打印日志
    # freertos: 用于互斥锁
    REQUIRES nvs_flash esp_common log freertos
)