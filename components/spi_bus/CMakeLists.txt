# 注册组件
# SRCS: 源文件列表
# INCLUDE_DIRS: 头文件目录
# REQUIRES: 依赖的系统组件 (driver用于SPI/GPIO, esp_timer用于计时)
idf_component_register(SRCS "spi_bus.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES driver esp_timer)