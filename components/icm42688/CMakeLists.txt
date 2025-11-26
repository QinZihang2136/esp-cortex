# 声明该组件依赖 spi_bus 组件
idf_component_register(SRCS "icm42688.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES spi_bus driver log)