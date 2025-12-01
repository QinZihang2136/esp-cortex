idf_component_register(SRCS "main.cpp" "modules/sensor/task_sensor.cpp"
                       INCLUDE_DIRS "." "include" "modules/sensor"
                       REQUIRES spi_bus icm42688 i2c_bus qmc5883l icp20100 param_registry) # 增加依赖