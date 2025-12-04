idf_component_register(SRCS "main.cpp" "modules/sensor/task_sensor.cpp"
                       INCLUDE_DIRS "." "include" "modules/sensor"
                       REQUIRES spi_bus icm42688 i2c_bus qmc5883l icp20100 param_registry wifi_manager spiffs_manager web_server) # 增加依赖

spiffs_create_partition_image(storage ../data FLASH_IN_PROJECT)