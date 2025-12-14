idf_component_register(SRCS "main.cpp" "modules/sensor/task_sensor.cpp" 
                            "modules/telemetry/task_telemetry.cpp"
                            "modules/estimator/task_estimator.cpp"  # 增加 EKF 任务源文件
                       INCLUDE_DIRS "." "include" 
                                        "modules/sensor" 
                                        "modules/telemetry"
                                        "modules/estimator"
                       REQUIRES spi_bus icm42688 i2c_bus qmc5883l icp20100 lps22hh param_registry wifi_manager spiffs_manager web_server common ekf ) # 增加依赖

spiffs_create_partition_image(storage ../data FLASH_IN_PROJECT)