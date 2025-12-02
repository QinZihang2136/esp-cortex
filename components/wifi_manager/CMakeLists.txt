idf_component_register(SRCS "wifi_manager.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_wifi nvs_flash esp_event driver)