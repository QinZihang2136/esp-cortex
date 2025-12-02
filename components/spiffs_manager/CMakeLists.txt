idf_component_register(SRCS "spiffs_manager.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES spiffs log)     # <--- 改为 spiffs