
set(requirements radioLib esp_timer driver espressif__arduino-esp32)

idf_component_register(SRCS sources/ev_loraphy.cpp sources/ev_lorawan.cpp
                       INCLUDE_DIRS includes
                       REQUIRES ${requirements})