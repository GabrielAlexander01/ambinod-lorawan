
set(requirements sensorMHZ19B sensorDHT22 sensorGUVAS12SD
                 sensorMAX4466 evLora esp_hw_support)

idf_component_register(SRCS sources/ev_sensors.c
                       INCLUDE_DIRS includes
                       REQUIRES ${requirements})
