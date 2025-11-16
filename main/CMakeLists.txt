# CMakeLists.txt for main application
# This file defines the main application component and its dependencies.

idf_component_register( SRCS "main.c" INCLUDE_DIRS "." PRIV_REQUIRES battery_monitor bme280_driver espnow_sender nvs_flash power_manager sensor_manager wifi_service )
