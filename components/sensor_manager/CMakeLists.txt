# CMakeLists.txt for Sensor Manager component
# This component manages sensor data acquisition and processing.
idf_component_register(SRCS "sensor_manager.c" INCLUDE_DIRS "." PRIV_REQUIRES _shared battery_monitor bme280_driver espnow_sender z_utils)