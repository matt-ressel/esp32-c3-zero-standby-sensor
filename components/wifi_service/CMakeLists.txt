# CMakeLists.txt for WiFi Service component
# This component initializes and configures the WiFi stack,
# which is a prerequisite for using ESP-NOW communication.
idf_component_register(SRCS "wifi_service.c" INCLUDE_DIRS "." PRIV_REQUIRES esp_wifi esp_event)