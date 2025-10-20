# CMakeLists.txt for BME280 driver component
# This component provides the driver for the BME280 sensor.
idf_component_register(SRCS "bme280_driver.c" INCLUDE_DIRS "." PRIV_REQUIRES driver)