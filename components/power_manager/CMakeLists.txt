# CMakeLists.txt for Power Manager component
# This component handles power management tasks such as controlling
# the shutdown signal to the TPL5111 timer to optimize power consumption.
idf_component_register(SRCS "power_manager.c" INCLUDE_DIRS "." PRIV_REQUIRES driver)