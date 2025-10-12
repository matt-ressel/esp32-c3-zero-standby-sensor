/**
 * @file main.c
 * @brief Main application entry point for the ESP32-C3-ZERO-STANDBY-SENSOR.
 *
 * This file contains the `app_main` function, which is the starting point for
 * the sensor's firmware. The function executes a fast, linear sequence of
 * operations: initialize hardware, perform measurements, transmit data, and
 * signal the external timer to cut power. The entire process is optimized for
 * minimal "on-time" to achieve multi-year battery life.
 */

// ESP-IDF Basic System Includes
#include "esp_err.h"    // For ESP_OK and other standard error codes from ESP-IDF
#include "esp_log.h"    // For ESP-IDF logging utilities (ESP_LOGI, ESP_LOGW, ESP_LOGE)
#include "esp_sleep.h"  // For deep sleep functions and types
#include "nvs_flash.h"  // For Non-Volatile Storage, a prerequisite for the WiFi stack

// Project-Specific Module Includes
// #include "battery_monitor.h"     // For measuring battery voltage
// #include "bme280_sensor.h"       // For acquiring environmental data
#include "espnow_transmitter.h"  // For ESP-NOW data transmission
// #include "power_manager.h"       // For controlling the shutdown signal to the TPL5111
// #include "shared_data_types.h"   // For the sensor_data_t payload structure
// #include "utils/crc.h"           // For calculating the CRC8 checksum
#include "wifi.h"  // For WiFi stack initialization (prerequisite for ESP-NOW)

/** @brief Logging tag specifically for messages originating from this main application file. */
static const char* TAG_MAIN_APP = "MAIN_APP";

/**
 * @brief Main application entry point and system initialization sequence.
 *
 * This function coordinates the entire sensor lifecycle during its active period
 * It is designed to execute as quickly as possible before signaling for a complete
 * power-down. The order of operations is critical.
 *
 * @par NVS Initialization
 * Initializes the Non-Volatile Storage. This is a mandatory prerequisite for the
 * WiFi driver, which stores calibration data here.
 *
 * @par Hardware Driver Initialization
 * Initializes all necessary hardware peripherals: I2C for the BME280 sensor,
 * ADC for battery measurement, and GPIO pins for controlling the power manager
 * and the battery measurement voltage divider.
 *
 * @par Data Acquisition
 * First, it reads the temperature from the BME280. This is done immediately
 * after startup to get a reading before the ESP32-C3 core begins to self-heat.
 * Second, it measures the battery voltage using the switched voltage divider.
 *
 * @par Communication Interfaces
 * Initializes the WiFi stack in a minimal configuration and then initializes the
 * ESP-NOW transmitter module, preparing it for sending data.
 *
 * @par Data Transmission
 * The acquired sensor and battery data is packaged into a `sensor_data_t`
 * structure. A CRC8 checksum is calculated and appended to ensure data integrity.
 * The complete payload is then sent via ESP-NOW.
 *
 * @par System Shutdown
 * After the data transmission is confirmed, the function de-initializes the
 * network stack and then calls the power manager to send a "DONE" signal to
 * the external TPL5111 timer, which immediately cuts power to the entire system.
 * This is the final action of the function. If this step fails, the function
 * will enter deep sleep mode as a fail-safe.
 *
 * @note `ESP_ERROR_CHECK()` is used for critical initialization steps. A failure
 *       will halt the device, which will then be power-cycled by the TPL5111
 *       for another attempt after the configured interval.
 */

void app_main(void) {
  ESP_LOGI(TAG_MAIN_APP, "--- ESP32-C3-ZERO-STANDBY-SENSOR: Cold Boot ---");

  // --- System Prerequisites Initialization ---
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // ---  Hardware Drivers Initialization ---
  ESP_LOGI(TAG_MAIN_APP, "Initializing hardware drivers...");
  // ESP_ERROR_CHECK(power_manager_init());    // GPIO for TPL5111 'DONE' signal
  // ESP_ERROR_CHECK(battery_monitor_init());  // GPIO for ADC enable and ADC itself
  // ESP_ERROR_CHECK(bme280_sensor_init());    // I2C for BME280 sensor

  // --- Data Acquisition ---
  ESP_LOGI(TAG_MAIN_APP, "Starting data collection...");
  // ESP_ERROR_CHECK(bme280_sensor_read(&payload));    // Read temperature and humidity
  // ESP_ERROR_CHECK(battery_monitor_read(&payload));  // Read battery voltage

  // --- Communication Interfaces Initialization ---
  ESP_LOGI(TAG_MAIN_APP, "Initializing communication interfaces...");
  ESP_ERROR_CHECK(wifi_init());                // WiFi logs its status
  ESP_ERROR_CHECK(espnow_transmitter_init());  // ESP-NOW logs its status

  // --- Data Transmission ---
  ESP_LOGI(TAG_MAIN_APP, "Preparing and sending data via ESP-NOW...");
  // ESP_ERROR_CHECK(espnow_transmitter_send(&payload));  // ESP_LOGI(TAG_MAIN_APP, "Data sent successfully via ESP-NOW.");

  // --- System Shutdown ---
  ESP_LOGI(TAG_MAIN_APP, "De-initializing network and shutting down...");


  // Signal the TPL5111 to cut power. This is the final action.
  // power_manager_shutdown();

  // --- This part of the code should be unreachable ---
  // If we are here, it means the shutdown signal to the TPL5111 failed.
  // Instead of a while(1) loop, we implement Plan B: Deep Sleep.
  ESP_LOGE(TAG_MAIN_APP, "CRITICAL: Shutdown signal failed! Entering deep sleep as a fail-safe.");

  // Configure wakeup time to be similar to the TPL5111's interval.
  // This gives the system another chance after the nominal sleep period.
  // const uint64_t uS_TO_S_FACTOR = 1000000;
  // const uint64_t WAKEUP_TIME_SEC = 20 * 60;  // Wake up in ~20 minutes.

  // esp_sleep_enable_timer_wakeup(WAKEUP_TIME_SEC * uS_TO_S_FACTOR);

  // // Enter deep sleep mode.
  // esp_deep_sleep_start();
}
