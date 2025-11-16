/**
 * @file main.c
 * @author Mateusz Ressel (https://github.com/matt-ressel)
 *
 * @brief Main application entry point for the ESP32-C3 Zero Standby Sensor.
 *
 * This file contains the `app_main` function, which serves as the top-level
 * orchestrator for the sensor's lifecycle. Its primary role is to initialize
 * all system services and components in a highly optimized order, execute the
 * main operational cycle by invoking the appropriate managers, and finally,
 * trigger the system power-down.
 *
 * @version 1.0
 * @date    2025-11-09
 *
 * @copyright Copyright (c) 2025 Mateusz Ressel. Licensed under the MIT License.
 *
 */

// ESP-IDF includes
#include "esp_err.h"    // For ESP_OK and other standard error codes from ESP-IDF
#include "esp_log.h"    // For ESP-IDF logging utilities (ESP_LOGI, ESP_LOGW, ESP_LOGE)
#include "esp_sleep.h"  // For deep sleep functions and types
#include "nvs_flash.h"  // For Non-Volatile Storage, a prerequisite for the WiFi stack

// Project specific module includes
#include "battery_monitor.h"  // For measuring battery voltage
#include "bme280_driver.h"    // For acquiring environmental data
#include "espnow_sender.h"    // For ESP-NOW data transmission
#include "power_manager.h"    // For controlling the shutdown signal to the TPL5111
#include "sensor_manager.h"   // For orchestrating the sensor operations
#include "wifi_service.h"     // For WiFi stack initialization (prerequisite for ESP-NOW)

/** @brief Logging tag specifically for messages originating from this main application file. */
static const char* TAG_MAIN_APP = "MAIN_APP";

/**
 * @brief Main application entry point and orchestrator of the sensor's operational sequence.
 *
 * The sequence of operations in this function is critically optimized to achieve
 * two primary goals:
 * 1. To perform the temperature-sensitive BME280 measurement as early as possible
 *    after power-on, minimizing the impact of CPU self-heating.
 * 2. To hide the latency of physical processes (like capacitor charging for the
 *    battery measurement) behind other necessary, long-running initializations
 *    (like the Wi-Fi stack).
 *
 * @par Phase 1: Prerequisites and Critical Driver Initialization
 * Initializes NVS (a mandatory prerequisite for Wi-Fi) and the core hardware
 * drivers that need to be ready for immediate action.
 *
 * @par Phase 2: Optimized, Time-Sensitive Operations
 * This is the most time-critical section. It starts the physical battery
 * measurement process (which requires a ~25ms stabilization delay) and then
 * *immediately* performs the BME280 digital measurement. This ensures the
 * temperature reading is captured before the CPU has a chance to significantly heat up.
 *
 * @par Phase 3: Non-Critical Initializations
 * While the filter capacitor for the battery measurement is physically stabilizing,
 * the CPU is productively used to initialize the time-consuming network stack
 * (Wi-Fi and ESP-NOW). This effectively "hides" the 25ms battery delay,
 * reducing the total system "on-time".
 *
 * @par Phase 4: Main Logic Execution
 * Control is passed to the `sensor_manager`. It receives the pre-measured BME280
 * data and is responsible for the rest of the business logic: finalizing the
 * battery measurement, assembling the data payload, and transmitting it.
 *
 * @par Phase 5: System Shutdown
 * The final step is to signal the external timer via the `power_manager` to cut
 * system power. A fail-safe mechanism (deep sleep) is included in the unlikely
 * event that the shutdown signal fails.
 *
 * @note Note how `main.c` acts as a high-level "CEO", managing the *lifecycle*
 *       and *sequence*, while the `sensor_manager` acts as a "Manager", handling
 *       the *business logic* of processing and sending data. This separation of
 *       concerns is a key architectural feature of this project.
 */

void app_main(void) {
  bme280_data_t sensor_data;  // Structure to hold BME280 measurement data

  ESP_LOGI(TAG_MAIN_APP, "--- ESP32-C3 Zero Standby Sensor: Cold Boot ---");

  // --- System Prerequisites Initialization ---
  // --- Phase 1: Prerequisites and Critical Driver Initialization ---
  ESP_LOGI(TAG_MAIN_APP, "Initializing prerequisites and critical drivers...");
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK_WITHOUT_ABORT(power_manager_init());    // Initialize power manager
  ESP_ERROR_CHECK_WITHOUT_ABORT(bme280_driver_init());    // Initialize BME280 driver
  ESP_ERROR_CHECK_WITHOUT_ABORT(battery_monitor_init());  // Initialize battery monitor

  // --- Phase 2: Optimized, Time-Sensitive Operations --
  ESP_LOGI(TAG_MAIN_APP, "Starting latency-hiding and time-critical measurements...");
  // Start the physical process of battery measurement (capacitor charging).
  // The ~25ms stabilization time will now pass in the background.
  ESP_ERROR_CHECK_WITHOUT_ABORT(battery_monitor_start_measurement());

  // Immediately perform the BME280 measurement to get the purest temperature reading
  ESP_LOGI(TAG_MAIN_APP, "Performing immediate BME280 measurement...");
  ESP_ERROR_CHECK_WITHOUT_ABORT(bme280_driver_read_data(&sensor_data));

  // --- Phase 3: Non-Critical Initializations ---
  ESP_LOGI(TAG_MAIN_APP, "Initializing network stack (hiding battery measurement latency)...");
  ESP_ERROR_CHECK_WITHOUT_ABORT(wifi_service_init());
  ESP_ERROR_CHECK_WITHOUT_ABORT(espnow_sender_init());

  // --- Phase 4: Main Logic Execution ---
  // ESP_LOGI(TAG_MAIN_APP, "All initializations complete. Running the sensor manager...");
  ESP_ERROR_CHECK_WITHOUT_ABORT(sensor_manager_run(&sensor_data));

  // --- Phase 5: System Shutdown ---
  ESP_LOGI(TAG_MAIN_APP, "Signaling shutdown...");
  // Signal the TPL5111 to cut power. This is the final action.
  power_manager_shutdown();

  // --- Fail-safe: This code should be unreachable ---
  // If we are here, it means the primary shutdown signal to the TPL5111 failed.
  // As a last resort, we enter deep sleep to minimize power consumption.
  ESP_LOGE(TAG_MAIN_APP, "CRITICAL: Shutdown signal failed! Entering deep sleep as a fail-safe.");
  // Disable all wake-up sources to ensure the device stays in deep sleep
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

  // We do not configure a wake-up timer. The external TPL5111 will perform a
  // hard power-cycle after its ~20 minute interval, which will restart the system.
  // This call puts the device into the lowest possible power state indefinitely.
  esp_deep_sleep_start();
}
