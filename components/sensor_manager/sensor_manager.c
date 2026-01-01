/**
 * @file sensor_manager.c
 * @author Mateusz Ressel (https://github.com/matt-ressel)
 *
 * @brief Implementation of the main sensor manager.
 *
 * This module acts as an orchestrator. It receives pre-measured critical data,
 * finalizes other measurements, assembles the complete data payload, and hands
 * it over to the network service for transmission.
 *
 * @version 0.2
 * @date    2025-11-15
 *
 * @copyright Copyright (c) 2025 Mateusz Ressel. Licensed under the MIT License.
 *
 */

#include "sensor_manager.h"

// ESP-IDF includes
#include "esp_err.h"  // esp_err_t type and error codes
#include "esp_log.h"  // Logging functions

// Specific component includes
#include "battery_monitor.h"  // Driver for battery measurement
#include "bme280_driver.h"    // Driver for the BME280 sensor
#include "crc8.h"             // CRC calculation utility
#include "espnow_sender.h"    // ESP-NOW sender module
#include "sensor_data.h"      // Definition of the sensor_data_t structure

// Logging tag for this module
static const char* TAG = "SENSOR_MANAGER";

// Implementation of the sensor_manager_run function
esp_err_t sensor_manager_run(const bme280_data_t* sensor_data) {
  ESP_LOGI(TAG, "Starting new sensor management cycle...");
  esp_err_t ret;

  // 1. Prepare data structures
  sensor_data_t payload = {0};

  // 2. Process the pre-measured BME280 data passed as an argument
  if (sensor_data != NULL) {
    payload.temperature = (int16_t)(sensor_data->temperature);        // DegC * 100
    payload.pressure = (uint16_t)(sensor_data->pressure / 256 / 10);  // Pa -> hPa * 10
    payload.humidity = (uint8_t)(sensor_data->humidity / 1024);       // %RH * 1024 -> %RH
    payload.air_quality = -1;                                         // Indicate no air quality data available
    ESP_LOGI(TAG, "Processed BME280 Data: T=%d.%d °C, P=%u.%u hPa, H=%u %%RH",
             payload.temperature / 10, payload.temperature % 10,
             payload.pressure / 10, payload.pressure % 10,
             payload.humidity);
  } else {
    ESP_LOGE(TAG, "Invalid BME280 data pointer. Using fallback error values.");
    payload.temperature = -127.0f;
  }

  uint16_t temp_battery_mv = 0;  // Temporary variable for battery voltage

  // 3. Finalize the battery measurement
  // This function is called after the ~25ms stabilization period has already
  // passed, hidden behind the Wi-Fi initialization in main.c.
  ret = battery_monitor_get_measurement(&temp_battery_mv);
  payload.battery_mv = temp_battery_mv;  // Assign the measured battery voltage
  // Log the battery voltage or an error if measurement failed
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to get battery measurement. Using fallback value.");
    payload.battery_mv = 0;
  }
  ESP_LOGI(TAG, "Finalized Battery Read: V_Batt=%u mV", payload.battery_mv);

  // 4. Calculate CRC for the payload
  payload.crc = crc8((uint8_t*)&payload, sizeof(sensor_data_t) - 1);
  ESP_LOGI(TAG, "Calculated CRC: 0x%02X", payload.crc);

  // 5. Transmit the data via ESP-NOW
  ESP_LOGI(TAG, "Handing over payload to ESP-NOW sender...");
  ret = espnow_sender_transmit((const uint8_t*)&payload, sizeof(payload));
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Transmission successful.");
  } else {
    ESP_LOGE(TAG, "Transmission failed with error: %s", esp_err_to_name(ret));
  }

  // Return the final status of the transmission
  return ret;
}