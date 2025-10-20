/**
 * @file sensor_manager.h
 * @author Mateusz Ressel (https://github.com/matt-ressel)
 *
 * @brief Public interface for the main sensor manager.
 *
 * This module orchestrates the primary operational sequence of the sensor:
 * acquiring data from all relevant drivers, packaging it, and transmitting it.
 *
 * @version 0.1
 * @date    2025-10-20
 *
 * @copyright Copyright (c) 2025 Mateusz Ressel. Licensed under the MIT License.
 *
 */

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "bme280_driver.h"  // bme280_data_t type
#include "esp_err.h"        // esp_err_t type

/**
 * @brief Executes one complete sensor operational cycle.
 *
 * This function performs the entire sequence of tasks for a single wake-up period:
 * 1. Reads data from the BME280 sensor (temperature, pressure, humidity).
 * 2. Measures the current battery voltage.
 * 3. Assembles the data into a standardized payload structure.
 * 4. Calculates a CRC8 checksum for data integrity.
 * 5. Transmits the final payload via ESP-NOW and waits for acknowledgement.
 *
 * @return esp_err_t
 *   - ESP_OK: If the entire cycle, including successful transmission, completed correctly.
 *   - An error code from one of the underlying modules on failure.
 */
esp_err_t sensor_manager_run(const bme280_data_t* sensor_data);

#endif  // SENSOR_MANAGER_H