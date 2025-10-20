/**
 * @file bme280_driver.h
 * @author Mateusz Ressel (https://github.com/matt-ressel)
 *
 * @brief Public interface for a minimal BME280 sensor driver.
 *
 * This module provides a lightweight, from-scratch implementation for reading
 * temperature, pressure, and humidity from a BME280 sensor, optimized for
 * low-power, single-shot measurements.
 *
 * @version 0.1
 * @date    2025-10-20
 *
 * @copyright Copyright (c) 2025 Mateusz Ressel. Licensed under the MIT License.
 *
 */

#ifndef BME280_DRIVER_H
#define BME280_DRIVER_H

#include "esp_err.h"

/**
 * @brief Structure to hold all compensated sensor readings.
 */
typedef struct {
  float temperature; /**< Temperature in degrees Celsius (°C) */
  float pressure;    /**< Pressure in hectoPascals (hPa) */
  float humidity;    /**< Relative humidity in percent (%RH) */
} bme280_data_t;
/**
 * @brief Initializes the BME280 sensor and the underlying I2C bus.
 *
 * This function performs the complete initialization sequence using the new I2C driver API
 * 1. Creates an I2C master bus handle.
 * 2. Adds the BME280 as a device on the bus, creating a device handle.
 * 3. Verifies the BME280 Chip ID (should be 0x60) to confirm communication.
 * 4. Reads all factory calibration coefficients (T, P, H) from the sensor's NVM.
 * 5. Configures the sensor for low-power operation (forced mode, minimal oversampling).
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t bme280_driver_init(void);

/**
 * @brief De-initializes the BME280 driver and releases I2C resources.
 *
 * This function removes the BME280 device from the bus and then deletes the
 * I2C master bus handle, freeing all allocated resources.
 *
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bme280_driver_deinit(void);

/**
 * @brief Performs a single measurement of all values and returns the compensated results.
 *
 * Triggers a measurement in "Forced Mode", waits, reads raw ADC values, and applies
 * factory calibration data to compute the final values.
 *
 * @param[out] data Pointer to a bme280_data_t struct where the results will be stored.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t bme280_driver_read_data(bme280_data_t* data);

#endif  // BME280_DRIVER_H