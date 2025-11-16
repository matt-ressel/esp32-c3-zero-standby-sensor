/**
 * @file battery_monitor.h
 * @author Mateusz Ressel (https://github.com/matt-ressel)
 *
 * @brief Public interface for the battery monitoring component.
 *
 * This component abstracts the process of measuring the battery voltage via a
 * switched voltage divider. It uses a two-step measurement process to allow
 * the required stabilization delay to be hidden behind other, long-running
 * operations like Wi-Fi initialization.
 *
 * @version 0.1
 * @date    2025-10-28
 *
 * @copyright Copyright (c) 2025 Mateusz Ressel. Licensed under the MIT License.
 *
 */

#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <stdint.h>

#include "esp_err.h"

/**
 * @brief Initializes the battery monitor component.
 *
 * Configures the GPIO for controlling the voltage divider's load switch and
 * initializes the ADC peripheral for reading the voltage.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t battery_monitor_init(void);

/**
 * @brief De-initializes the battery monitor and releases all resources.
 */
esp_err_t battery_monitor_deinit(void);

/**
 * @brief Starts the battery measurement process (non-blocking).
 *
 * This function enables the external voltage divider by setting the control GPIO
 * to HIGH. This allows the filter capacitor on the ADC input to begin charging.
 * This function should be called as early as possible to hide the stabilization
 * latency behind other operations. It returns immediately.
 *
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t battery_monitor_start_measurement(void);

/**
 * @brief Finalizes the measurement, returns the result, and disables the divider.
 *
 * This function performs the ADC conversion on the now-stabilized voltage from
 * the divider. It then immediately disables the voltage divider to conserve power.
 *
 * @note This function MUST be called after a sufficient delay has passed since
 *       battery_monitor_start_measurement() was called (typically >25ms for
 *       a 100kΩ/0.1µF RC filter).
 *
 * @param[out] voltage_mv Pointer to a variable where the final battery voltage
 *                        in millivolts will be stored.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t battery_monitor_get_measurement(uint16_t* voltage_mv);

#endif  // BATTERY_MONITOR_H