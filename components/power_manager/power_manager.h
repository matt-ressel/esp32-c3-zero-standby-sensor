/**
 * @file power_manager.h
 * @author Mateusz Ressel (https://github.com/matt-ressel)
 *
 * @brief Public interface for the Power Manager component.
 *
 * This component is responsible for controlling the external power-gating
 * mechanism, specifically the TPL5111 system timer. Its primary role is to
 * signal the timer when the main application cycle is complete, thereby
 * triggering a full system power-down.
 *
 * @version 0.2
 * @date    2025-11-11
 *
 * @copyright Copyright (c) 2025 Mateusz Ressel. Licensed under the MIT License.
 */

#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include "esp_err.h"

/**
 * @brief Initializes the Power Manager component.
 *
 * Configures the GPIO pin connected to the TPL5111's 'DONE' input as an output,
 * ensuring it is in a safe, low state at startup.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t power_manager_init(void);

/**
 * @brief De-initializes the Power Manager and releases its resources.
 *
 * Resets the 'DONE' GPIO pin to its default state.
 *
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t power_manager_deinit(void);

/**
 * @brief Signals the external timer to cut system power.
 *
 * This function sends a pulse to the 'DONE' pin of the TPL5111.
 * @note In normal, successful operation, this function SHOULD NEVER RETURN,
 *       as the system power will be cut. A return of any value indicates a
 *       critical hardware or state failure.
 *
 * @return esp_err_t
 *       - ESP_FAIL if the function returns, indicating a hardware fault.
 */
esp_err_t power_manager_shutdown(void);

#endif  // POWER_MANAGER_H