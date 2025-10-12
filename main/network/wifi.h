/**
 * @file wifi.h
 * @brief Public interface for the WiFi initialization module.
 *
 * This module provides the necessary functions to initialize and configure
 * the WiFi stack on the ESP32, specifically for enabling ESP-NOW communication.
 */

#ifndef WIFI_H
#define WIFI_H

#include "esp_err.h"  // For esp_err_t error codes (e.g., ESP_OK)

/**
 * @brief Initializes the WiFi stack for ESP-NOW communication.
 *
 * This function performs the essential setup of the WiFi hardware and software
 * stack to enable ESP-NOW operation. This typically involves:
 * - Initializing the network interface (netif).
 * - Creating the default system event loop.
 * - Initializing the WiFi driver.
 * - Configuring the WiFi mode (usually Station mode for ESP-NOW).
 * - Starting the WiFi stack.
 * - Setting the specific WiFi channel for ESP-NOW.
 * - Ensuring the WiFi bandwidth is set to HT20.
 * - Configuring the WiFi protocol.
 *
 * ESP-NOW requires the WiFi stack to be active, even if the device does not
 * connect to an Access Point. This function must be called successfully
 * before attempting to initialize the ESP-NOW service (`espnow_init`).
 *
 * @note The specific WiFi channel and other parameters are typically configured
 *       via Kconfig (e.g., `CONFIG_ESPNOW_CHANNEL`).
 *
 * @return ESP_OK on successful initialization of all required WiFi components.
 * @return Specific esp_err_t error codes on failure at any stage of the
 *         WiFi initialization process. These errors should be handled by the
 *         calling function (e.g., `app_main`), potentially by halting the
 *         application if WiFi functionality is critical.
 */
esp_err_t wifi_init(void);

#endif  // WIFI_H