/**
 * @file wifi.c
 * @author Mateusz Ressel (https://github.com/matt-ressel)
 *
 * @brief WiFi initialization and configuration module for ESP-NOW.
 *
 * This module handles the necessary setup of the WiFi stack on the ESP32
 * to enable ESP-NOW communication. It configures the WiFi in station mode,
 * sets the appropriate channel, and configures the communication protocol.
 *
 * @version 0.1
 * @date    2025-10-20
 *
 * @copyright Copyright (c) 2025 Mateusz Ressel. Licensed under the MIT License.
 *
 */

#include "wifi_service.h"  // Header for this WiFi initialization module

// ESP-IDF includes
#include "esp_err.h"    // For ESP_OK and other error codes
#include "esp_event.h"  // For esp_event_loop_create_default (was missing)
#include "esp_log.h"    // For ESP-IDF logging utilities
#include "esp_mac.h"    // For MAC address functions and macros (MACSTR, MAC2STR)
#include "esp_netif.h"  // For esp_netif_init (was missing)
#include "esp_wifi.h"   // For core WiFi functionality and definitions

// Logging tag for this module
static const char* TAG_WIFI = "WIFI_SERVICE";

/** @brief Defines the WiFi mode to be used for ESP-NOW (must be Station or AP mode). */
#define ESPNOW_WIFI_MODE WIFI_MODE_STA

/** @brief Defines the WiFi interface to be configured (must match ESPNOW_WIFI_MODE). */
#define ESPNOW_WIFI_IF WIFI_IF_STA

// Public API function implementations (documented in header)
esp_err_t wifi_service_init(void) {
  esp_err_t ret;  // Variable to store return codes from ESP-IDF functions

  ESP_LOGI(TAG_WIFI, "Initializing network interface (netif)...");
  ret = esp_netif_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_WIFI, "Failed to initialize netif: %s (0x%X)", esp_err_to_name(ret), ret);
    return ret;
  }

  ESP_LOGI(TAG_WIFI, "Creating default event loop...");
  ret = esp_event_loop_create_default();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_WIFI, "Failed to create default event loop: %s (0x%X)", esp_err_to_name(ret), ret);
    // Consider esp_netif_deinit() here if appropriate on failure
    return ret;
  }

  // Use default WiFi initialization configuration
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_LOGI(TAG_WIFI, "Initializing WiFi driver with default configuration...");
  ret = esp_wifi_init(&cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_WIFI, "Failed to initialize WiFi driver: %s (0x%X)", esp_err_to_name(ret), ret);
    // Consider esp_event_loop_delete_default() and esp_netif_deinit() for full cleanup
    return ret;
  }

  ESP_LOGI(TAG_WIFI, "Setting WiFi storage to RAM...");
  ret = esp_wifi_set_storage(WIFI_STORAGE_RAM);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_WIFI, "Failed to set WiFi storage: %s (0x%X)", esp_err_to_name(ret), ret);
    // Consider esp_wifi_deinit() for cleanup
    return ret;
  }

  ESP_LOGI(TAG_WIFI, "Setting WiFi mode to %s...", (ESPNOW_WIFI_MODE == WIFI_MODE_STA) ? "Station" : "AP");
  ret = esp_wifi_set_mode(ESPNOW_WIFI_MODE);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_WIFI, "Failed to set WiFi mode: %s (0x%X)", esp_err_to_name(ret), ret);
    return ret;
  }

  ESP_LOGI(TAG_WIFI, "Starting WiFi stack...");
  ret = esp_wifi_start();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_WIFI, "Failed to start WiFi stack: %s (0x%X)", esp_err_to_name(ret), ret);
    return ret;
  }

  // Set the WiFi channel for ESP-NOW communication.
  // CONFIG_ESPNOW_CHANNEL should be defined in Kconfig.
  ESP_LOGI(TAG_WIFI, "Setting WiFi channel for ESP-NOW to: %d", CONFIG_ESPNOW_CHANNEL);
  ret = esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_WIFI, "Failed to set WiFi channel: %s (0x%X)", esp_err_to_name(ret), ret);
    // Consider esp_wifi_stop() for cleanup
    return ret;
  }

  // Ensure bandwidth is set to HT20 for ESP-NOW compatibility
  ret = esp_wifi_set_bandwidth(ESPNOW_WIFI_IF, WIFI_BW_HT20);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_WIFI, "Failed to set WiFi bandwidth: %s (0x%X)", esp_err_to_name(ret), ret);
    return ret;
  }

  // Set the WiFi protocol. For ESP-NOW, 802.11b/g/n and LR (Long Range) are common.
  // Adjust as needed for compatibility with peer devices.
  ESP_LOGI(TAG_WIFI, "Setting WiFi protocol (802.11bgn + LR)...");
  ret = esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_LR);
  // ret = esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_WIFI, "Failed to set WiFi protocol: %s (0x%X)", esp_err_to_name(ret), ret);
    return ret;
  }

  // Retrieve and log the MAC address of the configured WiFi interface (STA or AP).
  uint8_t my_mac_addr[6] = {0};  // Local variable to store MAC address
  ESP_LOGI(TAG_WIFI, "Retrieving MAC address for interface %s...", (ESPNOW_WIFI_IF == WIFI_IF_STA) ? "STA" : "AP");
  ret = esp_wifi_get_mac(ESPNOW_WIFI_IF, my_mac_addr);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_WIFI, "Failed to get MAC address: %s (0x%X)", esp_err_to_name(ret), ret);
    return ret;  // MAC address is often crucial, so treat as an error if it fails
  }
  ESP_LOGI(TAG_WIFI, "Device MAC Address (" MACSTR ") retrieved successfully for ESP-NOW operations.", MAC2STR(my_mac_addr));

  ESP_LOGI(TAG_WIFI, "WiFi initialization for ESP-NOW complete. Channel: %d, Mode: %s.",
           CONFIG_ESPNOW_CHANNEL, (ESPNOW_WIFI_MODE == WIFI_MODE_STA) ? "Station" : "AP");

  return ESP_OK;  // All initialization steps successful
}