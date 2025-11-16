/**
 * @file espnow_sender.c
 * @author Mateusz Ressel (https://github.com/matt-ressel)
 *
 * @brief Implementation of the ESP-NOW sender module for the sensor node.
 *
 * This file contains the logic for initializing the ESP-NOW service, adding a
 * peer, and performing synchronous, blocking sends. It uses a FreeRTOS binary
 * semaphore to wait for the transmission acknowledgement, ensuring data is sent
 * successfully before the system powers down.
 *
 * @version 0.2
 * @date    2025-11-15
 *
 * @copyright Copyright (c) 2025 Mateusz Ressel. Licensed under the MIT License.
 *
 */

#include "espnow_sender.h"  // Include the header for this module

#include <string.h>  // For memcpy

// ESP-IDF includes
#include "esp_log.h"   // Logging utilities (ESP_LOGI, ESP_LOGE, etc.)
#include "esp_mac.h"   // MAC address utilities
#include "esp_now.h"   // ESP-NOW API functions
#include "esp_wifi.h"  // For Wi-Fi interface type

// FreeRTOS includes
#include "freertos/FreeRTOS.h"  // For FreeRTOS types
#include "freertos/semphr.h"    // For FreeRTOS semaphores

// Logging tag for this module
static const char* TAG = "ESPNOW_SENDER";

/** @brief Handle for the binary semaphore used to signal send completion. */
static SemaphoreHandle_t s_send_sem = NULL;

/** @brief Flag indicating if the ESP-NOW sender has been initialized. */
static bool s_is_initialized = false;

/** @brief Variable to store the status of the last transmission attempt. */
static esp_now_send_status_t s_send_status = ESP_NOW_SEND_FAIL;

/**
 * @brief MAC address of the gateway/receiver, populated from Kconfig at compile-time.
 */
static const uint8_t s_gateway_mac_addr[ESP_NOW_ETH_ALEN] = {GATEWAY_MAC_AS_BYTE_ARRAY};

/**
 * @brief ESP-NOW send callback function (executed in Wi-Fi task context).
 *
 * This callback is invoked by the ESP-NOW stack after a packet transmission attempt.
 * It updates the global `s_send_status` variable and gives a semaphore to unblock
 * the `espnow_send` function, which is waiting for this signal.
 *
 * @note This function runs in a high-priority Wi-Fi task context. Processing here
 *       must be minimal and non-blocking.
 *
 * @param mac_addr MAC address of the peer the data was sent to.
 * @param status Status of the transmission (ESP_NOW_SEND_SUCCESS or ESP_NOW_SEND_FAIL).
 */
static void espnow_send_cb(const wifi_tx_info_t* tx_info, esp_now_send_status_t status) {
  (void)tx_info;  // Unused parameter
  s_send_status = status;
  if (s_send_sem != NULL) {
    xSemaphoreGive(s_send_sem);
  }
}

// Initializes the ESP-NOW sender module
esp_err_t espnow_sender_init(void) {
  ESP_LOGI(TAG, "Initializing ESP-NOW sender...");

  esp_err_t ret;  // Variable to store return codes from ESP-IDF functions

  // Create a binary semaphore to signal the completion of a send operation.
  s_send_sem = xSemaphoreCreateBinary();
  if (s_send_sem == NULL) {
    ESP_LOGE(TAG, "Failed to create send semaphore");
    return ESP_ERR_NO_MEM;
  }

  // Initialize the ESP-NOW stack.
  ret = esp_now_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize ESP-NOW: %s", esp_err_to_name(ret));
    vSemaphoreDelete(s_send_sem);
    s_send_sem = NULL;
    return ret;
  }

  // Register the send callback function.
  ret = esp_now_register_send_cb(espnow_send_cb);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register ESP-NOW send callback: %s", esp_err_to_name(ret));
    esp_now_deinit();  // De-initialize ESP-NOW

    // Delete the semaphore
    vSemaphoreDelete(s_send_sem);
    s_send_sem = NULL;
    return ret;
  }

  // Set the Primary Master Key (PMK) for encryption.
  ret = esp_now_set_pmk((const uint8_t*)CONFIG_ESPNOW_PMK);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set ESP-NOW PMK: %s", esp_err_to_name(ret));
    esp_now_unregister_send_cb();  // Unregister callback
    esp_now_deinit();              // De-initialize ESP-NOW

    // Delete the semaphore
    vSemaphoreDelete(s_send_sem);
    s_send_sem = NULL;
    return ret;
  }

  // Prepare the peer information structure for the gateway.
  esp_now_peer_info_t peer_info = {0};
  memcpy(peer_info.peer_addr, s_gateway_mac_addr, ESP_NOW_ETH_ALEN);

  // Configure the peer settings
  peer_info.channel = CONFIG_ESPNOW_CHANNEL;  // Use configured channel
  peer_info.ifidx = ESP_IF_WIFI_STA;          // Use Station interface
  peer_info.encrypt = true;                   // Enable encryption

  // Set the Local Master Key (LMK) for this peer.
  memcpy(peer_info.lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);

  // Add the gateway as a peer.
  ret = esp_now_add_peer(&peer_info);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add gateway peer: %s", esp_err_to_name(ret));
    esp_now_set_pmk(NULL);         // Clear PMK
    esp_now_unregister_send_cb();  // Unregister callback
    esp_now_deinit();              // De-initialize ESP-NOW

    // Delete the semaphore
    vSemaphoreDelete(s_send_sem);
    s_send_sem = NULL;
    return ret;
  }

  // Mark the sender as initialized
  s_is_initialized = true;
  ESP_LOGI(TAG, "ESP-NOW sender initialized successfully.");
  return ESP_OK;
}

// De-initializes the ESP-NOW sender module
esp_err_t espnow_sender_deinit(void) {
  ESP_LOGI(TAG, "De-initializing ESP-NOW sender...");
  esp_now_unregister_send_cb();  // Unregister the send callback
  esp_now_deinit();              // De-initialize the ESP-NOW stack

  // Delete the semaphore
  if (s_send_sem != NULL) {
    vSemaphoreDelete(s_send_sem);
    s_send_sem = NULL;
  }
  
  // Clear initialization flag
  s_is_initialized = false;
  ESP_LOGI(TAG, "ESP-NOW sender de-initialized.");
  return ESP_OK;
}

// Sends data via ESP-NOW and waits for confirmation
esp_err_t espnow_sender_transmit(const uint8_t* data, size_t len) {
  // Check if initialized
  if (!s_is_initialized) {
    ESP_LOGE(TAG, "Component not initialized, cannot perform operation.");
    return ESP_ERR_INVALID_STATE;
  }

  // Validate input parameters
  if (data == NULL || len == 0) {
    ESP_LOGE(TAG, "Invalid arguments for send (data is NULL or len is 0)");
    return ESP_ERR_INVALID_ARG;
  }
  if (s_send_sem == NULL) {
    ESP_LOGE(TAG, "Cannot send, semaphore not initialized");
    return ESP_FAIL;
  }

  // Reset status and clear semaphore before sending
  s_send_status = ESP_NOW_SEND_FAIL;
  xSemaphoreTake(s_send_sem, 0);  // Clear any pending signals

  // Send the data. This is a non-blocking call that queues the packet for transmission.
  esp_err_t ret = esp_now_send(s_gateway_mac_addr, data, len);  //
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ESP-NOW send failed: %s", esp_err_to_name(ret));
    return ret;
  }

  // Wait for the send callback to fire and give the semaphore.
  ESP_LOGD(TAG, "Waiting for send confirmation...");
  if (xSemaphoreTake(s_send_sem, pdMS_TO_TICKS(ESPNOW_SEND_TIMEOUT_MS)) == pdTRUE) {
    // Semaphore was given, check the status from the callback.
    if (s_send_status == ESP_NOW_SEND_SUCCESS) {
      ESP_LOGI(TAG, "Data sent successfully.");
      return ESP_OK;
    } else {
      ESP_LOGE(TAG, "Data send failed, delivery status: %d", s_send_status);
      return ESP_FAIL;
    }
  } else {
    // Semaphore was not given within the timeout period.
    ESP_LOGE(TAG, "Send confirmation timed out.");
    return ESP_ERR_TIMEOUT;
  }
}