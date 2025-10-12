/**
 * @file espnow_transmitter.h
 * @brief Public interface for the ESP-NOW transmitter module.
 *
 * This module is designed for a battery-powered sensor node. It handles the
 * initialization of ESP-NOW, management of a single gateway peer, and the
 * synchronous, blocking transmission of data with status checking. The design
 * is optimized for minimal "on-time" by avoiding FreeRTOS tasks and queues.
 */

#ifndef ESPNOW_TRANSMITTER_H
#define ESPNOW_TRANSMITTER_H

#include <stddef.h>   // For size_t
#include <stdint.h>   // Standard integer types (e.g., uint8_t)
#include "esp_err.h"  // ESP-IDF error codes (esp_err_t)

/**
 * @brief Maximum time (in milliseconds) to wait for a send confirmation (ACK).
 *
 * If the receiver does not acknowledge the packet within this time, the send
 * operation will fail with a timeout error. This prevents the device from
 * getting stuck if the gateway is offline.
 */
#define ESPNOW_SEND_TIMEOUT_MS 100

/**
 * @brief Initializes the ESP-NOW service in transmitter mode.
 *
 * This function performs the setup required for sending ESP-NOW data:
 * - Initializes the underlying ESP-NOW library.
 * - Creates a binary semaphore used for synchronizing the send operation.
 * - Registers the send callback function (`espnow_send_cb`) to get status updates.
 * - Sets the Primary Master Key (PMK) for establishing encrypted sessions.
 * - Adds a pre-configured gateway peer for encrypted communication.
 *
 * @note This function must be called after Wi-Fi has been initialized.
 *
 * @return esp_err_t
 *   - ESP_OK: If initialization was successful.
 *   - ESP_ERR_NO_MEM: If memory allocation fails for the semaphore.
 *   - Other esp_err_t codes from ESP-IDF functions on failure.
 */
esp_err_t espnow_transmitter_init(void);

/**
 * @brief De-initializes the ESP-NOW service.
 *
 * This function cleans up resources used by the ESP-NOW transmitter module:
 * - Unregisters the send callback.
 * - Deletes the synchronization semaphore.
 * - De-initializes the ESP-NOW library.
 *
 * @return esp_err_t
 *   - ESP_OK: On success.
 *   - Other esp_err_t codes on failure.
 */
esp_err_t espnow_transmitter_deinit(void);

/**
 * @brief Sends data to the pre-configured gateway peer and waits for confirmation.
 *
 * This is a synchronous (blocking) function. It sends the provided data payload
 * via ESP-NOW and then waits for the send callback to fire, indicating whether
 * the transmission was successful (acknowledged by the peer) or failed.
 *
 * @param data Pointer to the buffer containing the data payload to send.
 * @param len Length of the data payload in bytes.
 *
 * @return esp_err_t
 *   - ESP_OK: If the data was sent and successfully acknowledged by the peer.
 *   - ESP_ERR_INVALID_ARG: If `data` is NULL or `len` is 0.
 *   - ESP_ERR_TIMEOUT: If the peer did not acknowledge the packet within `ESPNOW_SEND_TIMEOUT_MS`.
 *   - ESP_FAIL: If the send callback reported a failure.
 *   - Other esp_err_t codes from `esp_now_send` on failure.
 */
esp_err_t espnow_transmitter_send(const uint8_t *data, size_t len);

#endif  // ESPNOW_TRANSMITTER_H