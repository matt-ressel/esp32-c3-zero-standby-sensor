/**
 * @file power_manager.c
 * @author Mateusz Ressel (https://github.com/matt-ressel)
 *
 * @brief Implementation of the Power Manager component.
 *
 * This component interfaces with an external power management IC (e.g., TPL5111)
 * to control power shutdown of the ESP32 device after completing tasks.
 * It configures a GPIO pin to signal the IC when the system is ready to power down.
 *
 * @version 0.2
 * @date    2025-11-11
 *
 * @copyright Copyright (c) 2025 Mateusz Ressel. Licensed under the MIT License.
 *
 */

#include "power_manager.h"

// ESP-IDF includes
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

// FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --- Hardware Specific Configuration ---
#define POWER_MANAGER_DONE_PIN GPIO_NUM_10  // Change to your pin connected to TPL5111 'DONE'
#define DONE_PULSE_DURATION_US 10           // Pulse duration in microseconds (>100ns)

// Logging tag for this component.
static const char* TAG = "POWER_MANAGER";

/** @brief Flag to track the initialization state of the component. */
static bool s_is_initialized = false;

// Initialize the Power Manager component.
esp_err_t power_manager_init(void) {
  ESP_LOGI(TAG, "Initializing Power Manager...");
  esp_err_t ret;

  // Configure the 'DONE' pin as a standard output
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << POWER_MANAGER_DONE_PIN),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure 'DONE' GPIO: %s", esp_err_to_name(ret));
    return ret;
  }

  // Ensure the pin is in a low state immediately after initialization
  ret = gpio_set_level(POWER_MANAGER_DONE_PIN, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set initial level on 'DONE' GPIO: %s", esp_err_to_name(ret));
    return ret;
  }

  s_is_initialized = true;
  ESP_LOGI(TAG, "Power Manager initialized successfully.");
  return ESP_OK;
}

// De-initialize the Power Manager and release resources.
esp_err_t power_manager_deinit(void) {
  if (!s_is_initialized) {
    return ESP_OK;
  }
  ESP_LOGI(TAG, "De-initializing Power Manager...");
  esp_err_t ret = gpio_reset_pin(POWER_MANAGER_DONE_PIN);
  s_is_initialized = false;
  return ret;
}

// Signal the external timer to cut system power.
esp_err_t power_manager_shutdown(void) {
  if (!s_is_initialized) {
    ESP_LOGE(TAG, "Cannot shut down, Power Manager not initialized!");
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGI(TAG, "Sending 'DONE' signal to external timer...");

  gpio_set_level(POWER_MANAGER_DONE_PIN, 1);  // Set 'DONE' high
  esp_rom_delay_us(DONE_PULSE_DURATION_US);   // 10 microseconds pulse
  gpio_set_level(POWER_MANAGER_DONE_PIN, 0);  // Set 'DONE' low

  // Wait briefly to ensure the pulse is registered by the hardware
  esp_rom_delay_us(100);

  // --- Code execution should NOT reach this point ---
  // If it does, it means the TPL5111 failed to cut power.
  // This makes it obvious during debugging that a hardware fault has occurred.
  ESP_LOGE(TAG, "CRITICAL HARDWARE FAULT: Power was not cut after 'DONE' signal.");

  return ESP_FAIL;
}
