/**
 * @file battery_monitor.c
 * @author Mateusz Ressel (https://github.com/matt-ressel)
 *
 * @brief Implementation of the battery monitoring component.
 *
 * This file contains the hardware-specific logic for controlling the voltage
 * divider and reading the ADC, using the modern ESP-IDF adc_oneshot driver API.
 *
 * @version 0.1
 * @date    2025-10-28
 *
 * @copyright Copyright (c) 2025 Mateusz Ressel. Licensed under the MIT License.
 *
 */
#include "battery_monitor.h"

// ESP-IDF includes
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

// --- Hardware Specific Configuration ---
#define ADC_ENABLE_PIN GPIO_NUM_6    // GPIO to enable the voltage divider
#define ADC_INPUT_PIN ADC_CHANNEL_4  // GPIO4 to ADC1_CH4

// --- Measurement Configuration ---
#define ADC_UNIT ADC_UNIT_1           // Using ADC1
#define ADC_ATTEN ADC_ATTEN_DB_12     // Attenuation for full-scale voltage ~2500mV
#define ADC_BITWIDTH ADC_BITWIDTH_12  // Bitwidth 12 bits
#define NUM_SAMPLES 64                // Number of ADC samples to average

// --- Voltage Divider Configuration ---
// Assuming a 1:1 divider (two identical resistors), so the multiplier is 2.
#define VOLTAGE_DIVIDER_RATIO 2

// Logging tag for this component.
static const char* TAG = "BATTERY_MONITOR";

/** @brief Static handle for the ADC one-shot unit. */
static adc_oneshot_unit_handle_t s_adc_unit_handle;

/** @brief Static handle for the ADC calibration. */
static adc_cali_handle_t s_adc_cali_handle = NULL;

/** @brief Flag indicating if ADC calibration was successful. */
static bool s_adc_calibrated = false;

/** @brief Flag indicating if the battery monitor has been initialized. */
static bool s_is_initialized = false;

/** @brief Initializes ADC calibration for improved accuracy.
 *
 * @param unit ADC unit to calibrate.
 * @param atten Attenuation setting for the ADC.
 * @param out_handle Pointer to store the created calibration handle.
 * @return true if calibration was successful, false otherwise.
 */
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t* out_handle) {
  adc_cali_handle_t handle = NULL;
  esp_err_t ret = ESP_FAIL;
  bool calibrated = false;

  ESP_LOGI(TAG, "Attempting ADC calibration with Curve Fitting scheme...");
  adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = unit,
      .chan = ADC_INPUT_PIN,
      .atten = atten,
      .bitwidth = ADC_BITWIDTH,
  };
  ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
  if (ret == ESP_OK) {
    calibrated = true;
  }

  *out_handle = handle;
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "ADC calibration successful.");
  } else {
    ESP_LOGW(TAG, "ADC calibration failed (Error: %s). Readings will be less accurate.", esp_err_to_name(ret));
  }

  return calibrated;
}

// Initializes the Battery Monitor component.
esp_err_t battery_monitor_init(void) {
  ESP_LOGI(TAG, "Initializing Battery Monitor...");
  esp_err_t ret;  // Temporary variable for error checking

  // 1. Configure the GPIO pin that enables the voltage divider
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << ADC_ENABLE_PIN),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure GPIO: %s", esp_err_to_name(ret));
    return ret;
  }

  // Ensure the divider is disabled at startup
  ret = gpio_set_level(ADC_ENABLE_PIN, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set initial GPIO level: %s", esp_err_to_name(ret));
    return ret;
  }

  // 2. Initialize the ADC using the modern one-shot driver
  adc_oneshot_unit_init_cfg_t init_config = {
      .unit_id = ADC_UNIT,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };

  ret = adc_oneshot_new_unit(&init_config, &s_adc_unit_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create new ADC unit: %s", esp_err_to_name(ret));
    return ret;
  }

  // 3. Configure the ADC channel
  adc_oneshot_chan_cfg_t chan_config = {
      .atten = ADC_ATTEN,
      .bitwidth = ADC_BITWIDTH,  // 12-bit
  };
  ret = adc_oneshot_config_channel(s_adc_unit_handle, ADC_INPUT_PIN, &chan_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
    adc_oneshot_del_unit(s_adc_unit_handle);  //
    s_adc_unit_handle = NULL;
    return ret;
  }

  // 4. Initialize ADC calibration for improved accuracy
  s_adc_calibrated = adc_calibration_init(ADC_UNIT, ADC_ATTEN, &s_adc_cali_handle);

  // 5. Mark the component as initialized
  s_is_initialized = true;
  ESP_LOGI(TAG, "Battery Monitor initialized successfully.");
  return ESP_OK;
}

// De-initializes the Battery Monitor component.
esp_err_t battery_monitor_deinit(void) {
  ESP_LOGI(TAG, "De-initializing Battery Monitor...");
  esp_err_t ret = ESP_OK;        // Temporary variable for error checking
  esp_err_t final_ret = ESP_OK;  // To capture any error during deinit

  // Release the calibration handle if it exists
  if (s_adc_cali_handle) {
    ret = adc_cali_delete_scheme_curve_fitting(s_adc_cali_handle);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to delete ADC calibration scheme: %s", esp_err_to_name(ret));
      final_ret = ret;  // Capture error but continue deinit
    }
    s_adc_cali_handle = NULL;
    s_adc_calibrated = false;
  }

  // Delete the ADC one-shot unit handle if it exists
  if (s_adc_unit_handle) {
    ret = adc_oneshot_del_unit(s_adc_unit_handle);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to delete ADC unit: %s", esp_err_to_name(ret));
      if (final_ret == ESP_OK) {  // Only set if no previous error
        final_ret = ret;
      }
    }
    s_adc_unit_handle = NULL;
  }

  // Reset the GPIO pin to its default state
  ret = gpio_reset_pin(ADC_ENABLE_PIN);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to reset GPIO pin: %s", esp_err_to_name(ret));
    if (final_ret == ESP_OK) {
      final_ret = ret;
    }
  }

  // Clear initialization flag
  s_is_initialized = false;
  ESP_LOGI(TAG, "Battery Monitor de-initialized.");
  // Return the first encountered error or ESP_OK if all went well
  return final_ret;
}

// Starts the battery measurement process (non-blocking).
esp_err_t battery_monitor_start_measurement(void) {
  // Check if initialized
  if (!s_is_initialized) {
    ESP_LOGE(TAG, "Component not initialized, cannot perform operation.");
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGD(TAG, "Starting battery measurement (enabling voltage divider)...");
  // Enable the load switch to start charging the filter capacitor.
  return gpio_set_level(ADC_ENABLE_PIN, 1);
}

// Finalizes the measurement, returns the result, and disables the divider.
esp_err_t battery_monitor_get_measurement(uint16_t* voltage_mv) {
  if (voltage_mv == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // 1. Perform ADC reading with averaging
  uint32_t valid_samples = 0;
  uint32_t adc_reading_sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    int raw_adc;
    if (adc_oneshot_read(s_adc_unit_handle, ADC_INPUT_PIN, &raw_adc) == ESP_OK) {
      adc_reading_sum += raw_adc;
      valid_samples++;
    }
  }
  uint32_t raw_adc_avg = adc_reading_sum / valid_samples;

  // 2. IMMEDIATELY disable the voltage divider to conserve power.
  // This is the most critical power-saving step in this module.
  esp_err_t ret = gpio_set_level(ADC_ENABLE_PIN, 0);
  ESP_LOGD(TAG, "Voltage divider disabled.");

  // 3. Convert the raw ADC value to millivolts.
  // This is a linear approximation. Vref for ESP32-C3 ADC is ~1100mV.
  // The attenuation of 12dB provides a full-scale range of approx. 2500mV.
  int voltage_on_pin_mv = 0;   // Voltage measured at the ADC pin
  bool conversion_ok = false;  // Flag to indicate if calibration conversion succeeded

  if (s_adc_calibrated) {
    esp_err_t cali_ret = adc_cali_raw_to_voltage(s_adc_cali_handle, raw_adc_avg, &voltage_on_pin_mv);
    if (cali_ret == ESP_OK) {
      conversion_ok = true;
    } else {
      ESP_LOGE(TAG, "ADC calibration conversion failed, using fallback. Error: %s", esp_err_to_name(cali_ret));
    }
  }

  if (!conversion_ok) {
    voltage_on_pin_mv = raw_adc_avg * 2500 / 4095;
  }

  // 4. Account for the voltage divider to get the actual battery voltage.
  // V_batt = V_pin * (R1 + R2) / R2. For a 1:1 divider, this is V_pin * 2.
  *voltage_mv = (uint16_t)(voltage_on_pin_mv * VOLTAGE_DIVIDER_RATIO);

  ESP_LOGD(TAG, "Raw ADC avg: %lu, Pin voltage: %lu mV, Final battery voltage: %u mV",
           raw_adc_avg, voltage_on_pin_mv, *voltage_mv);

  return ret;  // Return the status of the gpio_set_level call
}