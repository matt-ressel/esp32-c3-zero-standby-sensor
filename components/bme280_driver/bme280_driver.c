/**
 * @file bme280_driver.c
 * @author Mateusz Ressel (https://github.com/matt-ressel)
 *
 * @brief Implementation of the minimal, custom BME280 sensor driver.
 *
 * This file contains the logic for direct I2C communication with the BME280,
 * readout of calibration data, and application of the compensation formulas
 * as specified in the Bosch BME280 datasheet.
 * It uses the modern, handle-based ESP-IDF I2C master driver from 'driver/i2c_master.h'.
 *
 * @version 0.2
 * @date    2025-11-15
 *
 * @copyright Copyright (c) 2025 Mateusz Ressel. Licensed under the MIT License.
 *
 */

#include "bme280_driver.h"  // Include the corresponding header file

// ESP-IDF includes
#include "driver/i2c_master.h"
#include "esp_log.h"
// #include "esp_rom_sys.h"

// FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --- Hardware Specific Configuration ---
#define I2C_MASTER_NUM I2C_NUM_0    /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_TIMEOUT_MS 100          /*!< I2C operation timeout in milliseconds */

// --- BME280 Register Addresses (from datasheet, chapter 5) ---

#define BME280_SENSOR_ADDR 0x76     // BME280 I2C address (0x77 if SDO is high)
#define BME280_REG_ID 0xD0          // Chip ID register
#define BME280_REG_CALIB_00 0x88    // Start of first calibration data block
#define BME280_REG_CALIB_26 0xE1    // Start of second calibration data block
#define BME280_REG_CTRL_HUM 0xF2    // Humidity control register
#define BME280_REG_CTRL_MEAS 0xF4   // Measurement control register
#define BME280_REG_DATA_START 0xF7  // Start of data registers (press, temp, hum)

// Logging tag for this module
static const char* TAG = "BME280_DRIVER";

// --- Static Handles for I2C Communication ---
static i2c_master_bus_handle_t s_i2c_bus_handle;
static i2c_master_dev_handle_t s_bme280_dev_handle;

/** @brief Flag indicating if the BME280 driver has been initialized. */
static bool s_is_initialized = false;

/**
 * @brief Structure to hold all factory calibration coefficients.
 * @note The data types match those specified in the BME280 datasheet, table 16.
 */
typedef struct {
  // Temperature calibration coefficients
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;

  // Pressure calibration coefficients
  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;

  // Humidity calibration coefficients
  uint8_t dig_H1;
  int16_t dig_H2;
  uint8_t dig_H3;
  int16_t dig_H4;
  int16_t dig_H5;
  int8_t dig_H6;
} bme280_calib_data_t;

/** @brief Static instance of calibration data, read once during initialization. */
static bme280_calib_data_t s_calib_data;
/** @brief Fine-resolution temperature value, used for P and H compensation. */
static int32_t t_fine;

// --- Private I2C Helper Functions ---

/** @brief Reads a sequence of bytes from a BME280 register using the new driver API. */
static esp_err_t bme280_i2c_read_reg(uint8_t reg_addr, uint8_t* data, size_t len) {
  return i2c_master_transmit_receive(s_bme280_dev_handle, &reg_addr, 1, data, len, I2C_TIMEOUT_MS);
}

/** @brief Writes a single byte to a BME280 register using the new driver API. */
static esp_err_t bme280_i2c_write_reg(uint8_t reg_addr, uint8_t data) {
  uint8_t write_buf[2] = {reg_addr, data};
  return i2c_master_transmit(s_bme280_dev_handle, write_buf, sizeof(write_buf), I2C_TIMEOUT_MS);
}

// --- Compensation Formulas (from BME280 datasheet, section 4.2.3) ---

/** @brief Compensates the raw temperature reading and sets the global t_fine. */
static int32_t compensate_temperature(int32_t adc_T) {
  int32_t var1 = ((((adc_T >> 3) - ((int32_t)s_calib_data.dig_T1 << 1))) * ((int32_t)s_calib_data.dig_T2)) >> 11;
  int32_t var2 = (((((adc_T >> 4) - ((int32_t)s_calib_data.dig_T1)) * ((adc_T >> 4) - ((int32_t)s_calib_data.dig_T1))) >> 12) * ((int32_t)s_calib_data.dig_T3)) >> 14;
  t_fine = var1 + var2;
  return (t_fine * 5 + 128) >> 8;  // Returns temp in DegC * 100
}

/** @brief Compensates the raw pressure reading using t_fine. */
static uint32_t compensate_pressure(int32_t adc_P) {
  int64_t var1, var2, p;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)s_calib_data.dig_P6;
  var2 += ((var1 * (int64_t)s_calib_data.dig_P5) << 17);
  var2 += (((int64_t)s_calib_data.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)s_calib_data.dig_P3) >> 8) + ((var1 * (int64_t)s_calib_data.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)s_calib_data.dig_P1) >> 33;
  if (var1 == 0) return 0;  // Avoid division by zero
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)s_calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)s_calib_data.dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)s_calib_data.dig_P7) << 4);
  return (uint32_t)p;  // Returns pressure in Pascals (Pa)
}

/** @brief Compensates the raw humidity reading using t_fine. */
static uint32_t compensate_humidity(int32_t adc_H) {
  int32_t v_x1_u32r = (t_fine - 76800);
  v_x1_u32r = (((((adc_H << 14) - (((int32_t)s_calib_data.dig_H4) << 20) - (((int32_t)s_calib_data.dig_H5) * v_x1_u32r)) + 16384) >> 15) *
               (((((((v_x1_u32r * ((int32_t)s_calib_data.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)s_calib_data.dig_H3)) >> 11) + 32768)) >> 10) + 2097152) *
                     ((int32_t)s_calib_data.dig_H2) +
                 8192) >>
                14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)s_calib_data.dig_H1)) >> 4));
  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
  return (uint32_t)(v_x1_u32r >> 12);  // Returns humidity in %RH * 1024
}

// --- Public API Function Implementations ---

/// Initialize the BME280 sensor
esp_err_t bme280_driver_init(void) {
  ESP_LOGI(TAG, "Initializing BME280 sensor driver...");
  esp_err_t ret;

  // 1. Configure I2C master interface
  i2c_master_bus_config_t i2c_bus_config = {
      .i2c_port = I2C_MASTER_NUM,
      .sda_io_num = CONFIG_I2C_MASTER_SDA_PIN,
      .scl_io_num = CONFIG_I2C_MASTER_SCL_PIN,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  ret = i2c_new_master_bus(&i2c_bus_config, &s_i2c_bus_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
    return ret;
  }

  // 2. Add the BME280 device to the bus
  i2c_device_config_t i2c_dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = BME280_SENSOR_ADDR,
      .scl_speed_hz = CONFIG_I2C_MASTER_FREQUENCY,  // Use configured frequency
  };
  ret = i2c_master_bus_add_device(s_i2c_bus_handle, &i2c_dev_config, &s_bme280_dev_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add BME280 device to bus: %s", esp_err_to_name(ret));
    i2c_del_master_bus(s_i2c_bus_handle);  // Clean up bus handle
    return ret;
  }

  // 3. Verify the chip ID
  uint8_t chip_id = 0;
  ret = bme280_i2c_read_reg(BME280_REG_ID, &chip_id, 1);
  if (ret != ESP_OK || chip_id != 0x60) {
    ESP_LOGE(TAG, "Sensor not found or wrong chip ID. Read: 0x%02X, Expected: 0x60", chip_id);
    bme280_driver_deinit();
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "BME280 found with Chip ID: 0x%02X", chip_id);

  // 4. Read all calibration coefficients from the device
  uint8_t calib_block1[26];
  ret = bme280_i2c_read_reg(BME280_REG_CALIB_00, calib_block1, sizeof(calib_block1));
  if (ret != ESP_OK) return ret;

  s_calib_data.dig_T1 = (calib_block1[1] << 8) | calib_block1[0];
  s_calib_data.dig_T2 = (calib_block1[3] << 8) | calib_block1[2];
  s_calib_data.dig_T3 = (calib_block1[5] << 8) | calib_block1[4];

  s_calib_data.dig_P1 = (calib_block1[7] << 8) | calib_block1[6];
  s_calib_data.dig_P2 = (calib_block1[9] << 8) | calib_block1[8];
  s_calib_data.dig_P3 = (calib_block1[11] << 8) | calib_block1[10];
  s_calib_data.dig_P4 = (calib_block1[13] << 8) | calib_block1[12];
  s_calib_data.dig_P5 = (calib_block1[15] << 8) | calib_block1[14];
  s_calib_data.dig_P6 = (calib_block1[17] << 8) | calib_block1[16];
  s_calib_data.dig_P7 = (calib_block1[19] << 8) | calib_block1[18];
  s_calib_data.dig_P8 = (calib_block1[21] << 8) | calib_block1[20];
  s_calib_data.dig_P9 = (calib_block1[23] << 8) | calib_block1[22];

  s_calib_data.dig_H1 = calib_block1[25];

  uint8_t calib_block2[7];
  ret = bme280_i2c_read_reg(BME280_REG_CALIB_26, calib_block2, sizeof(calib_block2));
  if (ret != ESP_OK) return ret;

  s_calib_data.dig_H2 = (calib_block2[1] << 8) | calib_block2[0];
  s_calib_data.dig_H3 = calib_block2[2];
  s_calib_data.dig_H4 = (calib_block2[3] << 4) | (calib_block2[4] & 0x0F);
  s_calib_data.dig_H5 = (calib_block2[5] << 4) | (calib_block2[4] >> 4);
  s_calib_data.dig_H6 = calib_block2[6];

  // 5. Configure sensor for our use case: T,P,H oversampling x1, Forced Mode
  ret = bme280_i2c_write_reg(BME280_REG_CTRL_HUM, 0x01);  // osrs_h = x1
  if (ret != ESP_OK) return ret;
  // osrs_t = x1 (001), osrs_p = x1 (001), mode = forced (01)
  uint8_t ctrl_meas_val = (0b001 << 5) | (0b001 << 2) | 0b01;
  ret = bme280_i2c_write_reg(BME280_REG_CTRL_MEAS, ctrl_meas_val);
  if (ret != ESP_OK) return ret;

  // 6. Mark the driver as initialized
  s_is_initialized = true;
  ESP_LOGI(TAG, "BME280 driver initialized successfully.");
  return ESP_OK;
}

// De-initializes the BME280 driver and releases I2C resources.
esp_err_t bme280_driver_deinit(void) {
  if (s_bme280_dev_handle) {
    i2c_master_bus_rm_device(s_bme280_dev_handle);
    s_bme280_dev_handle = NULL;
  }
  if (s_i2c_bus_handle) {
    i2c_del_master_bus(s_i2c_bus_handle);
    s_i2c_bus_handle = NULL;
  }

  // Clear initialization flag
  s_is_initialized = false;
  ESP_LOGI(TAG, "BME280 driver de-initialized.");
  return ESP_OK;
}

// Performs a single measurement of all values and returns the compensated results.
esp_err_t bme280_driver_read_data(bme280_data_t* data) {
  // Check if initialized
  if (!s_is_initialized) {
    ESP_LOGE(TAG, "Component not initialized, cannot perform operation.");
    return ESP_ERR_INVALID_STATE;
  }

  if (data == NULL) return ESP_ERR_INVALID_ARG;
  esp_err_t ret;

  // 1. Trigger a measurement by writing to the control register
  // Set osrs_t = x1 (001), osrs_p = x1 (001), mode = forced (01)
  uint8_t ctrl_meas_val = (0b001 << 5) | (0b001 << 2) | 0b01;
  ret = bme280_i2c_write_reg(BME280_REG_CTRL_MEAS, ctrl_meas_val);
  if (ret != ESP_OK) return ret;

  // 2. Wait for the measurement to complete. Datasheet suggests ~8ms for TPH x1.
  // esp_rom_delay_us(10000);   // 10 ms delay to ensure measurement completion
  vTaskDelay(pdMS_TO_TICKS(10));  // 10 ms delay to ensure measurement completion

  // 3. Read all 8 data registers (Pressure, Temp, Humidity) in a single burst read
  uint8_t data_buffer[8];
  ret = bme280_i2c_read_reg(BME280_REG_DATA_START, data_buffer, sizeof(data_buffer));
  if (ret != ESP_OK) return ret;

  // 4. Reconstruct the raw ADC values from the buffer
  int32_t adc_P = (data_buffer[0] << 12) | (data_buffer[1] << 4) | (data_buffer[2] >> 4);
  int32_t adc_T = (data_buffer[3] << 12) | (data_buffer[4] << 4) | (data_buffer[5] >> 4);
  int32_t adc_H = (data_buffer[6] << 8) | data_buffer[7];

  // 5. Apply compensation formulas to get final values
  int32_t temp_int = compensate_temperature(adc_T);  // This must be first
  uint32_t press_int = compensate_pressure(adc_P);
  uint32_t hum_int = compensate_humidity(adc_H);

  data->temperature = temp_int;  // in °C * 100
  data->pressure = press_int;    // // in Pa * 256 (Q24.8 format)
  data->humidity = hum_int;      // in %RH * 1024

  // data->temperature = (float)temp_int / 100.0f;
  // data->pressure = (float)press_int / 256.0f / 100.0f;  // Pa -> hPa
  // data->humidity = (float)hum_int / 1024.0f;            // %RH

  // ESP_LOGI(TAG, "Read successful: T=%.2f C, P=%.2f hPa, H=%.2f %%", data->temperature, data->pressure, data->humidity);
  ESP_LOGI(TAG, "Read successful: T_raw=%ld, P_raw=%lu, H_raw=%lu", temp_int, press_int, hum_int);
  return ESP_OK;
}