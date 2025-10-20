/**
 * @file sensor_data.h
 * @author Mateusz Ressel (https://github.com/matt-ressel)
 *
 * @brief Defines the data structure for encapsulating sensor readings.
 *
 * This header file specifies the `sensor_data_t` structure, which is designed
 * to aggregate various environmental sensor readings (such as temperature,
 * humidity, pressure, air quality, and battery voltage). It also includes a
 * Cyclic Redundancy Check (CRC-8) field for ensuring data integrity.
 * For the air quality field, a specific convention (value of -1) is used
 * to indicate when a sensor node does not provide this particular reading.
 * This standardized structure is typically employed for data payloads transmitted
 * via ESP-NOW from sensor nodes to a gateway.
 * 
 * @version 0.1
 * @date    2025-10-20
 *
 * @copyright Copyright (c) 2025 Mateusz Ressel. Licensed under the MIT License.
 *
 */

#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <stdint.h>  // Required for standard integer types (e.g., int16_t, uint16_t, uint8_t)

/**
 * @brief Structure for encapsulating a set of sensor data readings.
 *
 * This structure defines the precise binary format for sensor data payloads.
 * It is intended for use in network transmissions (e.g., ESP-NOW) from sensor
 * nodes to a receiving gateway or data logger.
 *
 * The `__attribute__((packed))` directive is crucial as it instructs the compiler
 * to lay out the structure members contiguously in memory, without introducing
 * any padding bytes. This ensures a minimal and predictable size for efficient
 * network transmission and facilitates consistent data parsing across different
 * platforms or by various software modules that might interpret this data.
 *
 * @note All multi-byte integer fields (e.g., `int16_t`, `uint16_t`) within this
 *       structure are assumed to be transmitted and received in **little-endian**
 *       byte order, unless explicitly specified otherwise by the overarching
 *       communication protocol or system design.
 */
typedef struct __attribute__((packed)) {
  /**
   * @brief Ambient Temperature reading.
   *
   * Unit: Degrees Celsius (°C).
   * Scaling: The value is scaled by a factor of 10.
   *          For example, a stored value of `255` represents `25.5°C`.
   * Storage: Signed 16-bit integer (`int16_t`).
   * Size: 2 bytes.
   * Effective Range (after scaling): -3276.8°C to +3276.7°C.
   * Expected Sensor Range: -40.0°C to +85.0°C.
   */
  int16_t temperature;  // Size: 2 bytes

  /**
   * @brief Relative Humidity reading.
   *
   * Unit: Percent (%).
   * Scaling: No scaling. Value directly represents percentage (e.g., a stored value of `55` represents `55% RH`).
   *          This precision aligns with typical sensor accuracy (e.g., ±3% RH).
   * Storage: Unsigned 8-bit integer (`uint8_t`).
   * Size: 1 byte.
   * Effective Range: 0% to 100% (typical for sensors), can store values up to 255%.
   */
  uint8_t humidity;  // Size: 1 byte

  /**
   * @brief Atmospheric Pressure reading.
   *
   * Unit: Hectopascals (hPa).
   * Scaling: The value is scaled by a factor of 10.
   *          For example, a stored value of `10132` represents `1013.2 hPa`.
   * Storage: Unsigned 16-bit integer (`uint16_t`).
   * Size: 2 bytes.
   * Effective Range (after scaling): 0.0 hPa to 6553.5 hPa.
   */
  uint16_t pressure;  // Size: 2 bytes

  /**
   * @brief Air Quality reading (e.g., an index like AQI, a VOC index, or a particulate matter concentration).
   *
   * Unit: Device-dependent. Common Air Quality Index (AQI) scales typically range from 0 to 500.
   *       The specific interpretation of this value depends on the air quality sensor used by the node.
   *       Refer to the sensor's datasheet or the standard for the specific index being used.
   * Scaling: No scaling is applied by default to this field; it stores the raw value from the sensor or a calculated index.
   * Storage: Signed 16-bit integer (`int16_t`). This type is chosen to allow for a special
   *          negative value (-1) to indicate absence of data, while still accommodating
   *          the typical positive range of AQI values (0-500).
   * Size: 2 bytes.
   * Valid Data Range: Typically 0 to 500 for actual AQI readings.
   * @note **"Not Available" Convention:** A stored value of **-1** in this field indicates
   *       that air quality data was not provided by the sending sensor node (e.g., if the
   *       node uses a sensor without air quality measurement capabilities, like a BME280).
   *       The receiving system (gateway and backend) must interpret this value accordingly.
   */
  int16_t air_quality;  // Size: 2 bytes

  /**
   * @brief Battery voltage of the sending sensor device.
   *
   * Unit: Millivolts (mV).
   *       For example, a stored value of `3300` represents `3.3V` or `3300mV`.
   * Storage: Unsigned 16-bit integer (`uint16_t`).
   * Size: 2 bytes.
   * Range: 0 mV to 65535 mV (which is 0V to 65.535V).
   */
  uint16_t battery_mv;  // Size: 2 bytes

  /**
   * @brief 8-bit Cyclic Redundancy Check (CRC-8) for data integrity.
   *
   * This checksum is calculated by the sending sensor node over all preceding
   * fields in this structure (i.e., from `temperature` through `battery_mv`).
   * The receiving device (gateway) should recalculate the CRC-8 on the received
   * data (excluding this CRC field itself) and compare it with this transmitted
   * `crc` value to verify that the sensor data has not been corrupted during
   * transmission.
   *
   * The specific CRC-8 polynomial, initial value, and other parameters must be
   * consistent between the sender (sensor node) and the receiver (gateway).
   * (e.g., CRC-8 with polynomial 0x07, initial value 0x00, no final XOR).
   *
   * Storage: Unsigned 8-bit integer (`uint8_t`).
   * Size: 1 byte.
   */
  uint8_t crc;  // Size: 1 byte

} sensor_data_t;  // Total structure size: 2 (temp) + 1 (hum) + 2 (press) + 2 (aq) + 2 (batt) + 1 (crc) = 10 bytes.

#endif  // SENSOR_DATA_H
