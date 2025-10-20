/**
 * @file crc8.c
 * @brief Implementation of CRC-8 checksum calculation.
 *
 * This file contains the function that computes an 8-bit Cyclic Redundancy Check.
 * The specific algorithm implemented here uses a common CRC-8 polynomial (0x07)
 * and an initial CRC value of 0x00. There is no final XOR operation.
 * This is sometimes referred to as CRC-8/ATM or CRC-8 (poly 0x07).
 */

#include "crc8.h"  // Header for this module

// Public API function implementation (documented in header)
/**
 * @brief Calculates a CRC-8 checksum for the given data buffer.
 *
 * The algorithm iterates through each byte of the input data. For each byte,
 * it is XORed with the current CRC value. Then, for each bit of the result,
 * if the most significant bit (MSB) of the CRC is set, the CRC is shifted left
 * by one bit and XORed with the polynomial (0x07). Otherwise, the CRC is just
 * shifted left by one bit. This process is repeated for all 8 bits of the byte.
 *
 * Polynomial used: 0x07 (x^8 + x^2 + x^1 + x^0, but represented as the lower 8 bits without the implicit x^8 term)
 * Initial CRC value: 0x00
 * Final XOR value: 0x00 (none)
 *
 * @param data Pointer to the data buffer.
 * @param len Length of the data buffer in bytes.
 *
 * @return The calculated 8-bit CRC value.
 */
uint8_t crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;  // Initial CRC value

  // Defensive check: if data is NULL and we expect to read from it,
  // return a predictable value (e.g., initial CRC or a specific error indicator if possible).
  // This prevents a crash if len > 0. If len is 0, the loop won't execute anyway.
  if (data == NULL && len > 0) {
    // ESP_LOGE("CRC8_MODULE_TAG", "crc8 called with NULL data pointer and non-zero length!"); // Optional log
    return 0;  // Or some other defined error/default CRC. Returning initial CRC might be misleading.
               // For simplicity and to avoid adding error codes to a simple CRC function,
               // returning 0 (initial CRC) might be acceptable if the caller knows this behavior.
               // Better would be to ensure caller never passes NULL with len > 0.
  }

  while (len--) {
    crc ^= *data++;  // XOR current data byte into CRC

    // Process all 8 bits of the current CRC byte
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x80) {                    // If MSB is set
        crc = (uint8_t)(crc << 1) ^ 0x07;  // Shift left and XOR with polynomial
      } else {
        crc = (uint8_t)(crc << 1);  // Shift left
      }
    }
  }

  return crc;
}