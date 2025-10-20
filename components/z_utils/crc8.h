/**
 * @file crc8.h
 * @brief Public interface for CRC-8 checksum calculation.
 *
 * This module provides a function to compute an 8-bit Cyclic Redundancy Check (CRC-8)
 * for a given block of data. CRC-8 is commonly used for error detection in
 * communication protocols and data storage.
 *
 * The specific CRC-8 parameters (e.g., polynomial, initial value, XOR out) are
 * defined by the implementation in `crc8.c`. This header only exposes the
 * calculation function.
 */

#ifndef CRC8_H
#define CRC8_H

#include <stddef.h>  // For size_t type (length of data)
#include <stdint.h>  // For uint8_t type (data elements and CRC result)

// --- Function Prototypes ---

/**
 * @brief Computes the CRC-8 checksum for a given data buffer.
 *
 * This function calculates an 8-bit Cyclic Redundancy Check (CRC) value for the
 * provided data. The calculation uses a predefined polynomial (typically 0x07 for
 * standard CRC-8, but consult the implementation in `crc8.c` for specifics if
 * different parameters like initial value or XOR out are used).
 *
 * @param data Pointer to the constant data buffer for which the CRC-8 is to be calculated.
 *             The data is treated as a sequence of bytes.
 * @param len  The length of the data buffer in bytes.
 *
 * @return uint8_t The calculated 8-bit CRC value.
 *
 * @note The caller is responsible for providing valid `data` and `len` arguments.
 *       If `data` is NULL and `len` is non-zero, dereferencing a NULL pointer will occur,
 *       leading to undefined behavior. If `len` is zero, the function will return the
 *       initial CRC value (0x00 with the current implementation).
 *       When used for data validation, `len` should typically be the size of the data
 *       payload *excluding* the stored CRC byte itself.
 */

uint8_t crc8(const uint8_t *data, size_t len);

#endif  // CRC8_H