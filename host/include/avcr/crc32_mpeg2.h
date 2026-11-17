#ifndef AVCR_CRC32_MPEG2_H
#define AVCR_CRC32_MPEG2_H

#include <stddef.h>
#include <stdint.h>

/*
 * CRC-32/MPEG-2: poly 0x04C11DB7, init 0xFFFFFFFF, no reflection, no final XOR.
 *
 * This matches the STM32 hardware CRC unit with the default HAL configuration
 * and the Crc32Mpeg2 class used by the Python bridge.
 */

/* Straight byte-stream CRC over the bytes in memory order (known-answer tests). */
uint32_t avcr_crc32_mpeg2_bytes(const uint8_t *data, size_t len);

/*
 * CRC exactly as the firmware produces it.
 *
 * STM32 HAL calls HAL_CRC_Calculate(&hcrc, (uint32_t *)buf, (len + 3) / 4):
 * the payload is rounded up to whole 32-bit words with zero padding, and each
 * word is fed to the peripheral MSB-first. On a little-endian MCU that means
 * the bytes of every 4-byte group are visited in reverse order. The Python
 * bridge compensates by byte-reversing each word before calling Crc32Mpeg2,
 * so a host implementation must do the same to agree with the device.
 */
uint32_t avcr_crc32_mpeg2(const uint8_t *data, size_t len);

#endif /* AVCR_CRC32_MPEG2_H */
