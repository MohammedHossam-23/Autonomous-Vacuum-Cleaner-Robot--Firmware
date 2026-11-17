#include "avcr/crc32_mpeg2.h"

#define AVCR_CRC32_POLY 0x04C11DB7u

static uint32_t crc_byte(uint32_t crc, uint8_t byte)
{
    crc ^= (uint32_t)byte << 24;
    for (int bit = 0; bit < 8; bit++)
    {
        if (crc & 0x80000000u)
        {
            crc = (crc << 1) ^ AVCR_CRC32_POLY;
        }
        else
        {
            crc <<= 1;
        }
    }
    return crc;
}

uint32_t avcr_crc32_mpeg2_bytes(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;

    if (data == NULL)
    {
        return crc;
    }

    for (size_t i = 0; i < len; i++)
    {
        crc = crc_byte(crc, data[i]);
    }

    return crc;
}

uint32_t avcr_crc32_mpeg2(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;

    if (data == NULL)
    {
        return crc;
    }

    size_t i = 0;
    for (; i + 4 <= len; i += 4)
    {
        /* The peripheral consumes a 32-bit word MSB-first, so within each
         * little-endian word the bytes are visited from index 3 down to 0. */
        crc = crc_byte(crc, data[i + 3]);
        crc = crc_byte(crc, data[i + 2]);
        crc = crc_byte(crc, data[i + 1]);
        crc = crc_byte(crc, data[i + 0]);
    }

    if (i < len)
    {
        uint8_t word[4] = {0, 0, 0, 0};
        for (size_t k = 0; i + k < len; k++)
        {
            word[k] = data[i + k];
        }
        crc = crc_byte(crc, word[3]);
        crc = crc_byte(crc, word[2]);
        crc = crc_byte(crc, word[1]);
        crc = crc_byte(crc, word[0]);
    }

    return crc;
}
