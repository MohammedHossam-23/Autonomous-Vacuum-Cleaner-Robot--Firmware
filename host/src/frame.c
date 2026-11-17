#include "avcr/frame.h"

#include <string.h>

#include "avcr/crc32_mpeg2.h"
#include "cobs.h"

#define AVCR_FRAME_CRC_BYTES 4u
#define AVCR_FRAME_SCRATCH (AVCR_FRAME_MAX_PAYLOAD + AVCR_FRAME_CRC_BYTES)

const char *avcr_frame_status_str(avcr_frame_status status)
{
    switch (status)
    {
    case AVCR_FRAME_OK:            return "ok";
    case AVCR_FRAME_ERR_NULL:      return "null argument";
    case AVCR_FRAME_ERR_OVERFLOW:  return "buffer too small";
    case AVCR_FRAME_ERR_TOO_SHORT: return "frame too short";
    case AVCR_FRAME_ERR_COBS:      return "cobs decode error";
    case AVCR_FRAME_ERR_CRC:       return "crc mismatch";
    default:                       return "unknown error";
    }
}

static void set_status(avcr_frame_status *status, avcr_frame_status value)
{
    if (status != NULL)
    {
        *status = value;
    }
}

size_t avcr_frame_encode(const uint8_t *payload, size_t len,
                         uint8_t *out, size_t out_cap,
                         avcr_frame_status *status)
{
    uint8_t scratch[AVCR_FRAME_SCRATCH];

    if (payload == NULL || out == NULL)
    {
        set_status(status, AVCR_FRAME_ERR_NULL);
        return 0;
    }

    if (len > AVCR_FRAME_MAX_PAYLOAD)
    {
        set_status(status, AVCR_FRAME_ERR_OVERFLOW);
        return 0;
    }

    if (len > 0)
    {
        memcpy(scratch, payload, len);
    }

    uint32_t crc = avcr_crc32_mpeg2(payload, len);
    size_t body_len = len + AVCR_FRAME_CRC_BYTES;
    scratch[len + 0] = (uint8_t)(crc & 0xFFu);
    scratch[len + 1] = (uint8_t)((crc >> 8) & 0xFFu);
    scratch[len + 2] = (uint8_t)((crc >> 16) & 0xFFu);
    scratch[len + 3] = (uint8_t)((crc >> 24) & 0xFFu);

    cobs_encode_result encoded = cobs_encode(out, out_cap, scratch, body_len);
    if (encoded.status != COBS_ENCODE_OK)
    {
        set_status(status, AVCR_FRAME_ERR_OVERFLOW);
        return 0;
    }

    if (encoded.out_len + 1u > out_cap)
    {
        set_status(status, AVCR_FRAME_ERR_OVERFLOW);
        return 0;
    }

    out[encoded.out_len] = 0x00u;
    set_status(status, AVCR_FRAME_OK);
    return encoded.out_len + 1u;
}

size_t avcr_frame_decode(const uint8_t *in, size_t len,
                         uint8_t *payload, size_t payload_cap,
                         avcr_frame_status *status)
{
    uint8_t scratch[AVCR_FRAME_SCRATCH];

    if (in == NULL || payload == NULL)
    {
        set_status(status, AVCR_FRAME_ERR_NULL);
        return 0;
    }

    /* A trailing delimiter is optional; strip it if present. */
    if (len > 0 && in[len - 1] == 0x00u)
    {
        len--;
    }

    if (len == 0)
    {
        set_status(status, AVCR_FRAME_ERR_TOO_SHORT);
        return 0;
    }

    cobs_decode_result decoded = cobs_decode(scratch, sizeof(scratch), in, len);
    if (decoded.status != COBS_DECODE_OK)
    {
        set_status(status, AVCR_FRAME_ERR_COBS);
        return 0;
    }

    if (decoded.out_len < AVCR_FRAME_CRC_BYTES + 1u)
    {
        set_status(status, AVCR_FRAME_ERR_TOO_SHORT);
        return 0;
    }

    size_t body_len = decoded.out_len - AVCR_FRAME_CRC_BYTES;
    if (body_len > payload_cap)
    {
        set_status(status, AVCR_FRAME_ERR_OVERFLOW);
        return 0;
    }

    uint32_t received = (uint32_t)scratch[body_len + 0] |
                        ((uint32_t)scratch[body_len + 1] << 8) |
                        ((uint32_t)scratch[body_len + 2] << 16) |
                        ((uint32_t)scratch[body_len + 3] << 24);
    uint32_t calculated = avcr_crc32_mpeg2(scratch, body_len);
    if (received != calculated)
    {
        set_status(status, AVCR_FRAME_ERR_CRC);
        return 0;
    }

    memcpy(payload, scratch, body_len);
    set_status(status, AVCR_FRAME_OK);
    return body_len;
}
