#ifndef AVCR_FRAME_H
#define AVCR_FRAME_H

#include <stddef.h>
#include <stdint.h>

/* Mirrors MAX_TX_MSG_LEN / RX_BUF_SIZE in Core/Src/main.c. */
#define AVCR_FRAME_MAX_PAYLOAD 254

typedef enum
{
    AVCR_FRAME_OK = 0,
    AVCR_FRAME_ERR_NULL,
    AVCR_FRAME_ERR_OVERFLOW,
    AVCR_FRAME_ERR_TOO_SHORT,
    AVCR_FRAME_ERR_COBS,
    AVCR_FRAME_ERR_CRC
} avcr_frame_status;

/*
 * Wire format: COBS(payload || crc32_le) followed by one 0x00 delimiter.
 *
 * Returns the number of bytes written to `out` (including the delimiter), or 0
 * on error. `status` may be NULL.
 */
size_t avcr_frame_encode(const uint8_t *payload, size_t len,
                         uint8_t *out, size_t out_cap,
                         avcr_frame_status *status);

/*
 * Decodes one frame. A trailing 0x00 delimiter is optional. Returns the payload
 * length (CRC stripped), or 0 on error.
 */
size_t avcr_frame_decode(const uint8_t *in, size_t len,
                         uint8_t *payload, size_t payload_cap,
                         avcr_frame_status *status);

const char *avcr_frame_status_str(avcr_frame_status status);

#endif /* AVCR_FRAME_H */
