#include <stdint.h>
#include <string.h>

#include "avcr/crc32_mpeg2.h"
#include "avcr/frame.h"
#include "test_util.h"

static void test_crc_known_answer(void)
{
    const uint8_t check[] = "123456789";
    CHECK(avcr_crc32_mpeg2_bytes(check, 9) == 0x0376E6E7u);
}

static void test_crc_matches_stm32_word_order(void)
{
    /* The STM32 peripheral consumes each 32-bit word MSB-first, so on a
     * little-endian MCU the bytes within a word are seen in reverse order and
     * the tail is zero padded to a whole word. */
    const uint8_t payload[5] = {0x01, 0x02, 0x03, 0x04, 0x05};
    const uint8_t expected_stream[8] = {0x04, 0x03, 0x02, 0x01, 0x00, 0x00, 0x00, 0x05};

    CHECK(avcr_crc32_mpeg2(payload, 5) == avcr_crc32_mpeg2_bytes(expected_stream, 8));
}

static void test_crc_depends_on_padding(void)
{
    const uint8_t a[4] = {0x01, 0x02, 0x03, 0x04};
    const uint8_t b[5] = {0x01, 0x02, 0x03, 0x04, 0x05};

    CHECK(avcr_crc32_mpeg2(a, 4) != avcr_crc32_mpeg2(b, 5));
}

static void test_roundtrip_lengths(void)
{
    uint8_t payload[AVCR_FRAME_MAX_PAYLOAD];
    uint8_t frame[AVCR_FRAME_MAX_PAYLOAD + 16];
    uint8_t decoded[AVCR_FRAME_MAX_PAYLOAD];
    avcr_frame_status status;

    for (size_t len = 1; len <= sizeof(payload); len++)
    {
        for (size_t i = 0; i < len; i++)
        {
            payload[i] = (uint8_t)(i * 37u + len);
        }

        size_t frame_len = avcr_frame_encode(payload, len, frame, sizeof(frame), &status);
        CHECK(status == AVCR_FRAME_OK);
        CHECK(frame_len > len);
        CHECK(frame[frame_len - 1] == 0x00);

        size_t out_len = avcr_frame_decode(frame, frame_len, decoded, sizeof(decoded), &status);
        CHECK(status == AVCR_FRAME_OK);
        CHECK(out_len == len);
        CHECK(memcmp(payload, decoded, len) == 0);
    }
}

static void test_decode_without_delimiter(void)
{
    const uint8_t payload[6] = {0xAA, 0x00, 0xBB, 0x00, 0x00, 0xCC};
    uint8_t frame[32];
    uint8_t decoded[32];
    avcr_frame_status status;

    size_t frame_len = avcr_frame_encode(payload, sizeof(payload), frame, sizeof(frame), &status);
    CHECK(status == AVCR_FRAME_OK);

    size_t out_len = avcr_frame_decode(frame, frame_len - 1, decoded, sizeof(decoded), &status);
    CHECK(status == AVCR_FRAME_OK);
    CHECK(out_len == sizeof(payload));
    CHECK(memcmp(payload, decoded, sizeof(payload)) == 0);
}

static void test_corruption_is_detected(void)
{
    const uint8_t payload[8] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80};
    uint8_t frame[32];
    uint8_t decoded[32];
    avcr_frame_status status;

    size_t frame_len = avcr_frame_encode(payload, sizeof(payload), frame, sizeof(frame), &status);
    CHECK(status == AVCR_FRAME_OK);

    for (size_t i = 0; i + 1 < frame_len; i++)
    {
        uint8_t saved = frame[i];
        frame[i] = (uint8_t)(saved ^ 0x5A);
        if (frame[i] == 0x00)
        {
            frame[i] = 0x01;
        }

        size_t out_len = avcr_frame_decode(frame, frame_len, decoded, sizeof(decoded), &status);
        CHECK(status != AVCR_FRAME_OK || out_len == 0);

        frame[i] = saved;
    }
}

static void test_truncated_frame_is_rejected(void)
{
    const uint8_t payload[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    uint8_t frame[48];
    uint8_t decoded[32];
    avcr_frame_status status;

    size_t frame_len = avcr_frame_encode(payload, sizeof(payload), frame, sizeof(frame), &status);
    CHECK(status == AVCR_FRAME_OK);

    size_t out_len = avcr_frame_decode(frame, frame_len - 4, decoded, sizeof(decoded), &status);
    CHECK(status != AVCR_FRAME_OK || out_len == 0);
}

static void test_small_buffers_are_rejected(void)
{
    const uint8_t payload[32] = {0};
    uint8_t tiny[4];
    avcr_frame_status status;

    CHECK(avcr_frame_encode(payload, sizeof(payload), tiny, sizeof(tiny), &status) == 0);
    CHECK(status == AVCR_FRAME_ERR_OVERFLOW);

    uint8_t frame[64];
    size_t frame_len = avcr_frame_encode(payload, sizeof(payload), frame, sizeof(frame), &status);
    CHECK(status == AVCR_FRAME_OK);

    uint8_t small_payload[8];
    CHECK(avcr_frame_decode(frame, frame_len, small_payload, sizeof(small_payload), &status) == 0);
    CHECK(status == AVCR_FRAME_ERR_OVERFLOW);
}

int main(void)
{
    printf("test_frame\n");
    RUN(test_crc_known_answer);
    RUN(test_crc_matches_stm32_word_order);
    RUN(test_crc_depends_on_padding);
    RUN(test_roundtrip_lengths);
    RUN(test_decode_without_delimiter);
    RUN(test_corruption_is_detected);
    RUN(test_truncated_frame_is_rejected);
    RUN(test_small_buffers_are_rejected);
    TEST_REPORT();
}
