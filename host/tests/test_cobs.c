#include <stdint.h>
#include <string.h>

#include "cobs.h"
#include "test_util.h"

static uint32_t rng_state = 0x12345678u;

static uint8_t rng_next(void)
{
    rng_state = rng_state * 1664525u + 1013904223u;
    return (uint8_t)(rng_state >> 24);
}

static void test_roundtrip_all_lengths(void)
{
    uint8_t payload[300];
    uint8_t encoded[400];
    uint8_t decoded[300];

    for (size_t len = 1; len <= sizeof(payload); len++)
    {
        for (size_t i = 0; i < len; i++)
        {
            payload[i] = rng_next();
        }

        cobs_encode_result enc = cobs_encode(encoded, sizeof(encoded), payload, len);
        CHECK(enc.status == COBS_ENCODE_OK);
        CHECK(enc.out_len < sizeof(encoded));

        cobs_decode_result dec = cobs_decode(decoded, sizeof(decoded), encoded, enc.out_len);
        CHECK(dec.status == COBS_DECODE_OK);
        CHECK(dec.out_len == len);
        CHECK(memcmp(payload, decoded, len) == 0);
    }
}

static void expect_cobs(const uint8_t *in, size_t in_len,
                        const uint8_t *want, size_t want_len)
{
    uint8_t out[32];
    uint8_t back[32];

    cobs_encode_result enc = cobs_encode(out, sizeof(out), in, in_len);
    CHECK(enc.status == COBS_ENCODE_OK);
    CHECK(enc.out_len == want_len);
    if (enc.out_len == want_len)
    {
        CHECK(memcmp(out, want, want_len) == 0);
    }

    /* Decode the independently supplied vector, not our own output. */
    cobs_decode_result dec = cobs_decode(back, sizeof(back), want, want_len);
    CHECK(dec.status == COBS_DECODE_OK);
    CHECK(dec.out_len == in_len);
    if (dec.out_len == in_len)
    {
        CHECK(memcmp(back, in, in_len) == 0);
    }
}

/*
 * Byte-exact vectors taken from the COBS specification examples (Cheshire &
 * Baker, "Consistent Overhead Byte Stuffing"; also the worked examples on
 * Wikipedia). A round-trip test alone cannot catch an encoder and decoder that
 * are wrong in the same way, so the wire bytes are pinned here.
 */
static void test_known_vectors(void)
{
    static const uint8_t no_input[] = {0x00}; /* length 0, contents unused */
    static const uint8_t zero1_in[] = {0x00};
    static const uint8_t zero1_out[] = {0x01, 0x01};
    static const uint8_t zero2_in[] = {0x00, 0x00};
    static const uint8_t zero2_out[] = {0x01, 0x01, 0x01};
    static const uint8_t mixed_in[] = {0x11, 0x22, 0x00, 0x33};
    static const uint8_t mixed_out[] = {0x03, 0x11, 0x22, 0x02, 0x33};
    static const uint8_t plain_in[] = {0x01, 0x02, 0x03, 0x04};
    static const uint8_t plain_out[] = {0x05, 0x01, 0x02, 0x03, 0x04};
    static const uint8_t empty_out[] = {0x01};

    expect_cobs(no_input, 0, empty_out, sizeof(empty_out));
    expect_cobs(zero1_in, sizeof(zero1_in), zero1_out, sizeof(zero1_out));
    expect_cobs(zero2_in, sizeof(zero2_in), zero2_out, sizeof(zero2_out));
    expect_cobs(mixed_in, sizeof(mixed_in), mixed_out, sizeof(mixed_out));
    expect_cobs(plain_in, sizeof(plain_in), plain_out, sizeof(plain_out));
}

static void test_zero_bytes(void)
{
    uint8_t payload[64];
    memset(payload, 0, sizeof(payload));

    uint8_t encoded[96];
    uint8_t decoded[64];

    cobs_encode_result enc = cobs_encode(encoded, sizeof(encoded), payload, sizeof(payload));
    CHECK(enc.status == COBS_ENCODE_OK);

    /* A COBS body never contains a zero byte. */
    for (size_t i = 0; i < enc.out_len; i++)
    {
        CHECK(encoded[i] != 0x00);
    }

    cobs_decode_result dec = cobs_decode(decoded, sizeof(decoded), encoded, enc.out_len);
    CHECK(dec.status == COBS_DECODE_OK);
    CHECK(dec.out_len == sizeof(payload));
    CHECK(memcmp(payload, decoded, sizeof(payload)) == 0);
}

static void test_overflow_is_reported(void)
{
    uint8_t payload[64];
    uint8_t tiny[8];
    memset(payload, 0xAB, sizeof(payload));

    cobs_encode_result enc = cobs_encode(tiny, sizeof(tiny), payload, sizeof(payload));
    CHECK(enc.status != COBS_ENCODE_OK);

    cobs_decode_result dec = cobs_decode(tiny, sizeof(tiny), payload, sizeof(payload));
    CHECK(dec.status != COBS_DECODE_OK);
}

static void test_zero_byte_in_encoded_input_is_rejected(void)
{
    uint8_t encoded[16] = {0x03, 0x11, 0x00, 0x22};
    uint8_t decoded[16];

    cobs_decode_result dec = cobs_decode(decoded, sizeof(decoded), encoded, 4);
    CHECK(dec.status != COBS_DECODE_OK);
}

int main(void)
{
    printf("test_cobs\n");
    RUN(test_roundtrip_all_lengths);
    RUN(test_known_vectors);
    RUN(test_zero_bytes);
    RUN(test_overflow_is_reported);
    RUN(test_zero_byte_in_encoded_input_is_rejected);
    TEST_REPORT();
}
