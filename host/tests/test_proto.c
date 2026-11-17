#include <stdint.h>
#include <string.h>

#include "avcr/frame.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "rx.pb.h"
#include "test_util.h"
#include "tx.pb.h"

static size_t encode_tx(const TxMsg *msg, uint8_t *payload, size_t payload_cap)
{
    pb_ostream_t stream = pb_ostream_from_buffer(payload, payload_cap);
    if (!pb_encode(&stream, TxMsg_fields, msg))
    {
        return 0;
    }
    return stream.bytes_written;
}

static void test_tx_roundtrip(void)
{
    TxMsg in = TxMsg_init_zero;
    in.has_odom = true;
    in.odom.has_x = in.odom.has_y = in.odom.has_yaw = true;
    in.odom.has_linear_x = in.odom.has_angular_z = true;
    in.odom.has_odom_time_stamp_us = true;
    in.odom.x = 1.5f;
    in.odom.y = -2.25f;
    in.odom.yaw = 0.5f;
    in.odom.linear_x = 0.3f;
    in.odom.angular_z = -0.1f;
    in.odom.odom_time_stamp_us = 987654u;

    in.has_battery = true;
    in.battery.has_StateOfCharge = true;
    in.battery.StateOfCharge = 42.5f;

    uint8_t payload[AVCR_FRAME_MAX_PAYLOAD];
    size_t payload_len = encode_tx(&in, payload, sizeof(payload));
    CHECK(payload_len > 0);

    uint8_t frame[AVCR_FRAME_MAX_PAYLOAD + 16];
    avcr_frame_status status;
    size_t frame_len = avcr_frame_encode(payload, payload_len, frame, sizeof(frame), &status);
    CHECK(status == AVCR_FRAME_OK);

    uint8_t decoded_payload[AVCR_FRAME_MAX_PAYLOAD];
    size_t decoded_len = avcr_frame_decode(frame, frame_len, decoded_payload,
                                           sizeof(decoded_payload), &status);
    CHECK(status == AVCR_FRAME_OK);
    CHECK(decoded_len == payload_len);

    TxMsg out = TxMsg_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(decoded_payload, decoded_len);
    CHECK(pb_decode(&stream, TxMsg_fields, &out));

    CHECK(out.has_odom);
    CHECK_NEAR(out.odom.x, in.odom.x, 1e-6f);
    CHECK_NEAR(out.odom.y, in.odom.y, 1e-6f);
    CHECK_NEAR(out.odom.yaw, in.odom.yaw, 1e-6f);
    CHECK(out.odom.odom_time_stamp_us == in.odom.odom_time_stamp_us);
    CHECK(out.has_battery);
    CHECK_NEAR(out.battery.StateOfCharge, in.battery.StateOfCharge, 1e-6f);
    CHECK(!out.has_imu);
}

static void test_rx_roundtrip(void)
{
    RxMsg in = RxMsg_init_zero;
    in.has_cmd_vel = true;
    in.cmd_vel.has_linear_x = in.cmd_vel.has_angular_z = true;
    in.cmd_vel.linear_x = 1.5f;
    in.cmd_vel.angular_z = 0.5f;

    in.has_hardware_cmd = true;
    in.hardware_cmd.has_vacuum_state = in.hardware_cmd.has_side_brushcmd = true;
    in.hardware_cmd.vacuum_state = 3;
    in.hardware_cmd.side_brushcmd = 1;

    uint8_t payload[AVCR_FRAME_MAX_PAYLOAD];
    pb_ostream_t ostream = pb_ostream_from_buffer(payload, sizeof(payload));
    CHECK(pb_encode(&ostream, RxMsg_fields, &in));

    uint8_t frame[AVCR_FRAME_MAX_PAYLOAD + 16];
    avcr_frame_status status;
    size_t frame_len = avcr_frame_encode(payload, ostream.bytes_written, frame, sizeof(frame), &status);
    CHECK(status == AVCR_FRAME_OK);

    uint8_t decoded[AVCR_FRAME_MAX_PAYLOAD];
    size_t decoded_len = avcr_frame_decode(frame, frame_len, decoded, sizeof(decoded), &status);
    CHECK(status == AVCR_FRAME_OK);

    RxMsg out = RxMsg_init_zero;
    pb_istream_t istream = pb_istream_from_buffer(decoded, decoded_len);
    CHECK(pb_decode(&istream, RxMsg_fields, &out));

    CHECK(out.has_cmd_vel);
    CHECK_NEAR(out.cmd_vel.linear_x, 1.5f, 1e-6f);
    CHECK_NEAR(out.cmd_vel.angular_z, 0.5f, 1e-6f);
    CHECK(out.has_hardware_cmd);
    CHECK(out.hardware_cmd.vacuum_state == 3);
    CHECK(out.hardware_cmd.side_brushcmd == 1);
    CHECK(!out.has_sync_msg_rx);
}

static void test_corrupted_frame_fails_before_decode(void)
{
    RxMsg in = RxMsg_init_zero;
    in.has_cmd_vel = true;
    in.cmd_vel.has_linear_x = true;
    in.cmd_vel.linear_x = 2.0f;

    uint8_t payload[AVCR_FRAME_MAX_PAYLOAD];
    pb_ostream_t ostream = pb_ostream_from_buffer(payload, sizeof(payload));
    CHECK(pb_encode(&ostream, RxMsg_fields, &in));

    uint8_t frame[AVCR_FRAME_MAX_PAYLOAD + 16];
    avcr_frame_status status;
    size_t frame_len = avcr_frame_encode(payload, ostream.bytes_written, frame, sizeof(frame), &status);
    CHECK(status == AVCR_FRAME_OK);

    frame[0] ^= 0x01;

    uint8_t decoded[AVCR_FRAME_MAX_PAYLOAD];
    size_t decoded_len = avcr_frame_decode(frame, frame_len, decoded, sizeof(decoded), &status);
    CHECK(status != AVCR_FRAME_OK);
    CHECK(decoded_len == 0);
}

int main(void)
{
    printf("test_proto\n");
    RUN(test_tx_roundtrip);
    RUN(test_rx_roundtrip);
    RUN(test_corrupted_frame_fails_before_decode);
    TEST_REPORT();
}
