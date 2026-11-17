/*
 * avcr-cli - host-side utilities for the AVCR serial protocol and controller.
 *
 * Everything here runs the same C sources as the firmware (COBS, nanopb,
 * PID, odometry through a HAL shim), so the tool can be used to inspect real
 * traffic captured with a logic analyzer or to replay frames offline.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "avcr/crc32_mpeg2.h"
#include "avcr/frame.h"
#include "cobs.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "PID.h"
#include "rx.pb.h"
#include "tx.pb.h"

#define CLI_MAX_BUF 1024

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static int parse_hex(const char *text, uint8_t *out, size_t cap)
{
    size_t len = 0;
    int high = -1;

    if (text == NULL)
    {
        return -1;
    }

    if (text[0] == '0' && (text[1] == 'x' || text[1] == 'X'))
    {
        text += 2;
    }

    for (const char *p = text; *p != '\0'; p++)
    {
        if (*p == ' ' || *p == ':' || *p == '_' || *p == ',')
        {
            continue;
        }

        int value = hex_nibble(*p);
        if (value < 0)
        {
            return -1;
        }

        if (high < 0)
        {
            high = value;
        }
        else
        {
            if (len >= cap)
            {
                return -1;
            }
            out[len++] = (uint8_t)((high << 4) | value);
            high = -1;
        }
    }

    if (high >= 0)
    {
        return -1;
    }

    return (int)len;
}

static void print_hex(const char *label, const uint8_t *data, size_t len)
{
    printf("%s", label);
    for (size_t i = 0; i < len; i++)
    {
        printf("%02x", data[i]);
    }
    printf("\n");
}

static void print_tx(const TxMsg *msg)
{
    if (msg->has_odom)
    {
        printf("odom.x=%.4f odom.y=%.4f odom.yaw=%.4f linear_x=%.4f angular_z=%.4f ts_us=%u\n",
               (double)msg->odom.x, (double)msg->odom.y, (double)msg->odom.yaw,
               (double)msg->odom.linear_x, (double)msg->odom.angular_z,
               (unsigned)msg->odom.odom_time_stamp_us);
    }
    if (msg->has_imu)
    {
        printf("imu.ax=%.4f imu.ay=%.4f imu.az=%.4f gx=%.4f gy=%.4f gz=%.4f ts_us=%u\n",
               (double)msg->imu.ax, (double)msg->imu.ay, (double)msg->imu.az,
               (double)msg->imu.gx, (double)msg->imu.gy, (double)msg->imu.gz,
               (unsigned)msg->imu.mpu6500_time_stamp_us);
    }
    if (msg->has_tofArray)
    {
        printf("tof.cliff1=%u cliff2=%u cliff3=%u cliff4=%u ts_us=%u\n",
               (unsigned)msg->tofArray.cliff1, (unsigned)msg->tofArray.cliff2,
               (unsigned)msg->tofArray.cliff3, (unsigned)msg->tofArray.cliff4,
               (unsigned)msg->tofArray.tof_time_stamp_us);
    }
    if (msg->has_battery)
    {
        printf("battery.soc=%.2f charging=%u\n",
               (double)msg->battery.StateOfCharge, (unsigned)msg->battery.ChargingState);
    }
    if (msg->has_sync_msg_tx)
    {
        printf("sync.pi_ts_us=%u stm32_rx_us=%u stm32_tx_us=%u\n",
               (unsigned)msg->sync_msg_tx.pi_initial_time_stamp_us,
               (unsigned)msg->sync_msg_tx.stm32_rx_time_stamp_us,
               (unsigned)msg->sync_msg_tx.stm32_tx_time_stamp_us);
    }
}

static void print_rx(const RxMsg *msg)
{
    if (msg->has_cmd_vel)
    {
        printf("cmd_vel.linear_x=%.4f angular_z=%.4f\n",
               (double)msg->cmd_vel.linear_x, (double)msg->cmd_vel.angular_z);
    }
    if (msg->has_hardware_cmd)
    {
        printf("hardware.vacuum_state=%u side_brush=%u\n",
               (unsigned)msg->hardware_cmd.vacuum_state,
               (unsigned)msg->hardware_cmd.side_brushcmd);
    }
    if (msg->has_sync_msg_rx)
    {
        printf("sync.pi_ts_us=%u stm32_rx_us=%u stm32_tx_us=%u\n",
               (unsigned)msg->sync_msg_rx.pi_initial_time_stamp_us,
               (unsigned)msg->sync_msg_rx.stm32_rx_time_stamp_us,
               (unsigned)msg->sync_msg_rx.stm32_tx_time_stamp_us);
    }
}

static int cmd_cobs(int argc, char **argv)
{
    if (argc < 3)
    {
        fprintf(stderr, "usage: avcr-cli cobs-encode|cobs-decode <hex>\n");
        return 2;
    }

    uint8_t input[CLI_MAX_BUF];
    int len = parse_hex(argv[2], input, sizeof(input));
    if (len < 0)
    {
        fprintf(stderr, "error: invalid hex input\n");
        return 1;
    }

    uint8_t output[CLI_MAX_BUF];
    if (strcmp(argv[1], "cobs-encode") == 0)
    {
        cobs_encode_result res = cobs_encode(output, sizeof(output), input, (size_t)len);
        if (res.status != COBS_ENCODE_OK)
        {
            fprintf(stderr, "error: cobs encode failed (%d)\n", (int)res.status);
            return 1;
        }
        print_hex("encoded=", output, res.out_len);
    }
    else
    {
        cobs_decode_result res = cobs_decode(output, sizeof(output), input, (size_t)len);
        if (res.status != COBS_DECODE_OK)
        {
            fprintf(stderr, "error: cobs decode failed (%d)\n", (int)res.status);
            return 1;
        }
        print_hex("decoded=", output, res.out_len);
    }

    return 0;
}

static int cmd_frame(int argc, char **argv)
{
    if (argc < 3)
    {
        fprintf(stderr, "usage: avcr-cli frame-encode|frame-decode <hex>\n");
        return 2;
    }

    uint8_t input[CLI_MAX_BUF];
    int len = parse_hex(argv[2], input, sizeof(input));
    if (len < 0)
    {
        fprintf(stderr, "error: invalid hex input\n");
        return 1;
    }

    uint8_t output[CLI_MAX_BUF];
    avcr_frame_status status;

    if (strcmp(argv[1], "frame-encode") == 0)
    {
        size_t frame_len = avcr_frame_encode(input, (size_t)len, output, sizeof(output), &status);
        if (frame_len == 0)
        {
            fprintf(stderr, "error: %s\n", avcr_frame_status_str(status));
            return 1;
        }
        printf("crc=0x%08x\n", (unsigned)avcr_crc32_mpeg2(input, (size_t)len));
        print_hex("frame=", output, frame_len);
    }
    else
    {
        size_t payload_len = avcr_frame_decode(input, (size_t)len, output, sizeof(output), &status);
        if (payload_len == 0)
        {
            fprintf(stderr, "error: %s\n", avcr_frame_status_str(status));
            return 1;
        }
        printf("status=%s len=%u\n", avcr_frame_status_str(status), (unsigned)payload_len);
        print_hex("payload=", output, payload_len);
    }

    return 0;
}

static int cmd_tx_demo(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    TxMsg msg = TxMsg_init_zero;
    msg.has_odom = true;
    msg.odom.has_x = msg.odom.has_y = msg.odom.has_yaw = true;
    msg.odom.has_linear_x = msg.odom.has_angular_z = true;
    msg.odom.has_odom_time_stamp_us = true;
    msg.odom.x = 1.25f;
    msg.odom.y = -0.5f;
    msg.odom.yaw = 0.785f;
    msg.odom.linear_x = 0.22f;
    msg.odom.angular_z = 0.1f;
    msg.odom.odom_time_stamp_us = 123456u;

    msg.has_imu = true;
    msg.imu.has_ax = msg.imu.has_ay = msg.imu.has_az = true;
    msg.imu.has_gx = msg.imu.has_gy = msg.imu.has_gz = true;
    msg.imu.has_mpu6500_time_stamp_us = true;
    msg.imu.ax = 0.01f;
    msg.imu.ay = -0.02f;
    msg.imu.az = 9.81f;
    msg.imu.gx = 0.001f;
    msg.imu.gy = 0.0f;
    msg.imu.gz = -0.002f;
    msg.imu.mpu6500_time_stamp_us = 123500u;

    msg.has_tofArray = true;
    msg.tofArray.has_cliff1 = msg.tofArray.has_cliff2 = true;
    msg.tofArray.has_cliff3 = msg.tofArray.has_cliff4 = true;
    msg.tofArray.has_tof_time_stamp_us = true;
    msg.tofArray.cliff1 = 1;
    msg.tofArray.cliff2 = 0;
    msg.tofArray.cliff3 = 0;
    msg.tofArray.cliff4 = 1;
    msg.tofArray.tof_time_stamp_us = 123600u;

    msg.has_battery = true;
    msg.battery.has_StateOfCharge = true;
    msg.battery.has_ChargingState = true;
    msg.battery.StateOfCharge = 70.0f;
    msg.battery.ChargingState = 0;

    msg.has_sync_msg_tx = true;
    msg.sync_msg_tx.has_pi_initial_time_stamp_us = true;
    msg.sync_msg_tx.has_stm32_rx_time_stamp_us = true;
    msg.sync_msg_tx.has_stm32_tx_time_stamp_us = true;
    msg.sync_msg_tx.pi_initial_time_stamp_us = 200000u;
    msg.sync_msg_tx.stm32_rx_time_stamp_us = 123700u;
    msg.sync_msg_tx.stm32_tx_time_stamp_us = 123800u;

    uint8_t payload[AVCR_FRAME_MAX_PAYLOAD];
    pb_ostream_t stream = pb_ostream_from_buffer(payload, sizeof(payload));
    if (!pb_encode(&stream, TxMsg_fields, &msg))
    {
        fprintf(stderr, "error: pb_encode failed: %s\n", PB_GET_ERROR(&stream));
        return 1;
    }

    uint8_t frame[CLI_MAX_BUF];
    avcr_frame_status status;
    size_t frame_len = avcr_frame_encode(payload, stream.bytes_written, frame, sizeof(frame), &status);
    if (frame_len == 0)
    {
        fprintf(stderr, "error: %s\n", avcr_frame_status_str(status));
        return 1;
    }

    printf("payload_len=%u frame_len=%u\n", (unsigned)stream.bytes_written, (unsigned)frame_len);
    print_hex("payload=", payload, stream.bytes_written);
    print_hex("frame=", frame, frame_len);
    print_tx(&msg);
    return 0;
}

static int cmd_tx_decode(int argc, char **argv)
{
    if (argc < 3)
    {
        fprintf(stderr, "usage: avcr-cli tx-decode <hex-frame>\n");
        return 2;
    }

    uint8_t frame[CLI_MAX_BUF];
    int len = parse_hex(argv[2], frame, sizeof(frame));
    if (len < 0)
    {
        fprintf(stderr, "error: invalid hex input\n");
        return 1;
    }

    uint8_t payload[CLI_MAX_BUF];
    avcr_frame_status status;
    size_t payload_len = avcr_frame_decode(frame, (size_t)len, payload, sizeof(payload), &status);
    if (payload_len == 0)
    {
        fprintf(stderr, "error: %s\n", avcr_frame_status_str(status));
        return 1;
    }

    const char *kind = "unknown";
    if (argc >= 4 && strcmp(argv[3], "rx") == 0)
    {
        kind = "rx";
    }
    else if (argc >= 4 && strcmp(argv[3], "tx") == 0)
    {
        kind = "tx";
    }
    else if (payload_len > 0 && (payload[0] & 0x07) != 0)
    {
        /* Best-effort guess: TxMsg uses field 1..5, RxMsg also uses 1..3. */
        kind = "tx";
    }

    pb_istream_t stream = pb_istream_from_buffer(payload, payload_len);

    if (strcmp(kind, "rx") == 0)
    {
        RxMsg msg = RxMsg_init_zero;
        if (!pb_decode(&stream, RxMsg_fields, &msg))
        {
            fprintf(stderr, "error: pb_decode(RxMsg) failed: %s\n", PB_GET_ERROR(&stream));
            return 1;
        }
        printf("message=RxMsg payload_len=%u\n", (unsigned)payload_len);
        print_rx(&msg);
        return 0;
    }

    TxMsg msg = TxMsg_init_zero;
    if (!pb_decode(&stream, TxMsg_fields, &msg))
    {
        fprintf(stderr, "error: pb_decode(TxMsg) failed: %s\n", PB_GET_ERROR(&stream));
        return 1;
    }
    printf("message=TxMsg payload_len=%u\n", (unsigned)payload_len);
    print_tx(&msg);
    return 0;
}

static int cmd_rx_demo(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    RxMsg msg = RxMsg_init_zero;
    msg.has_cmd_vel = true;
    msg.cmd_vel.has_linear_x = true;
    msg.cmd_vel.has_angular_z = true;
    msg.cmd_vel.linear_x = 1.5f;
    msg.cmd_vel.angular_z = 0.5f;
    msg.has_hardware_cmd = true;
    msg.hardware_cmd.has_vacuum_state = true;
    msg.hardware_cmd.has_side_brushcmd = true;
    msg.hardware_cmd.vacuum_state = 2;
    msg.hardware_cmd.side_brushcmd = 1;
    msg.has_sync_msg_rx = true;
    msg.sync_msg_rx.has_pi_initial_time_stamp_us = true;
    msg.sync_msg_rx.pi_initial_time_stamp_us = 200000u;

    uint8_t payload[AVCR_FRAME_MAX_PAYLOAD];
    pb_ostream_t stream = pb_ostream_from_buffer(payload, sizeof(payload));
    if (!pb_encode(&stream, RxMsg_fields, &msg))
    {
        fprintf(stderr, "error: pb_encode failed: %s\n", PB_GET_ERROR(&stream));
        return 1;
    }

    uint8_t frame[CLI_MAX_BUF];
    avcr_frame_status status;
    size_t frame_len = avcr_frame_encode(payload, stream.bytes_written, frame, sizeof(frame), &status);
    if (frame_len == 0)
    {
        fprintf(stderr, "error: %s\n", avcr_frame_status_str(status));
        return 1;
    }

    printf("payload_len=%u frame_len=%u\n", (unsigned)stream.bytes_written, (unsigned)frame_len);
    print_hex("frame=", frame, frame_len);
    print_rx(&msg);
    return 0;
}

static void usage(void)
{
    printf(
        "avcr-cli - host utilities for the AVCR serial protocol\n"
        "\n"
        "Commands:\n"
        "  cobs-encode <hex>            COBS-encode raw bytes\n"
        "  cobs-decode <hex>            COBS-decode raw bytes\n"
        "  frame-encode <hex>           wrap a payload in COBS + CRC32 + delimiter\n"
        "  frame-decode <hex>           validate a frame and print its payload\n"
        "  tx-demo                      build a sample TxMsg and print its frame\n"
        "  rx-demo                      build a sample RxMsg and print its frame\n"
        "  tx-decode <hex> [tx|rx]      decode a frame into protobuf fields\n"
        "  pid-step [options]           simulate a wheel-speed PID loop (CSV)\n"
        "\n"
        "pid-step options:\n"
        "  --kp F --ki F --kd F --kff F   controller gains (default 1.5/0.03/0/6)\n"
        "  --dt F                        sample time in seconds (default 0.013)\n"
        "  --steps N                     iterations (default 300)\n"
        "  --setpoint F                  target RPM (default 300)\n"
        "  --tau F                       plant time constant (default 0.08)\n"
        "  --gain F                      plant gain (default 1.0)\n"
        "  --lim F --lim-int F           output and integrator limits\n"
        "\n"
        "Examples:\n"
        "  avcr-cli frame-encode 08960112041f\n"
        "  avcr-cli tx-demo\n"
        "  avcr-cli pid-step --setpoint 250 --steps 400 > step.csv\n");
}

static int cmd_pid_step(int argc, char **argv)
{
    float kp = 1.5f, ki = 0.03f, kd = 0.0f, kff = 6.0f;
    float dt = 0.013f, setpoint = 300.0f, tau = 0.08f, gain = 1.0f;
    float lim = 1000.0f, lim_int = 200.0f;
    int steps = 300;

    for (int i = 2; i < argc; i++)
    {
        const char *arg = argv[i];
        if (strcmp(arg, "--kp") == 0 && i + 1 < argc)        kp = (float)atof(argv[++i]);
        else if (strcmp(arg, "--ki") == 0 && i + 1 < argc)   ki = (float)atof(argv[++i]);
        else if (strcmp(arg, "--kd") == 0 && i + 1 < argc)   kd = (float)atof(argv[++i]);
        else if (strcmp(arg, "--kff") == 0 && i + 1 < argc)  kff = (float)atof(argv[++i]);
        else if (strcmp(arg, "--dt") == 0 && i + 1 < argc)   dt = (float)atof(argv[++i]);
        else if (strcmp(arg, "--steps") == 0 && i + 1 < argc) steps = atoi(argv[++i]);
        else if (strcmp(arg, "--setpoint") == 0 && i + 1 < argc) setpoint = (float)atof(argv[++i]);
        else if (strcmp(arg, "--tau") == 0 && i + 1 < argc)  tau = (float)atof(argv[++i]);
        else if (strcmp(arg, "--gain") == 0 && i + 1 < argc) gain = (float)atof(argv[++i]);
        else if (strcmp(arg, "--lim") == 0 && i + 1 < argc)  lim = (float)atof(argv[++i]);
        else if (strcmp(arg, "--lim-int") == 0 && i + 1 < argc) lim_int = (float)atof(argv[++i]);
        else
        {
            fprintf(stderr, "error: unknown option %s\n", arg);
            return 2;
        }
    }

    if (steps <= 0 || dt <= 0.0f)
    {
        fprintf(stderr, "error: --steps and --dt must be positive\n");
        return 2;
    }

    PIDController pid;
    PIDController_Init(&pid);
    pid.Kp = kp;
    pid.Ki = ki;
    pid.Kd = kd;
    pid.kff = kff;
    pid.tau = 0.02f;
    pid.T = dt;
    pid.limMax = lim;
    pid.limMin = -lim;
    pid.limMaxInt = lim_int;
    pid.limMinInt = -lim_int;

    float measurement = 0.0f;

    printf("step,time_s,setpoint,measurement,output\n");
    for (int step = 0; step < steps; step++)
    {
        float output = PIDController_UpdateWithFeedForward(&pid, setpoint, measurement);

        if (tau > 0.0f)
        {
            measurement += (gain * output - measurement) * (dt / tau);
        }
        else
        {
            measurement = gain * output;
        }

        printf("%d,%.6f,%.6f,%.6f,%.6f\n",
               step, (double)((step + 1) * dt), (double)setpoint,
               (double)measurement, (double)output);
    }

    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        usage();
        return 2;
    }

    const char *command = argv[1];

    if (strcmp(command, "help") == 0 || strcmp(command, "--help") == 0 || strcmp(command, "-h") == 0)
    {
        usage();
        return 0;
    }
    if (strcmp(command, "cobs-encode") == 0 || strcmp(command, "cobs-decode") == 0)
    {
        return cmd_cobs(argc, argv);
    }
    if (strcmp(command, "frame-encode") == 0 || strcmp(command, "frame-decode") == 0)
    {
        return cmd_frame(argc, argv);
    }
    if (strcmp(command, "tx-demo") == 0)
    {
        return cmd_tx_demo(argc, argv);
    }
    if (strcmp(command, "rx-demo") == 0)
    {
        return cmd_rx_demo(argc, argv);
    }
    if (strcmp(command, "tx-decode") == 0)
    {
        return cmd_tx_decode(argc, argv);
    }
    if (strcmp(command, "pid-step") == 0)
    {
        return cmd_pid_step(argc, argv);
    }

    fprintf(stderr, "error: unknown command '%s'\n\n", command);
    usage();
    return 2;
}
