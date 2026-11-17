#include <math.h>
#include <string.h>

#include "test_util.h"
#include "timerEncoder.h"

#define TICKS_PER_QUARTER_REV (TICKS_PER_REV / 4)

static void test_straight_line(void)
{
    TIM_HandleTypeDef left = {0};
    TIM_HandleTypeDef right = {0};
    Odometry_t odom;

    Odom_Init(&odom, &left, &right);

    left.CNT = (uint32_t)TICKS_PER_QUARTER_REV;
    right.CNT = (uint32_t)TICKS_PER_QUARTER_REV;

    Odom_Update(&odom, 0.013f);

    float expected_distance = 2.0f * PI_f * WHEEL_radius_M / 4.0f;

    CHECK_NEAR(odom.x, expected_distance, 1e-4f);
    CHECK_NEAR(odom.y, 0.0f, 1e-4f);
    CHECK_NEAR(odom.theta_rad, 0.0f, 1e-5f);
    CHECK_NEAR(odom.linear_x, expected_distance / 0.013f, 1e-2f);
    CHECK_NEAR(odom.angular_z, 0.0f, 1e-5f);
}

static void test_turn_in_place(void)
{
    TIM_HandleTypeDef left = {0};
    TIM_HandleTypeDef right = {0};
    Odometry_t odom;

    Odom_Init(&odom, &left, &right);

    /* Left wheel backwards, right wheel forwards, same magnitude. */
    left.CNT = (uint32_t)(-(int32_t)TICKS_PER_QUARTER_REV);
    right.CNT = (uint32_t)TICKS_PER_QUARTER_REV;

    Odom_Update(&odom, 0.013f);

    float wheel_distance = 2.0f * PI_f * WHEEL_radius_M / 4.0f;
    float expected_theta = (2.0f * wheel_distance) / WHEELBASE;

    CHECK_NEAR(odom.x, 0.0f, 1e-4f);
    CHECK_NEAR(odom.y, 0.0f, 1e-4f);
    CHECK_NEAR(odom.linear_x, 0.0f, 1e-4f);
    CHECK_NEAR(odom.theta_rad, expected_theta, 1e-4f);
    CHECK_NEAR(odom.angular_z, expected_theta / 0.013f, 1e-2f);
}

static void test_rpm_is_smoothed_and_signed(void)
{
    TIM_HandleTypeDef left = {0};
    TIM_HandleTypeDef right = {0};
    Odometry_t odom;

    Odom_Init(&odom, &left, &right);
    left.CNT = (uint32_t)TICKS_PER_QUARTER_REV;

    Odom_Update(&odom, 1.0f);

    float raw_rpm = ((float)TICKS_PER_QUARTER_REV / (float)TICKS_PER_REV) * 60.0f;

    /* First sample blends against zero: EMA_ALPHA * raw. */
    CHECK_NEAR(odom.measured_RPM_left, RPM_EMA_ALPHA * raw_rpm, 1e-3f);
    CHECK(odom.measured_RPM_left > 0.0f);
    CHECK_NEAR(odom.measured_RPM_right, 0.0f, 1e-6f);
}

static void test_heading_stays_wrapped(void)
{
    TIM_HandleTypeDef left = {0};
    TIM_HandleTypeDef right = {0};
    Odometry_t odom;

    Odom_Init(&odom, &left, &right);

    /* Repeated in-place turns must keep theta within [-pi, pi]. */
    for (int i = 0; i < 40; i++)
    {
        left.CNT -= (uint32_t)TICKS_PER_QUARTER_REV;
        right.CNT += (uint32_t)TICKS_PER_QUARTER_REV;
        Odom_Update(&odom, 0.013f);
        CHECK(odom.theta_rad >= -PI_f - 1e-3f);
        CHECK(odom.theta_rad <= PI_f + 1e-3f);
    }
}

int main(void)
{
    printf("test_odometry\n");
    RUN(test_straight_line);
    RUN(test_turn_in_place);
    RUN(test_rpm_is_smoothed_and_signed);
    RUN(test_heading_stays_wrapped);
    TEST_REPORT();
}
