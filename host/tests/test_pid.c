#include <string.h>

#include "PID.h"
#include "test_util.h"

static void make_pid(PIDController *pid, float kp, float ki, float kd, float kff)
{
    PIDController_Init(pid);
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->kff = kff;
    pid->tau = 0.02f;
    pid->T = 0.01f;
    pid->limMax = 1000.0f;
    pid->limMin = -1000.0f;
    pid->limMaxInt = 200.0f;
    pid->limMinInt = -200.0f;
}

static void test_proportional_only(void)
{
    PIDController pid;
    make_pid(&pid, 2.0f, 0.0f, 0.0f, 0.0f);
    pid.limMax = 1000.0f;

    float out = PIDController_Update(&pid, 10.0f, 0.0f);
    CHECK_NEAR(out, 20.0f, 1e-4f);

    out = PIDController_Update(&pid, 10.0f, 5.0f);
    CHECK_NEAR(out, 10.0f, 1e-4f);
}

static void test_output_clamping(void)
{
    PIDController pid;
    make_pid(&pid, 100.0f, 0.0f, 0.0f, 0.0f);
    pid.limMax = 5.0f;
    pid.limMin = -5.0f;

    CHECK_NEAR(PIDController_Update(&pid, 10.0f, 0.0f), 5.0f, 1e-5f);
    CHECK_NEAR(PIDController_Update(&pid, -10.0f, 0.0f), -5.0f, 1e-5f);
    CHECK(pid.out <= 5.0f && pid.out >= -5.0f);
}

static void test_integrator_anti_windup(void)
{
    PIDController pid;
    make_pid(&pid, 0.0f, 1.0f, 0.0f, 0.0f);
    pid.limMaxInt = 1.0f;
    pid.limMinInt = -1.0f;

    for (int i = 0; i < 50; i++)
    {
        PIDController_Update(&pid, 10.0f, 0.0f);
    }

    CHECK_NEAR(pid.integrator, 1.0f, 1e-5f);
}

static void test_feed_forward_adds_setpoint_scale(void)
{
    PIDController pid;
    make_pid(&pid, 0.0f, 0.0f, 0.0f, 6.0f);

    float out = PIDController_UpdateWithFeedForward(&pid, 10.0f, 0.0f);
    CHECK_NEAR(out, 60.0f, 1e-4f);
}

static void test_derivative_rejects_measurement_step(void)
{
    PIDController pid;
    make_pid(&pid, 0.0f, 0.0f, 1.0f, 0.0f);
    pid.limMax = 10000.0f;
    pid.limMin = -10000.0f;

    /* A jump in measurement must produce a negative derivative kick. */
    float out = PIDController_Update(&pid, 0.0f, 10.0f);
    CHECK(out < 0.0f);
}

static void test_closed_loop_converges(void)
{
    PIDController pid;
    make_pid(&pid, 1.0f, 2.0f, 0.0f, 0.0f);
    pid.limMax = 1000.0f;
    pid.limMin = -1000.0f;
    pid.limMaxInt = 200.0f;
    pid.limMinInt = -200.0f;
    pid.T = 0.01f;

    const float setpoint = 100.0f;
    float measurement = 0.0f;

    for (int i = 0; i < 5000; i++)
    {
        float out = PIDController_Update(&pid, setpoint, measurement);
        /* First-order plant: y' = (gain * u - y) / tau */
        measurement += (2.0f * out - measurement) * (0.01f / 0.05f);
    }

    CHECK_NEAR(measurement, setpoint, 1.0f);
}

int main(void)
{
    printf("test_pid\n");
    RUN(test_proportional_only);
    RUN(test_output_clamping);
    RUN(test_integrator_anti_windup);
    RUN(test_feed_forward_adds_setpoint_scale);
    RUN(test_derivative_rejects_measurement_step);
    RUN(test_closed_loop_converges);
    TEST_REPORT();
}
