#include <pid_controller.h>
#include <unity.h>

void setUp() {
}

void tearDown() {
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
void test_pid_init() {
    const PidController pid;
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getP());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getI());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getD());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getS());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getK());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getSetpoint());

    const pid_error_t error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(error.p, 0.0);
    TEST_ASSERT_EQUAL_FLOAT(error.i, 0.0);
    TEST_ASSERT_EQUAL_FLOAT(error.d, 0.0);
    TEST_ASSERT_EQUAL_FLOAT(error.s, 0.0);
    TEST_ASSERT_EQUAL_FLOAT(error.k, 0.0);
}

void test_pid() {
    PidController pid(pid_constants_t { 5.0F, 3.0F, 1.0F, 0.0F, 0.0F });

    TEST_ASSERT_EQUAL_FLOAT(5.0F, pid.getP());
    TEST_ASSERT_EQUAL_FLOAT(3.0F, pid.getI());
    TEST_ASSERT_EQUAL_FLOAT(1.0F, pid.getD());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getS());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getK());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getSetpoint());

    const float delta_t {0.01F};
    const float input  {0.5F};
    const float output = pid.update(input, delta_t);

    const pid_error_t error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-input * 5.0F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-input * 3.0F * delta_t, error.i);
    TEST_ASSERT_EQUAL_FLOAT(-input * 1.0F / delta_t, error.d);
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i + error.d, output);
}

void test_p_controller()
{
    PidController pid(pid_constants_t { 1.0, 0.0, 0.0, 0.0, 0.0F });
    const float delta_t {1};

    TEST_ASSERT_EQUAL_FLOAT(1.0, pid.getP());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getI());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getD());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getS());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getK());
    TEST_ASSERT_EQUAL_FLOAT(0.0, pid.getSetpoint());

    float output = pid.update(0, delta_t);
    pid_error_t error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(0, error.p);
    TEST_ASSERT_EQUAL_FLOAT(0, error.i);
    TEST_ASSERT_EQUAL_FLOAT(0, error.d);
    TEST_ASSERT_EQUAL_FLOAT(0, error.s);
    TEST_ASSERT_EQUAL_FLOAT(0, error.k);
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i + error.d, output);

    pid.setSetpoint(5.0);
    output = pid.update(0.0, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(5.0, output);
    TEST_ASSERT_EQUAL_FLOAT(5.0, error.p);
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i + error.d, output);

    output = pid.update(1.0, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(4.0, output);
    TEST_ASSERT_EQUAL_FLOAT(4.0, error.p);

    output = pid.update(2.0, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(3.0, output);
    TEST_ASSERT_EQUAL_FLOAT(3.0, error.p);

    output = pid.update(3.0, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(2.0, output);
    TEST_ASSERT_EQUAL_FLOAT(2.0, error.p);

    output = pid.update(4.0, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(1.0, output);
    TEST_ASSERT_EQUAL_FLOAT(1.0, error.p);

    output = pid.update(5.0, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(0.0, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0, error.p);

    output = pid.update(6.0, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-1.0, output);
    TEST_ASSERT_EQUAL_FLOAT(-1.0, error.p);

    output = pid.update(5.0, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(0.0, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0, error.p);
}

void test_pi_controller()
{
    PidController pid(pid_constants_t { 0.3F, 0.2F, 0.0F, 0.0F, 0.0F });
    const float delta_t {1};

    TEST_ASSERT_EQUAL_FLOAT(0.3F, pid.getP());
    TEST_ASSERT_EQUAL_FLOAT(0.2F, pid.getI());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getD());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getS());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getK());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getSetpoint());

    float output = pid.update(0, delta_t);
    pid_error_t error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.i);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.d);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.s);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.k);
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i + error.d, output);

    pid.setSetpoint(5.0F);
    output = pid.update(0.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(1.5F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(5.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(1.0F, error.i);
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.s);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.k);
    TEST_ASSERT_EQUAL_FLOAT(2.5F, output);

    output = pid.update(1.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(1.2F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(4.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(1.8F, error.i); // 1.0 + (5.0 - 1.0) * 0.2
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.s);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.k);
    TEST_ASSERT_EQUAL_FLOAT(3.0F, output);

    output = pid.update(4.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(0.3F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(2.0F, error.i); // 1.8 + (5.0 - 4.0) * 0.2
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.s);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.k);
    TEST_ASSERT_EQUAL_FLOAT(2.3F, output);

    output = pid.update(7.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.6F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(1.6F, error.i); // 2.0 + -2.0 * 0.2
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.s);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.k);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, output);

    output = pid.update(6.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.3F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(1.4F, error.i); // 1.6 + -1.0 * 0.2
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.s);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.k);
    TEST_ASSERT_EQUAL_FLOAT(1.1F, output);

    output = pid.update(5.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(1.4F, error.i); // 1.5 + (0.0 - 1.0) * 0.2 /2
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.s);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.k);
    TEST_ASSERT_EQUAL_FLOAT(1.4F, output);

    output = pid.update(5.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(1.4F, error.i); // 1.4 + (0.0 + 0.0) * 0.2 / 2
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.s);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.k);
    TEST_ASSERT_EQUAL_FLOAT(1.4F, output);
}

void test_update_pi()
{
    PidController pid(pid_constants_t { 0.3F, 0.2F, 0.0F, 0.0F, 0.0F });
    const float delta_t {1};

    TEST_ASSERT_EQUAL_FLOAT(0.3F, pid.getP());
    TEST_ASSERT_EQUAL_FLOAT(0.2F, pid.getI());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getD());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getS());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getK());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getSetpoint());

    float output = pid.update_spi(0, delta_t);
    pid_error_t error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.i);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.d);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.s);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.k);
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i + error.d, output);

    pid.setSetpoint(5.0F);
    output = pid.update_spi(0.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(1.5F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(5.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(1.0F, error.i); // 5.0 * 0.2
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.s);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.k);
    TEST_ASSERT_EQUAL_FLOAT(2.5F, output);

    output = pid.update_spi(1.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(1.2F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(4.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(1.8F, error.i); // 1.0 + 4.0 * 0.2
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.s);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.k);
    TEST_ASSERT_EQUAL_FLOAT(3.0F, output);

    output = pid.update_spi(4.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(0.3F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(2.0F, error.i); // 1.8 + 1.0 * 0.2
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.s);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.k);
    TEST_ASSERT_EQUAL_FLOAT(2.3F, output);

    output = pid.update_spi(7.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.6F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(1.6F, error.i); // 2.0 + -2.0 * 0.2
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.s);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.k);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, output);

    output = pid.update_spi(6.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.3F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(1.4F, error.i); // 1.6 + -1.0 * 0.2
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.s);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.k);
    TEST_ASSERT_EQUAL_FLOAT(1.1F, output);

    output = pid.update_spi(5.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(1.4F, error.i); // 1.4 + 0.0 * 0.2
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.s);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.k);
    TEST_ASSERT_EQUAL_FLOAT(1.4F, output);

    output = pid.update_spi(5.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(1.4F, error.i); // 1.4 + (0.0 + 0.0) * 0.2 / 2
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i, output);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.s);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.k);
    TEST_ASSERT_EQUAL_FLOAT(1.4F, output);
}

void test_integration_on_off()
{
    PidController pid(pid_constants_t { 0.2F, 0.3F, 0.0F, 0.0F, 0.0F });
    const float delta_t {1};

    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getSetpoint());

    float output = pid.update(0.0F, delta_t);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, output);
    pid_error_t error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.i);

    output = pid.update(2.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(-0.6F, error.i); // 0.0 + -2.0* 0.3
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i, output);
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, output);

    output = pid.update(2.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(-1.2F, error.i); // -0.6 +-2.0 * 0.3
    TEST_ASSERT_EQUAL_FLOAT(-1.6F, output);

    // Integration OFF
    pid.switchIntegrationOff();
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.i);

    output = pid.update(0.0F, delta_t);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, output);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.i);

    output = pid.update(2.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.i);
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i, output);
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, output);

    output = pid.update(2.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.i);
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, output);

    // Integration back ON
    pid.switchIntegrationOn();
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.i);

    output = pid.update(0.0F, delta_t);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, output);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.i); // 0.0 + 0.0 * 0.3

    output = pid.update(2.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(-0.6F, error.i); // - 0.3 + (0.0 - 2.0) * 0.3 / 2
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i, output);
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, output);

    output = pid.update(2.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(-1.2F, error.i); // -0.6 + (-2.0 - 2.0) * 0.3 / 2
    TEST_ASSERT_EQUAL_FLOAT(-1.6F, output);

    pid.resetAll();
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getSetpoint());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getPreviousSetpoint());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getSetpointDelta());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getPreviousMeasurement());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.i);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.d);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.s);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.k);
}

void test_integral_limit()
{
    PidController pid(pid_constants_t { 0.2F, 0.3F, 0.0F, 0.0F, 0.0F });
    pid.set_integral_limit(2.0F);
    const float delta_t {1};

    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getSetpoint());

    float output = pid.update(0.0F, delta_t);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, output);
    pid_error_t error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.i);

    output = pid.update(2.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-0.6F, error.i);
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i, output);
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, output);

    output = pid.update(2.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-1.2F, error.i);
    TEST_ASSERT_EQUAL_FLOAT(-1.6F, output);

    output = pid.update(2.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-1.8F, error.i);
    TEST_ASSERT_EQUAL_FLOAT(-2.2F, output);

    output = pid.update(2.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, error.i);
    TEST_ASSERT_EQUAL_FLOAT(-2.4F, output);

    output = pid.update(2.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-2.0F, error.i);
    TEST_ASSERT_EQUAL_FLOAT(-2.4F, output);
}

void test_integral_saturation_positive()
{
    PidController pid(pid_constants_t { 0.2F, 0.3F, 0.0F, 0.0F, 0.0F });
    pid.setoutput_saturation_value(1.5F);
    const float delta_t {1};

    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getSetpoint());

    float output = pid.update(0.0F, delta_t);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, output);
    pid_error_t error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.i);

    output = pid.update(2.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-0.6F, error.i);
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i, output);
    TEST_ASSERT_EQUAL_FLOAT(-1.0F, output);

    output = pid.update(2.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-1.1F, error.i);
    TEST_ASSERT_EQUAL_FLOAT(-1.5F, output);

    output = pid.update(2.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-1.1F, error.i);
    TEST_ASSERT_EQUAL_FLOAT(-1.5F, output);

    output = pid.update(2.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.4F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-1.1F, error.i);
    TEST_ASSERT_EQUAL_FLOAT(-1.5F, output);

    output = pid.update(1.5F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.3F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-1.2F, error.i);
    TEST_ASSERT_EQUAL_FLOAT(-1.5F, output);

    output = pid.update(1.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.2F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-1.3F, error.i);
    TEST_ASSERT_EQUAL_FLOAT(-1.5F, output);

    output = pid.update(0.5F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.1F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-0.5F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(-1.4F, error.i);
    TEST_ASSERT_EQUAL_FLOAT(-1.5F, output);

    output = pid.update(0.2F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.04F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(-0.2F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(-1.46F, error.i);
    TEST_ASSERT_EQUAL_FLOAT(-1.5F, output);

    output = pid.update(0.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(-0.0F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(-1.46F, error.i); // -1.46 + 0.0 * 0.3
    TEST_ASSERT_EQUAL_FLOAT(-1.46F, output);

    output = pid.update(0.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.get_previous_error());
    TEST_ASSERT_EQUAL_FLOAT(-1.46F, error.i); // -1.495 + 0.0 * 0.3
    TEST_ASSERT_EQUAL_FLOAT(-1.46F, output);
}

void test_integral_saturation_negative()
{
    PidController pid(pid_constants_t { 0.2F, 0.3F, 0.0F, 0.0F, 0.0F });
    pid.setoutput_saturation_value(1.5F);
    const float delta_t {1};

    TEST_ASSERT_EQUAL_FLOAT(0.0F, pid.getSetpoint());

    float output = pid.update(0.0F, delta_t);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, output);
    pid_error_t error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, error.i);

    output = pid.update(-2.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(0.4F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(0.6F, error.i);
    TEST_ASSERT_EQUAL_FLOAT(error.p + error.i, output);
    TEST_ASSERT_EQUAL_FLOAT(1.0F, output);

    output = pid.update(-2.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(0.4F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(1.1F, error.i);
    TEST_ASSERT_EQUAL_FLOAT(1.5F, output);

    output = pid.update(-2.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(0.4F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(1.1F, error.i);
    TEST_ASSERT_EQUAL_FLOAT(1.5F, output);

    output = pid.update(-2.0F, delta_t);
    error = pid.get_error();
    TEST_ASSERT_EQUAL_FLOAT(0.4F, error.p);
    TEST_ASSERT_EQUAL_FLOAT(1.1F, error.i);
    TEST_ASSERT_EQUAL_FLOAT(1.5F, output);
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_pid_init);
    RUN_TEST(test_pid);
    RUN_TEST(test_p_controller);
    RUN_TEST(test_pi_controller);
    RUN_TEST(test_update_pi);
    RUN_TEST(test_integration_on_off);
    RUN_TEST(test_integral_limit);
    RUN_TEST(test_integral_saturation_positive);
    RUN_TEST(test_integral_saturation_negative);

    UNITY_END();
}
