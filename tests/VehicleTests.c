#include "unity.h"

#include "Vehicle.h"

static VehicleParams make_simple_params(void)
{
    VehicleParams params = {
        .mass_kg = 1000.0,
        .frontal_area_m2 = 0.0,
        .rolling_resistance_coefficient = 0.0,
        .motor_max_speed_rad_per_sec = 1000.0,
        .motor_max_torque_nm = 100.0,
        .gear_ratio = 4.0,
        .wheel_radius_m = 0.5
    };

    return params;
}

void setUp(void)
{
}

void tearDown(void)
{
}

void test_Vehicle_init_copies_parameters_and_zeros_state(void)
{
    Vehicle vehicle;
    VehicleParams params = make_simple_params();

    Vehicle_init(&vehicle, &params);

    TEST_ASSERT_FLOAT_WITHIN(0.000001f, (float) params.mass_kg, (float) vehicle.params.mass_kg);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, (float) params.frontal_area_m2, (float) vehicle.params.frontal_area_m2);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.0f, (float) vehicle.commanded_torque_nm);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.0f, (float) vehicle.speed_m_per_s);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.0f, (float) vehicle.distance_m);
}

void test_Vehicle_set_torque_clamps_to_motor_limits(void)
{
    Vehicle vehicle;
    VehicleParams params = make_simple_params();

    Vehicle_init(&vehicle, &params);

    Vehicle_set_torque(&vehicle, 150.0);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 100.0f, (float) vehicle.commanded_torque_nm);

    Vehicle_set_torque(&vehicle, -150.0);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, -100.0f, (float) vehicle.commanded_torque_nm);
}

void test_Vehicle_step_50hz_advances_speed_and_distance_with_positive_torque(void)
{
    Vehicle vehicle;
    VehicleParams params = make_simple_params();

    Vehicle_init(&vehicle, &params);
    Vehicle_set_torque(&vehicle, 50.0);

    Vehicle_step_50hz(&vehicle);

    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.008f, (float) vehicle.speed_m_per_s);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.00008f, (float) vehicle.distance_m);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.008f, (float) Vehicle_get_speed(&vehicle));
}

void test_Vehicle_step_50hz_does_not_move_from_rest_when_torque_cannot_overcome_rolling_resistance(void)
{
    Vehicle vehicle;
    VehicleParams params = {
        .mass_kg = 1000.0,
        .frontal_area_m2 = 0.0,
        .rolling_resistance_coefficient = 0.02,
        .motor_max_speed_rad_per_sec = 1000.0,
        .motor_max_torque_nm = 100.0,
        .gear_ratio = 1.0,
        .wheel_radius_m = 1.0
    };

    Vehicle_init(&vehicle, &params);
    Vehicle_set_torque(&vehicle, 10.0);

    Vehicle_step_50hz(&vehicle);

    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.0f, (float) vehicle.speed_m_per_s);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.0f, (float) vehicle.distance_m);
}

void test_Vehicle_step_50hz_stops_applying_torque_above_motor_max_speed(void)
{
    Vehicle vehicle;
    VehicleParams params = make_simple_params();

    Vehicle_init(&vehicle, &params);
    Vehicle_set_torque(&vehicle, 50.0);
    vehicle.speed_m_per_s = 200.0;

    Vehicle_step_50hz(&vehicle);

    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 200.0f, (float) vehicle.speed_m_per_s);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_Vehicle_init_copies_parameters_and_zeros_state);
    RUN_TEST(test_Vehicle_set_torque_clamps_to_motor_limits);
    RUN_TEST(test_Vehicle_step_50hz_advances_speed_and_distance_with_positive_torque);
    RUN_TEST(test_Vehicle_step_50hz_does_not_move_from_rest_when_torque_cannot_overcome_rolling_resistance);
    RUN_TEST(test_Vehicle_step_50hz_stops_applying_torque_above_motor_max_speed);
    return UNITY_END();
}
