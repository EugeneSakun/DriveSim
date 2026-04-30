#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ModbusTcp.h"
#include "Vehicle.h"
#include "nanomodbus.h"
#include "tinycthread.h"

enum
{
    DISCRETE_INPUT_ADDRESS = 0,
    TORQUE_REGISTER_ADDRESS = 0,
    PARAM_MASS_ADDRESS = 0,
    PARAM_FRONTAL_AREA_ADDRESS = 1,
    PARAM_ROLLING_RESISTANCE_ADDRESS = 2,
    PARAM_MOTOR_MAX_SPEED_ADDRESS = 3,
    PARAM_MOTOR_MAX_TORQUE_ADDRESS = 4,
    PARAM_GEAR_RATIO_ADDRESS = 5,
    PARAM_WHEEL_RADIUS_ADDRESS = 6,
    PARAM_COUNT = 7,
    PORT = 5020
};

typedef struct
{
    Vehicle vehicle;
    mtx_t vehicle_mutex;
    bool discrete_input;
} AppContext;

static int heartbeat_thread(void* arg)
{
    AppContext* context = (AppContext*) arg;
    struct timespec interval = {
        .tv_sec = 1,
        .tv_nsec = 0
    };

    while (1)
    {
        double speed_m_per_s;

        mtx_lock(&context->vehicle_mutex);
        speed_m_per_s = Vehicle_get_speed(&context->vehicle);
        mtx_unlock(&context->vehicle_mutex);

        printf("Vehicle speed: %.2f m/s\n", speed_m_per_s);
        fflush(stdout);
        thrd_sleep(&interval, NULL);
    }

    return 0;
}

static int vehicle_simulation_thread(void* arg)
{
    AppContext* context = (AppContext*) arg;
    struct timespec interval = {
        .tv_sec = 0,
        .tv_nsec = 20000000
    };

    while (1)
    {
        mtx_lock(&context->vehicle_mutex);
        Vehicle_step_50hz(&context->vehicle);
        mtx_unlock(&context->vehicle_mutex);
        thrd_sleep(&interval, NULL);
    }

    return 0;
}

static nmbs_error expect_single_register(uint16_t address, uint16_t quantity, uint16_t expected_address)
{
    if (address != expected_address || quantity != 1)
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;

    return NMBS_ERROR_NONE;
}

static uint16_t encode_scaled(double value, double scale)
{
    if (value <= 0.0)
        return 0;

    return (uint16_t) (value * scale + 0.5);
}

static bool is_torque_command_valid(const AppContext* context, int16_t torque_command)
{
    const double max_torque = context->vehicle.params.motor_max_torque_nm;
    return torque_command >= -(int16_t) max_torque && torque_command <= (int16_t) max_torque;
}

static nmbs_error read_discrete_inputs(
    uint16_t address,
    uint16_t quantity,
    nmbs_bitfield inputs_out,
    uint8_t unit_id,
    void* arg)
{
    (void) unit_id;

    AppContext* context = (AppContext*) arg;
    nmbs_error err = expect_single_register(address, quantity, DISCRETE_INPUT_ADDRESS);
    if (err != NMBS_ERROR_NONE)
        return err;

    memset(inputs_out, 0, 1);
    nmbs_bitfield_write(inputs_out, 0, context->discrete_input);
    return NMBS_ERROR_NONE;
}

static nmbs_error read_holding_registers(
    uint16_t address,
    uint16_t quantity,
    uint16_t* registers_out,
    uint8_t unit_id,
    void* arg)
{
    (void) unit_id;

    AppContext* context = (AppContext*) arg;
    nmbs_error err = expect_single_register(address, quantity, TORQUE_REGISTER_ADDRESS);
    if (err != NMBS_ERROR_NONE)
        return err;

    mtx_lock(&context->vehicle_mutex);
    registers_out[0] = (uint16_t) ((int16_t) context->vehicle.commanded_torque_nm);
    mtx_unlock(&context->vehicle_mutex);
    return NMBS_ERROR_NONE;
}

static nmbs_error read_input_registers(
    uint16_t address,
    uint16_t quantity,
    uint16_t* registers_out,
    uint8_t unit_id,
    void* arg)
{
    (void) unit_id;

    AppContext* context = (AppContext*) arg;
    uint16_t i;

    if (quantity < 1)
        return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;

    if ((uint32_t) address + (uint32_t) quantity > PARAM_COUNT)
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;

    mtx_lock(&context->vehicle_mutex);

    for (i = 0; i < quantity; ++i)
    {
        switch (address + i)
        {
        case PARAM_MASS_ADDRESS:
            registers_out[i] = encode_scaled(context->vehicle.params.mass_kg, 1.0);
            break;
        case PARAM_FRONTAL_AREA_ADDRESS:
            registers_out[i] = encode_scaled(context->vehicle.params.frontal_area_m2, 1000.0);
            break;
        case PARAM_ROLLING_RESISTANCE_ADDRESS:
            registers_out[i] = encode_scaled(context->vehicle.params.rolling_resistance_coefficient, 10000.0);
            break;
        case PARAM_MOTOR_MAX_SPEED_ADDRESS:
            registers_out[i] = encode_scaled(context->vehicle.params.motor_max_speed_rad_per_sec, 1.0);
            break;
        case PARAM_MOTOR_MAX_TORQUE_ADDRESS:
            registers_out[i] = encode_scaled(context->vehicle.params.motor_max_torque_nm, 1.0);
            break;
        case PARAM_GEAR_RATIO_ADDRESS:
            registers_out[i] = encode_scaled(context->vehicle.params.gear_ratio, 100.0);
            break;
        case PARAM_WHEEL_RADIUS_ADDRESS:
            registers_out[i] = encode_scaled(context->vehicle.params.wheel_radius_m, 1000.0);
            break;
        default:
            mtx_unlock(&context->vehicle_mutex);
            return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }
    }

    mtx_unlock(&context->vehicle_mutex);
    return NMBS_ERROR_NONE;
}

static nmbs_error write_single_register(uint16_t address, uint16_t value, uint8_t unit_id, void* arg)
{
    (void) unit_id;

    AppContext* context = (AppContext*) arg;
    int16_t torque_command = (int16_t) value;
    nmbs_error err = expect_single_register(address, 1, TORQUE_REGISTER_ADDRESS);
    if (err != NMBS_ERROR_NONE)
        return err;

    mtx_lock(&context->vehicle_mutex);
    if (!is_torque_command_valid(context, torque_command))
    {
        context->discrete_input = true;
        mtx_unlock(&context->vehicle_mutex);
        printf("Invalid torque command received: %d Nm\n", torque_command);
        return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
    }

    context->discrete_input = false;
    Vehicle_set_torque(&context->vehicle, (double) torque_command);
    mtx_unlock(&context->vehicle_mutex);

    printf("Torque command written: %d Nm\n", torque_command);
    return NMBS_ERROR_NONE;
}

static nmbs_error write_multiple_registers(
    uint16_t address,
    uint16_t quantity,
    const uint16_t* registers,
    uint8_t unit_id,
    void* arg)
{
    (void) unit_id;

    AppContext* context = (AppContext*) arg;
    int16_t torque_command;
    nmbs_error err = expect_single_register(address, quantity, TORQUE_REGISTER_ADDRESS);
    if (err != NMBS_ERROR_NONE)
        return err;

    torque_command = (int16_t) registers[0];

    mtx_lock(&context->vehicle_mutex);
    if (!is_torque_command_valid(context, torque_command))
    {
        context->discrete_input = true;
        mtx_unlock(&context->vehicle_mutex);
        printf("Invalid torque command received: %d Nm\n", torque_command);
        return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
    }

    context->discrete_input = false;
    Vehicle_set_torque(&context->vehicle, (double) torque_command);
    mtx_unlock(&context->vehicle_mutex);

    printf("Torque command written: %d Nm\n", torque_command);
    return NMBS_ERROR_NONE;
}

int main(void)
{
    AppContext app;
    VehicleParams vehicle_params = {
        .mass_kg = 1600.0,
        .frontal_area_m2 = 2.2,
        .rolling_resistance_coefficient = 0.015,
        .motor_max_speed_rad_per_sec = 1200.0,
        .motor_max_torque_nm = 220.0,
        .gear_ratio = 9.1,
        .wheel_radius_m = 0.31
    };
    thrd_t heartbeat;
    thrd_t simulation;

    Vehicle_init(&app.vehicle, &vehicle_params);
    Vehicle_set_torque(&app.vehicle, 120.0);
    app.discrete_input = false;

    if (mtx_init(&app.vehicle_mutex, mtx_plain) != thrd_success)
    {
        fprintf(stderr, "mtx_init failed\n");
        return 1;
    }

    if (thrd_create(&simulation, vehicle_simulation_thread, &app) != thrd_success)
    {
        fprintf(stderr, "thrd_create failed\n");
        mtx_destroy(&app.vehicle_mutex);
        return 1;
    }

    if (thrd_detach(simulation) != thrd_success)
    {
        fprintf(stderr, "thrd_detach failed\n");
        mtx_destroy(&app.vehicle_mutex);
        return 1;
    }

    if (thrd_create(&heartbeat, heartbeat_thread, &app) != thrd_success)
    {
        fprintf(stderr, "thrd_create failed\n");
        mtx_destroy(&app.vehicle_mutex);
        return 1;
    }

    if (thrd_detach(heartbeat) != thrd_success)
    {
        fprintf(stderr, "thrd_detach failed\n");
        mtx_destroy(&app.vehicle_mutex);
        return 1;
    }

    printf("Discrete input 0 = accelerator pedal position sensor failure\n");
    printf("Holding register 0 = torque command in Nm (signed int16)\n");
    printf("Input registers 0..6 = mass, area*1000, crr*10000, max motor speed, max torque, gear ratio*100, wheel radius*1000\n");

    ModbusTcpConfig config;
    ModbusTcp_config_init(&config);
    config.port = PORT;

    ModbusTcpCallbacks callbacks;
    ModbusTcp_callbacks_init(&callbacks);
    callbacks.read_discrete_inputs = read_discrete_inputs;
    callbacks.read_holding_registers = read_holding_registers;
    callbacks.read_input_registers = read_input_registers;
    callbacks.write_single_register = write_single_register;
    callbacks.write_multiple_registers = write_multiple_registers;
    callbacks.user_data = &app;

    return ModbusTcp_run(&config, &callbacks);
}
