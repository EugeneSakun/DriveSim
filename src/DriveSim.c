#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "ModbusTcp.h"
#include "Vehicle.h"
#include "nanomodbus.h"
#include "tinycthread.h"

enum
{
    /* TCP порт сервера Modbus */
    PORT = 5020,

    /* Адреси Modbus регістрів */

    /* Дискретні входи (тільки читання) */
    ACCELERATOR_SENSOR_FAILURE_ADDRESS = 0,

    /* 16-бітні вхідні регістри (тільки читання) */
    PARAM_MASS_ADDRESS = 0,
    PARAM_FRONTAL_AREA_ADDRESS = 1,
    PARAM_ROLLING_RESISTANCE_ADDRESS = 2,
    PARAM_MOTOR_MAX_SPEED_ADDRESS = 3,
    PARAM_MOTOR_MAX_TORQUE_ADDRESS = 4,
    PARAM_GEAR_RATIO_ADDRESS = 5,
    PARAM_WHEEL_RADIUS_ADDRESS = 6,
    /* Кількість 16-бітних вхідних регістрів */
    PARAM_COUNT = 7,

    /* 16-бітні вихідні регістри (читання та запис) */
    TORQUE_REGISTER_ADDRESS = 0,
};

typedef struct
{
    /* Усі потоки працюють з одним спільним станом через цю структуру. */
    Vehicle vehicle;
    mtx_t vehicle_mutex;
    bool accelerator_sensor_failure;
    bool shutdown_requested;
    FILE *log_file;
} AppContext;

/* Періодично виводить поточну швидкість у консоль.
 * arg - вказівник на AppContext зі спільним станом програми.
 */
static int heartbeat_thread(void* arg)
{
    /* Цей потік лише показує поточну швидкість раз на секунду. */
    AppContext* context = (AppContext*) arg;
    struct timespec interval = {
        .tv_sec = 1,
        .tv_nsec = 0
    };

    while (1)
    {
        double speed_m_per_s;

        /* Mutex потрібен, щоб не читати стан одночасно з його оновленням в іншому потоці. */
        mtx_lock(&context->vehicle_mutex);
        speed_m_per_s = Vehicle_get_speed(&context->vehicle);
        mtx_unlock(&context->vehicle_mutex);

        printf("Vehicle speed: %.2f m/s\n", speed_m_per_s);
        fflush(stdout);
        thrd_sleep(&interval, NULL);
    }

    return 0;
}

/* Виконує симуляцію електромобіля в окремому потоці.
 * arg — вказівник на AppContext зі спільним станом програми.
 */
static int vehicle_simulation_thread(void *arg)
{
    /* Основний цикл симуляції: кожні 20 мс робимо один крок розрахунку. */
    AppContext *context = (AppContext *) arg;

    struct timespec interval = {
        .tv_sec = 0,
        .tv_nsec = 20000000L
    };

    uint32_t log_counter = 0;

    while (!context->shutdown_requested)
    {
        mtx_lock(&context->vehicle_mutex);

        Vehicle_step_50hz(&context->vehicle);

        if ((log_counter++ % 5U) == 0U)
        {
            fprintf(context->log_file,
                    "%.3f,%.3f,%.3f\n",
                    context->vehicle.commanded_torque_nm,
                    Vehicle_get_speed(&context->vehicle),
                    context->vehicle.distance_m);
            fflush(context->log_file);
        }

        mtx_unlock(&context->vehicle_mutex);

        thrd_sleep(&interval, NULL);
    }

    if (context->log_file)
    {
        fclose(context->log_file);
        context->log_file = NULL;
    }

    return 0;
}

/* Перевіряє, що запит адресований рівно одному очікуваному регістру.
 * address - початкова адреса з Modbus-запиту.
 * quantity - кількість регістрів у запиті.
 * expected_address - адреса, яку ця функція дозволяє.
 */
static nmbs_error expect_single_register(uint16_t address, uint16_t quantity, uint16_t expected_address)
{
    /* Для простоти цей навчальний приклад дозволяє працювати лише з одним регістром за раз. */
    if (address != expected_address || quantity != 1)
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;

    return NMBS_ERROR_NONE;
}

/* Перетворює число double у 16-бітне ціле з заданим масштабом.
 * value - вихідне значення.
 * scale - множник для збереження дробової частини.
 */
static uint16_t encode_scaled(double value, double scale)
{
    /* Modbus-регістр має 16 біт, тому дробові величини передаємо як ціле число зі scale. */
    if (value <= 0.0)
        return 0;

    return (uint16_t) (value * scale + 0.5);
}

/* Перевіряє, чи не перевищує команда моменту можливості двигуна.
 * context - спільний стан з параметрами електромобіля.
 * torque_command - команда моменту в int16.
 */
static bool is_torque_command_valid(const AppContext* context, int16_t torque_command)
{
    const double max_torque = context->vehicle.params.motor_max_torque_nm;
    return torque_command >= -(int16_t) max_torque && torque_command <= (int16_t) max_torque;
}

/* Обробляє читання discrete inputs через Modbus.
 * address - адреса першого біта.
 * quantity - кількість бітів для читання.
 * inputs_out - буфер, куди записується результат.
 * unit_id - адреса пристрою Modbus, тут не використовується.
 * arg - вказівник на AppContext.
 */
static nmbs_error read_discrete_inputs(
    uint16_t address,
    uint16_t quantity,
    nmbs_bitfield inputs_out,
    uint8_t unit_id,
    void* arg)
{
    (void) unit_id;

    AppContext* context = (AppContext*) arg;
    nmbs_error err = expect_single_register(address, quantity, ACCELERATOR_SENSOR_FAILURE_ADDRESS);
    if (err != NMBS_ERROR_NONE)
        return err;

    /* У цьому прикладі discrete input 0 сигналізує про помилкову команду моменту. */
    memset(inputs_out, 0, 1);
    nmbs_bitfield_write(inputs_out, 0, context->accelerator_sensor_failure);
    return NMBS_ERROR_NONE;
}

/* Обробляє читання holding register з командою моменту.
 * address - адреса першого регістру.
 * quantity - кількість регістрів для читання.
 * registers_out - буфер, куди записується значення.
 * unit_id - адреса пристрою Modbus, тут не використовується.
 * arg - вказівник на AppContext.
 */
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

    /* Повертаємо останню прийняту команду моменту. */
    mtx_lock(&context->vehicle_mutex);
    registers_out[0] = (uint16_t) ((int16_t) context->vehicle.commanded_torque_nm);
    mtx_unlock(&context->vehicle_mutex);
    return NMBS_ERROR_NONE;
}

/* Обробляє читання input registers з параметрами електромобіля.
 * address - адреса першого регістру.
 * quantity - кількість регістрів для читання.
 * registers_out - буфер, куди записуються значення.
 * unit_id - адреса пристрою Modbus, тут не використовується.
 * arg - вказівник на AppContext.
 */
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

    /* Клієнт може читати параметри електромобіля як таблицю послідовних регістрів. */
    mtx_lock(&context->vehicle_mutex);

    for (i = 0; i < quantity; ++i)
    {
        switch (address + i)
        {
        case PARAM_MASS_ADDRESS:
            /* Для цілих значень scale = 1, для дробових зберігаємо значення помноженим. */
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

/* Обробляє запис одного holding register з командою моменту.
 * address - адреса регістру для запису.
 * value - нове значення регістру.
 * unit_id - адреса пристрою Modbus, тут не використовується.
 * arg - вказівник на AppContext.
 */
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
        /* Якщо команда виходить за межі, не застосовуємо її і підіймаємо прапорець помилки. */
        context->accelerator_sensor_failure = true;
        mtx_unlock(&context->vehicle_mutex);
        printf("Invalid torque command received: %d Nm\n", torque_command);
        return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
    }

    context->accelerator_sensor_failure = false;
    /* Після перевірки передаємо команду в модель електромобіля. */
    Vehicle_set_torque(&context->vehicle, (double) torque_command);
    mtx_unlock(&context->vehicle_mutex);

    printf("Torque command written: %d Nm\n", torque_command);
    return NMBS_ERROR_NONE;
}

/* Обробляє запис кількох регістрів; у цьому прикладі використовується один.
 * address - адреса першого регістру для запису.
 * quantity - кількість переданих регістрів.
 * registers - масив нових значень.
 * unit_id - адреса пристрою Modbus, тут не використовується.
 * arg - вказівник на AppContext.
 */
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
        context->accelerator_sensor_failure = true;
        mtx_unlock(&context->vehicle_mutex);
        printf("Invalid torque command received: %d Nm\n", torque_command);
        return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
    }

    context->accelerator_sensor_failure = false;
    Vehicle_set_torque(&context->vehicle, (double) torque_command);
    mtx_unlock(&context->vehicle_mutex);

    printf("Torque command written: %d Nm\n", torque_command);
    return NMBS_ERROR_NONE;
}

/* Налаштовує модель електромобіля, потоки і Modbus-сервер, а потім запускає програму. */
int main(void)
{
    AppContext app;
    /* Початкові параметри моделі електромобіля. */
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
    app.accelerator_sensor_failure = false;
    app.shutdown_requested = false;

    char filename[64];

    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);

    strftime(filename, sizeof(filename), "%Y-%m-%d_%H-%M-%S_vehicle_log.csv", tm_info);

    app.log_file = fopen(filename, "w");

    if (!app.log_file)
    {
        fprintf(stderr, "Failed to open log file\n");
        return 1;
    }

    fprintf(app.log_file, "torque,speed,distance\n");
    fflush(app.log_file);

    printf("Logging to file: %s\n", filename);

    /* Один mutex захищає увесь стан vehicle, бо він невеликий і так простіше. */
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
    /* Тут призначаємо функції-обробники для Modbus-команд (callbacks). */
    callbacks.read_discrete_inputs = read_discrete_inputs;
    callbacks.read_holding_registers = read_holding_registers;
    callbacks.read_input_registers = read_input_registers;
    callbacks.write_single_register = write_single_register;
    callbacks.write_multiple_registers = write_multiple_registers;
    callbacks.user_data = &app;

    return ModbusTcp_run(&config, &callbacks);
}
