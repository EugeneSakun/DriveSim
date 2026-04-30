#ifndef MODBUS_TCP_H
#define MODBUS_TCP_H

#include <stdbool.h>
#include <stdint.h>

#include "nanomodbus.h"

typedef struct
{
    /* Це таблиця функцій-зворотних викликів (callbacks).
     * Бібліотека Modbus викликає їх, коли клієнт читає або записує регістри.
     */
    nmbs_error (*read_coils)(uint16_t address, uint16_t quantity, nmbs_bitfield coils_out, uint8_t unit_id, void* arg);
    nmbs_error (*read_discrete_inputs)(uint16_t address, uint16_t quantity, nmbs_bitfield inputs_out, uint8_t unit_id, void* arg);
    nmbs_error (*read_holding_registers)(uint16_t address, uint16_t quantity, uint16_t* registers_out, uint8_t unit_id, void* arg);
    nmbs_error (*read_input_registers)(uint16_t address, uint16_t quantity, uint16_t* registers_out, uint8_t unit_id, void* arg);
    nmbs_error (*write_single_coil)(uint16_t address, bool value, uint8_t unit_id, void* arg);
    nmbs_error (*write_single_register)(uint16_t address, uint16_t value, uint8_t unit_id, void* arg);
    nmbs_error (*write_multiple_coils)(uint16_t address, uint16_t quantity, const nmbs_bitfield coils, uint8_t unit_id, void* arg);
    nmbs_error (*write_multiple_registers)(uint16_t address, uint16_t quantity, const uint16_t* registers, uint8_t unit_id, void* arg);
    void* user_data;
} ModbusTcpCallbacks;

typedef struct
{
    /* Параметри TCP-сервера Modbus. */
    uint16_t port;
    uint8_t unit_id;
    int32_t read_timeout_ms;
    int32_t byte_timeout_ms;
} ModbusTcpConfig;

/* Заповнює структуру конфігурації типовими значеннями.
 * config - структура, яку потрібно ініціалізувати.
 */
void ModbusTcp_config_init(ModbusTcpConfig* config);
/* Обнуляє таблицю callback-функцій перед налаштуванням.
 * callbacks - структура з функціями обробки Modbus-запитів.
 */
void ModbusTcp_callbacks_init(ModbusTcpCallbacks* callbacks);

/* Запускає Modbus TCP сервер і починає обробляти клієнтів.
 * config - параметри TCP-сервера.
 * callbacks - функції, які обробляють запити на читання і запис.
 */
int ModbusTcp_run(const ModbusTcpConfig* config, const ModbusTcpCallbacks* callbacks);

#endif // MODBUS_TCP_H
