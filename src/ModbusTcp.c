#include "ModbusTcp.h"

#include <stdio.h>
#include <string.h>

#include "tinycsocket.h"

typedef struct
{
    TcsSocket socket;
} ModbusTransport;

static int fail_tcs(const char* step, TcsResult result)
{
    fprintf(stderr, "%s failed: %d\n", step, result);
    return 1;
}

static int fail_nmbs(const char* step, nmbs_error err)
{
    fprintf(stderr, "%s failed: %s (%d)\n", step, nmbs_strerror(err), err);
    return 1;
}

static int32_t modbus_transport_read(uint8_t* buf, uint16_t count, int32_t byte_timeout_ms, void* arg)
{
    ModbusTransport* transport = (ModbusTransport*) arg;
    size_t bytes_received = 0;

    if (byte_timeout_ms >= 0)
    {
        TcsResult timeout_result = tcs_opt_receive_timeout_set(transport->socket, byte_timeout_ms);
        if (timeout_result != TCS_SUCCESS)
            return -1;
    }

    TcsResult result = tcs_receive(
        transport->socket,
        buf,
        count,
        TCS_MSG_WAITALL,
        &bytes_received);

    if (result == TCS_SUCCESS)
        return (int32_t) bytes_received;

    if (result == TCS_ERROR_TIMED_OUT)
        return (int32_t) bytes_received;

    return -1;
}

static int32_t modbus_transport_write(const uint8_t* buf, uint16_t count, int32_t byte_timeout_ms, void* arg)
{
    (void) byte_timeout_ms;

    ModbusTransport* transport = (ModbusTransport*) arg;
    size_t bytes_sent = 0;
    TcsResult result = tcs_send(transport->socket, buf, count, TCS_MSG_SENDALL, &bytes_sent);
    if (result != TCS_SUCCESS)
        return -1;

    return (int32_t) bytes_sent;
}

static nmbs_error run_modbus_session(
    TcsSocket client_socket,
    const ModbusTcpConfig* config,
    const ModbusTcpCallbacks* callbacks)
{
    ModbusTransport transport = {
        .socket = client_socket
    };

    nmbs_platform_conf platform_conf;
    nmbs_platform_conf_create(&platform_conf);
    platform_conf.transport = NMBS_TRANSPORT_TCP;
    platform_conf.read = modbus_transport_read;
    platform_conf.write = modbus_transport_write;
    platform_conf.arg = &transport;

    nmbs_callbacks nmbs_callbacks_conf;
    nmbs_callbacks_create(&nmbs_callbacks_conf);
    nmbs_callbacks_conf.read_coils = callbacks->read_coils;
    nmbs_callbacks_conf.read_discrete_inputs = callbacks->read_discrete_inputs;
    nmbs_callbacks_conf.read_holding_registers = callbacks->read_holding_registers;
    nmbs_callbacks_conf.read_input_registers = callbacks->read_input_registers;
    nmbs_callbacks_conf.write_single_coil = callbacks->write_single_coil;
    nmbs_callbacks_conf.write_single_register = callbacks->write_single_register;
    nmbs_callbacks_conf.write_multiple_coils = callbacks->write_multiple_coils;
    nmbs_callbacks_conf.write_multiple_registers = callbacks->write_multiple_registers;

    nmbs_t nmbs;
    nmbs_error err = nmbs_server_create(&nmbs, config->unit_id, &platform_conf, &nmbs_callbacks_conf);
    if (err != NMBS_ERROR_NONE)
        return err;

    nmbs_set_callbacks_arg(&nmbs, callbacks->user_data);
    nmbs_set_read_timeout(&nmbs, config->read_timeout_ms);
    nmbs_set_byte_timeout(&nmbs, config->byte_timeout_ms);

    while (1)
    {
        err = nmbs_server_poll(&nmbs);
        if (err == NMBS_ERROR_NONE)
            continue;

        if (err == NMBS_ERROR_TRANSPORT)
        {
            printf("Client disconnected\n");
            return NMBS_ERROR_NONE;
        }

        return err;
    }
}

void ModbusTcp_config_init(ModbusTcpConfig* config)
{
    config->port = 5020;
    config->unit_id = 1;
    config->read_timeout_ms = 1000;
    config->byte_timeout_ms = 1000;
}

void ModbusTcp_callbacks_init(ModbusTcpCallbacks* callbacks)
{
    memset(callbacks, 0, sizeof(*callbacks));
}

int ModbusTcp_run(const ModbusTcpConfig* config, const ModbusTcpCallbacks* callbacks)
{
    TcsResult result = tcs_lib_init();
    if (result != TCS_SUCCESS)
        return fail_tcs("tcs_lib_init", result);

    TcsSocket listen_socket = TCS_SOCKET_INVALID;
    result = tcs_socket(&listen_socket, TCS_AF_IP4, TCS_SOCK_STREAM, TCS_PROTOCOL_IP_TCP);
    if (result != TCS_SUCCESS)
        goto cleanup;

    struct TcsAddress listen_address = TCS_ADDRESS_NONE;
    listen_address.family = TCS_AF_IP4;
    listen_address.data.ip4.address = TCS_ADDRESS_ANY_IP4;
    listen_address.data.ip4.port = config->port;

    result = tcs_bind(listen_socket, &listen_address);
    if (result != TCS_SUCCESS)
        goto cleanup;

    result = tcs_listen(listen_socket, TCS_BACKLOG_MAX);
    if (result != TCS_SUCCESS)
        goto cleanup;

    printf("Modbus TCP slave listening on 0.0.0.0:%u\n", config->port);

    while (1)
    {
        TcsSocket client_socket = TCS_SOCKET_INVALID;
        struct TcsAddress remote_address = TCS_ADDRESS_NONE;

        result = tcs_accept(listen_socket, &client_socket, &remote_address);
        if (result != TCS_SUCCESS)
            goto cleanup;

        char remote_string[70] = {0};
        if (tcs_address_to_str(&remote_address, remote_string) == TCS_SUCCESS)
            printf("Accepted Modbus client from %s\n", remote_string);

        nmbs_error err = run_modbus_session(client_socket, config, callbacks);

        TcsResult close_result = tcs_close(&client_socket);
        if (close_result != TCS_SUCCESS)
            return fail_tcs("tcs_close(client_socket)", close_result);

        if (err != NMBS_ERROR_NONE)
            return fail_nmbs("run_modbus_session", err);
    }

cleanup:
    if (listen_socket != TCS_SOCKET_INVALID)
        tcs_close(&listen_socket);

    tcs_lib_free();

    if (result != TCS_SUCCESS)
        return fail_tcs("server", result);

    return 0;
}
