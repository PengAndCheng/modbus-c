
#ifndef MODBUS_SLAVE_CALLBACK_H
#define MODBUS_SLAVE_CALLBACK_H

//#define PRINTF printf
#define PRINTF

static inline ModbusError defaultSlaveRegisterCallback(
    const ModbusSlave *slave,
    const ModbusRegisterCallbackArgs *args,
    ModbusRegisterCallbackResult *result)
{
    PRINTF(
        "Register query:\n"
        "\tquery: %d\n"
        "\t type: %d\n"
        "\t   id: %d\n"
        "\tvalue: %d\n"
        "\t  fun: %d\n",
        args->query,
        args->type,
        args->index,
        args->value,
        args->function
    );

    switch (args->query)
    {
        case MODBUS_REGQ_R_CHECK:
        case MODBUS_REGQ_W_CHECK:
            result->exceptionCode = MODBUS_OK;
            break;


        case MODBUS_REGQ_R:
            result->value = 0;
            break;

        default: break;
    }

    return MODBUS_OK;
}



#endif /* MODBUS_SLAVE_CALLBACK_H */
