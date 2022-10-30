
#ifndef MODBUS_MASTER_CALLBACK_H
#define MODBUS_MASTER_CALLBACK_H

#define PRINTF printf



static inline ModbusError defaultMasterdataCallback(const ModbusMaster *master, const ModbusDataCallbackArgs *args)
{
    PRINTF(
        "Received data:\n"
        "\t from: %d\n"
        "\t  fun: %d\n"
        "\t type: %d\n"
        "\t   id: %d\n"
        "\tvalue: %d\n",
        args->address,
        args->function,
        args->type,
        args->index,
        args->value
    );



    return MODBUS_OK;
}

#endif /* MODBUS_MASTER_CALLBACK_H */
