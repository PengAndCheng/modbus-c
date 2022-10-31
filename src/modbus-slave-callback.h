
#ifndef MODBUS_SLAVE_CALLBACK_H
#define MODBUS_SLAVE_CALLBACK_H

#define PRINTF printf
//#define PRINTF

uint8_t R01[32]={0};
uint8_t R02[32]={0};
uint16_t R03[32]={0};
uint16_t R04[32]={0};

static int isInit = 0;

void init(void){
    for (int i = 0; i < 32; ++i) {
        R01[i]=i;
        R02[i]=i;
        R03[i]=i;
        R04[i]=i;
    }
}


static inline ModbusError defaultSlaveRegisterCallback(
    const ModbusSlave *slave,
    const ModbusRegisterCallbackArgs *args,
    ModbusRegisterCallbackResult *result)
{
    PRINTF(
        "Register query:\n query: %d\n type: %d\n id: %d\n value: %d\n fun: %d\n\n",
        args->query,
        args->type,
        args->index,
        args->value,
        args->function
    );

    if (!isInit) {
        init();
    }

    switch (args->query)
    {
        case MODBUS_REGQ_R_CHECK:
        case MODBUS_REGQ_W_CHECK:
            result->exceptionCode = MODBUS_OK;
            break;


        case MODBUS_REGQ_R:
            if (args->index >= 32) {
                result->value = 7;
                return MODBUS_OK;
            }

            break;
        case MODBUS_REGQ_W:
            if (args->index >= 32) {
                return MODBUS_OK;
            }

            break;

        default: break;
    }

    if (args->function == 01 || args->function == 05 || args->function == 15) {
        switch (args->query) {
            case MODBUS_REGQ_R:
                result->value = R01[args->index];
                break;
            case MODBUS_REGQ_W:
                R01[args->index] = args->value;
                break;

            default: break;
        }
    }else if (args->function == 02) {
        switch (args->query) {
            case MODBUS_REGQ_R:
                result->value = R02[args->index];
                break;
            case MODBUS_REGQ_W:
                R02[args->index] = args->value;
                break;

            default: break;
        }
    }else if (args->function == 03 || args->function == 06 || args->function == 16) {
        switch (args->query) {
            case MODBUS_REGQ_R:
                result->value = R03[args->index];
                break;
            case MODBUS_REGQ_W:
                R03[args->index] = args->value;
                break;

            default: break;
        }
    }else if (args->function == 04) {
        switch (args->query) {
            case MODBUS_REGQ_R:
                result->value = R04[args->index];
                break;
            case MODBUS_REGQ_W:
                R04[args->index] = args->value;
                break;

            default: break;
        }
    }



    return MODBUS_OK;
}



#endif /* MODBUS_SLAVE_CALLBACK_H */
