
#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include "modbus-base.h"


#define SLAVE_DEBUG 1
#if SLAVE_DEBUG
#define SLAVE_DEBUG_PRINTF  printf("modbus slave error, %s, %d.\r\n",__FILE__ , __LINE__);
#else
#define SLAVE_DEBUG_PRINTF
#endif


typedef struct ModbusSlave ModbusSlave;

typedef enum ModbusSlaveRequestResult
{
    REQUEST_PASS,
} ModbusSlaveRequestResult;

typedef enum ModbusSlaveRequestResultCallbackResult
{
    CALLBACK_RESULT_EXEC_REGISTER_CALLBACK,
    CALLBACK_RESULT_SKIP_REGISTER_CALLBACK,
} ModbusSlaveRequestResultCallbackResult;

typedef ModbusSlaveRequestResultCallbackResult (*ModbusSlaveRequestResultCallback)(
    const ModbusSlave *status,
    ModbusSlaveRequestResult result,
    void* vp);

typedef enum ModbusRegisterQuery
{
    MODBUS_REGQ_R_CHECK,
    MODBUS_REGQ_W_CHECK,
    MODBUS_REGQ_R,
    MODBUS_REGQ_W
} ModbusRegisterQuery;

typedef struct ModbusRegisterCallbackArgs
{
    ModbusDataType type;
    ModbusRegisterQuery query;
    uint16_t index;
    uint16_t value;
    uint8_t function;
} ModbusRegisterCallbackArgs;

typedef struct ModbusRegisterCallbackResult
{
    ModbusError exceptionCode;
    uint16_t value;
} ModbusRegisterCallbackResult;


typedef ModbusError (*ModbusSlaveRegisterCallback)(
    const ModbusSlave *status,
    const ModbusRegisterCallbackArgs *args,
    ModbusRegisterCallbackResult *out);


union ID
{
    const uint8_t* tcp_unitID;
    const uint8_t* rtu_slave_address;
};

struct ModbusSlave
{
    ModbusLineType line_type;

    const uint8_t* request_frame;
    uint16_t request_frame_length;//tcp ADU > 255


    const uint8_t* tcp_transactionID;//事务标识符
    const uint8_t* tcp_protocolID;//协议标识符号
    const uint8_t* tcp_messageLengthlength;//协议标识符号

    union ID id;

    const uint8_t* requestPDU;
    uint16_t requestPDU_length;

    const uint8_t* rtu_crc;

    const uint8_t* function;//异常码需要提取到此参数


    uint8_t* response;
    uint16_t response_buf_size;
    uint16_t response_length;

    uint8_t* responsePDU;
    uint16_t responsePDU_length;

    ModbusSlaveRegisterCallback slaveRegisterCallback;
    ModbusSlaveRequestResultCallback resultCallback;
};


ModbusError modbusParseRequestRTU(ModbusSlave *status, uint8_t slaveAddress, const uint8_t *request, uint16_t requestLength, uint8_t checkCRC);

ModbusError modbusParseRequestTCP(ModbusSlave *status, const uint8_t *request, uint16_t requestLength);

void modbusSlaveInit(ModbusSlave *status,uint8_t* response_buf, uint16_t response_buf_size,ModbusSlaveRegisterCallback callback);

#endif /* MODBUS_SLAVE_H */
