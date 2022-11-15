
#ifndef MODBUS_MASTER_H
#define MODBUS_MASTER_H

#include "modbus-base.h"

#define MASTER_DEBUG 1
#if MASTER_DEBUG
#define MASTER_DEBUG_PRINTF  printf("modbus master error, %s, %d.\r\n",__FILE__ , __LINE__);
#else
#define MASTER_DEBUG_PRINTF
#endif

typedef struct ModbusMaster ModbusMaster;


typedef enum ModbusMasterResponseResult
{
    RESPONSE_OK,
    RESPONSE_EXCEPTION_CODE,
    RESPONSE_TIMEOUT,
} ModbusMasterResponseResult;

typedef enum ModbusMasterResponseResultCallbackResult
{
    CALLBACK_RESULT_EXEC_DATA_CALLBACK,
    CALLBACK_RESULT_SKIP_DATA_CALLBACK,
} ModbusMasterResponseResultCallbackResult;

typedef ModbusMasterResponseResultCallbackResult (*ModbusMasterResponseResultCallback)(
    const ModbusMaster *status,
    ModbusMasterResponseResult result,
    void* vp);



typedef struct ModbusDataCallbackArgs
{
    ModbusDataType type;
    uint16_t index;
    uint16_t value;
    uint8_t function;
    uint8_t address;
} ModbusDataCallbackArgs;

typedef ModbusError (*ModbusMasterDataCallback)(
    const ModbusMaster *status,
    const ModbusDataCallbackArgs *args);

union ID
{
    const uint8_t* tcp_unitID;
    const uint8_t* rtu_slave_address;
};

struct ModbusMaster
{
    ModbusLineType line_type;

    uint8_t* request;
    uint16_t request_buf_size;
    uint16_t request_length;

    uint8_t* tcp_transactionID;       //事务标识符
    uint8_t* tcp_protocolID;          //协议标识符号
    uint8_t* tcp_messageLengthlength;//协议标识符号

    union ID id;

    uint8_t* requestPDU;
    uint16_t requestPDU_length;

    uint8_t* request_rtu_crc;

    uint8_t* request_function;

    uint8_t* response;
    uint16_t response_buf_size;
    uint16_t response_length;

    uint8_t* responsePDU;
    uint16_t responsePDU_length;

    ModbusMasterDataCallback dataCallback;
    ModbusMasterResponseResultCallback resultCallback;
};


ModbusError modbusRequestRTU(ModbusMaster *status, uint8_t slave_address, uint8_t function_code, uint16_t register_address, uint16_t count, uint16_t value0506, void* data1516);
ModbusError modbusRequestTCP(ModbusMaster *status, uint16_t transactionID, uint8_t unitID, uint8_t function_code, uint16_t register_address, uint16_t count, uint16_t value0506, void* data1516);

int modbusExpectedResponseFrameLength(ModbusLineType type, uint8_t function_code, uint16_t register_count);
uint8_t modbusExpectedResponseExceptionCode(ModbusLineType type,uint8_t* response, int response_lenght);

#endif /* MODBUS_MASTER_H */
