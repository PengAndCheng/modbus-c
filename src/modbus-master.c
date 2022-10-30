#include "modbus-master.h"
#include "modbus-master-function.h"




ModbusError modbusRequestRTU(ModbusMaster *status, uint8_t slave_address, uint8_t function_code, uint16_t register_address, uint16_t count, uint16_t value0506, void* data1516){
    status->line_type = RTU;
    status->requestPDU = status->request + MODBUS_RTU_PDU_OFFSET;
    ModbusError err;
    if (function_code == 1 || function_code == 2 || function_code == 3 || function_code == 4) {
        err = modbusBuildRequest01020304(status,function_code,register_address,count);
    }else if (function_code == 5 || function_code == 6) {
        err = modbusBuildRequest0506(status,function_code,register_address,value0506);
    }else if (function_code == 15) {
        uint8_t* data15  = (uint8_t*)data1516;
        err = modbusBuildRequest15(status,register_address,count,data15);
    }else if (function_code == 16) {
        uint16_t* data16  = (uint16_t*)data1516;
        err = modbusBuildRequest16(status,register_address,count,data16);
    }else {
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_FUNCTION;
    }

    if (err) {
        MASTER_DEBUG_PRINTF;
        return err + MODBUS_FUNCTION_EXCEPTIONAL_BASE;
    }
    if (status->requestPDU_length) {
        status->request_length = status->requestPDU_length + MODBUS_RTU_ADU_PADDING;
        if (status->request_length < MODBUS_RTU_ADU_MIN || status->request_length > MODBUS_RTU_ADU_MAX)
        {
            MASTER_DEBUG_PRINTF;
            return MODBUS_ERROR_LENGTH;
        }
        status->request[0] = slave_address;
        modbusWriteLittleEndian(&status->request[status->request_length - 2], modbusCRC(status->request, status->request_length - 2));
        return MODBUS_OK;
    }else {
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_LENGTH;
    }
}

ModbusError modbusRequestTCP(ModbusMaster *status, uint16_t transactionID, uint8_t unitID, uint8_t function_code, uint16_t register_address, uint16_t count, uint16_t value0506, void* data1516){
    status->line_type = TCP;
    status->requestPDU = status->request + MODBUS_TCP_PDU_OFFSET;
    ModbusError err;
    if (function_code == 1 || function_code == 2 || function_code == 3 || function_code == 4) {
        err = modbusBuildRequest01020304(status,function_code,register_address,count);
    }else if (function_code == 5 || function_code == 6) {
        err = modbusBuildRequest0506(status,function_code,register_address,value0506);
    }else if (function_code == 15) {
        uint8_t* data15  = (uint8_t*)data1516;
        err = modbusBuildRequest15(status,register_address,count,data15);
    }else if (function_code == 16) {
        uint16_t* data16  = (uint16_t*)data1516;
        err = modbusBuildRequest16(status,register_address,count,data16);
    }else {
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_FUNCTION;
    }

    if (err) {
        MASTER_DEBUG_PRINTF;
        return err + MODBUS_FUNCTION_EXCEPTIONAL_BASE;
    }
    if (status->requestPDU_length) {
        status->request_length = status->requestPDU_length + MODBUS_TCP_ADU_PADDING;
        if (status->request_length < MODBUS_TCP_ADU_MIN || status->request_length > MODBUS_TCP_ADU_MAX){
            MASTER_DEBUG_PRINTF;
            return MODBUS_ERROR_LENGTH;}
        modbusWriteBigEndian(&status->request[0], transactionID);
        modbusWriteBigEndian(&status->request[2], 0);
        modbusWriteBigEndian(&status->request[4], status->request_length - 6);
        status->request[6] = unitID;
        return MODBUS_OK;
    }else {
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_LENGTH;
    }
}



int modbusExpectedResponseInputCount(ModbusLineType type, uint8_t function_code, uint16_t register_count){
    int rtu_input_count = 0;
    if (function_code == 1 || function_code == 2) {
        rtu_input_count = (register_count+7)/8+5;   //从机地址1+功能码1+数据长度1+数据(N+7)/8+校验2
    }else if (function_code == 3 || function_code == 4) {
        rtu_input_count = register_count*2+5;       //从机地址1+功能码1+数据长度1+数据N+校验2*N
    }else if (function_code == 5) {
        rtu_input_count = 8;                    //从机地址1+功能码1+寄存器地址2+数据2+校验2   与主机发送的报文格式及数据内容完全相同
    }else if (function_code == 6) {
        rtu_input_count = 8;                    //从机地址1+功能码1+寄存器地址2+数据2+校验2   与主机发送的报文格式及数据内容完全相同
    }else if (function_code == 15) {
        rtu_input_count = 9;                    //从机地址1+功能码1+寄存器起始地址2+寄存器数量2+字节数1+校验2
    }else if (function_code == 16) {
        rtu_input_count = 8;                    //从机地址1+功能码1+寄存器起始地址2+寄存器数量2+校验2
    }else {
        MASTER_DEBUG_PRINTF;
        return 0;
    }

    if (RTU == type) {
        return rtu_input_count;
    }else if (TCP == type) {
        return rtu_input_count + 4; //TCP帧头是7，1个id重叠，没有2个校验码  7-1-2=4
    }

    MASTER_DEBUG_PRINTF;
    return 0;
}

uint8_t modbusExpectedResponseInputExceptionCode(ModbusLineType type,uint8_t* response, int response_lenght){
    uint8_t function_code;

    if (type == RTU) {
        if (response_lenght >= MODBUS_RTU_ADU_MIN && response_lenght <= MODBUS_RTU_ADU_MAX) {
            function_code = response[MODBUS_RTU_PDU_OFFSET];
            if (function_code == 01+0x80 || function_code == 02+0x80 || function_code == 03+0x80 || function_code == 04+0x80) {
                //RTU检验CRC 因为未知帧长 且不会频繁进入此行 所以校验CRC不浪费MCU性能
                if (modbusCRC(response, response_lenght - 2) == modbusReadLittleEndian(response + response_lenght - 2)){
                    return function_code;
                }
            }
        }
    }else if (type == TCP) {
        if (response_lenght >= MODBUS_TCP_ADU_MIN && response_lenght <= MODBUS_TCP_ADU_MAX) {
            function_code = response[MODBUS_TCP_PDU_OFFSET];
            if (function_code == 01+0x80 || function_code == 02+0x80 || function_code == 03+0x80 || function_code == 04+0x80) {
                //TCP校验长度
                uint16_t messageLength = modbusReadBigEndian(response + MODBUS_TCP_MESSAGELENGTH_OFFSET);
                if (messageLength == response_lenght - 6){
                    return function_code;
                }
            }
        }
    }

    return 0;
}
