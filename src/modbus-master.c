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
        return MODBUS_ERROR_FUNCTION;
    }

    if (err) return err;
    if (status->requestPDU_length) {
        status->request_length = status->requestPDU_length + MODBUS_RTU_ADU_PADDING;
        if (status->request_length < MODBUS_RTU_ADU_MIN || status->request_length > MODBUS_RTU_ADU_MAX)
            return MODBUS_ERROR_LENGTH;
        status->request[0] = slave_address;
        modbusWriteLittleEndian(&status->request[status->request_length - 2], modbusCRC(status->request, status->request_length - 2));
        return MODBUS_OK;
    }else {
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
        return MODBUS_ERROR_FUNCTION;
    }

    if (err) return err;
    if (status->requestPDU_length) {
        status->request_length = status->requestPDU_length + MODBUS_TCP_ADU_PADDING;
        if (status->request_length < MODBUS_TCP_ADU_MIN || status->request_length > MODBUS_TCP_ADU_MAX)
            return MODBUS_ERROR_LENGTH;
        modbusWriteBigEndian(&status->request[0], transactionID);
        modbusWriteBigEndian(&status->request[2], 0);
        modbusWriteBigEndian(&status->request[4], status->request_length - 6);
        status->request[6] = unitID;
        return MODBUS_OK;
    }else {
        return MODBUS_ERROR_LENGTH;
    }
}
