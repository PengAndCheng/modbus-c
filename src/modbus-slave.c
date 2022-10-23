
#include "modbus-slave.h"
#include "modbus-function.h"



inline ModbusError modbusParseRequest(ModbusSlave *status, const uint8_t *requestPDU, uint8_t requestPDULength)
{
    status->function = requestPDU;

    //直行匹配功能函数
    if (*status->function == 1 || *status->function == 2 || *status->function == 3 || *status->function == 4) {
        return modbusParseRequest01020304(status, *status->function, requestPDU, requestPDULength);
    }else if (*status->function == 5 || *status->function == 6) {
        return modbusParseRequest0506(status, *status->function, requestPDU, requestPDULength);
    }else if (*status->function == 15 || *status->function == 16) {
        return modbusParseRequest1516(status, *status->function, requestPDU, requestPDULength);
    }

    //没有匹配功能函数
    return MODBUS_ERROR_FUNCTION;
}




ModbusError modbusParseRequestRTU(ModbusSlave *status, uint8_t slaveAddress, const uint8_t *request, uint16_t requestLength, uint8_t checkCRC){
    ModbusError err;

    //检查从机地址
    if (request[0] != BROADCAST_ADDRESS && request[0] != slaveAddress)
        return MODBUS_ERROR_ADDRESS;

    //检查长度
    if (requestLength < MODBUS_RTU_ADU_MIN || requestLength > MODBUS_RTU_ADU_MAX)
        return MODBUS_ERROR_LENGTH;

    //提取地址
    status->request_frame = (uint8_t *)request;
    status->request_frame_length = requestLength;
    status->id.rtu_slave_address = &request[0];
    status->requestPDU = request + MODBUS_RTU_PDU_OFFSET;
    status->requestPDU_length = requestLength - MODBUS_RTU_ADU_PADDING;
    status->rtu_crc = request + requestLength - 2;

    //检查CRC CRC小端
    if (checkCRC && modbusCRC(status->request_frame, status->request_frame_length - 2) != modbusReadLittleEndian(status->rtu_crc))
        return MODBUS_ERROR_CRC;

    //提供PDU指针 返回PDU长度
    status->responsePDU = status->response + MODBUS_RTU_PDU_OFFSET;
    err = modbusParseRequest(status, status->requestPDU, status->requestPDU_length);
    if (err) return err;

    //该长度为ADU长度 写地址和CRC
    if (status->responsePDU_length)
    {
        //得到响应长度
        status->response_length = status->responsePDU_length + MODBUS_RTU_ADU_PADDING;

        //广播地址 不回复
        if (*status->id.rtu_slave_address == BROADCAST_ADDRESS)
        {
            return MODBUS_ERROR_OK;
        }

        //增加非PDU数据
        //检查长度
        if (status->response_length < MODBUS_RTU_ADU_MIN || status->response_length > MODBUS_RTU_ADU_MAX)
            return MODBUS_ERROR_LENGTH;

        //填充地址
        status->response[0] = *status->id.rtu_slave_address;

        //填充CRC CRC小端
        modbusWriteLittleEndian(&status->response[status->response_length - 2], modbusCRC(status->response, status->response_length - 2));

        return MODBUS_OK;
    }else {
        return MODBUS_ERROR_LENGTH;
    }
}





ModbusError modbusParseRequestTCP(ModbusSlave *status, const uint8_t *request, uint16_t requestLength)
{
    ModbusError err;

    //检查长度
    if (requestLength < MODBUS_TCP_ADU_MIN || requestLength > MODBUS_TCP_ADU_MAX)
        return MODBUS_ERROR_LENGTH;

    //取地址
    status->request_frame = request;
    status->request_frame_length = requestLength;
    status->tcp_transactionID = request + MODBUS_TCP_TRANSACTIONID_OFFSET;
    status->tcp_protocolID = request + MODBUS_TCP_PROTOCOLID_OFFSET;
    status->tcp_messageLengthlength = request + MODBUS_TCP_MESSAGELENGTH_OFFSET;
    status->id.tcp_unitID = request + MODBUS_TCP_UNITID_OFFSET;

    status->requestPDU = request + MODBUS_TCP_PDU_OFFSET;
    status->requestPDU_length = requestLength - MODBUS_TCP_ADU_PADDING;

    //协议标识符 00 00 表示Modbus TCP协议 MODBUS通讯大端
    uint16_t protocolID = modbusReadBigEndian(status->tcp_protocolID);
    if (protocolID != 0)
        return MODBUS_ERROR_BAD_PROTOCOL;

    //检查帧内容长度
    uint16_t messageLength = modbusReadBigEndian(status->tcp_messageLengthlength);
    if (messageLength != status->request_frame_length - 6)
        return MODBUS_ERROR_LENGTH;

    uint16_t transactionID = modbusReadBigEndian(status->tcp_transactionID);

    status->responsePDU = status->response + MODBUS_TCP_PDU_OFFSET;
    err = modbusParseRequest(status, status->requestPDU, status->requestPDU_length);
    if (err) return err;

    //该长度为ADU长度 写MBAP头
    if (status->responsePDU_length)
    {
        status->response_length = status->responsePDU_length + MODBUS_TCP_ADU_PADDING;

        //检查长度
        if (status->response_length < MODBUS_TCP_ADU_MIN || status->response_length > MODBUS_TCP_ADU_MAX)
            return MODBUS_ERROR_LENGTH;

        modbusWriteBigEndian(status->response + MODBUS_TCP_TRANSACTIONID_OFFSET, transactionID);
        modbusWriteBigEndian(status->response + MODBUS_TCP_PROTOCOLID_OFFSET, 0);
        modbusWriteBigEndian(status->response + MODBUS_TCP_MESSAGELENGTH_OFFSET, status->response_length - 6);
        status->response[MODBUS_TCP_UNITID_OFFSET] = *status->id.tcp_unitID;
        return MODBUS_OK;
    }else {
        return MODBUS_ERROR_LENGTH;
    }
}
