
#ifndef MODBUS_MASTER_FUNCTION_H
#define MODBUS_MASTER_FUNCTION_H

#include "modbus-master-callback.h"

static inline ModbusError modbusBuildRequest01020304(
    ModbusMaster *status,
    uint8_t function,
    uint16_t index,
    uint16_t count)
{
    uint16_t maxCount;
    switch (function)
    {
        case 1:
        case 2:
            maxCount = MODBUS_READ_0102_REGISTER_MAX;
            break;

        case 3:
        case 4:
            maxCount = MODBUS_READ_0304_REGISTER_MAX;
            break;

        default:
            MASTER_DEBUG_PRINTF;
            return MODBUS_ERROR_FUNCTION;
    }

    //检查读取的数量 按照协议是可以读取0个寄存器的 但是读0个没有意义 这里校验读0个不通过
    if (count == 0 || count > maxCount){
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_COUNT;
    }

    //检查读取的范围是否超限
    if (modbusCheckRangeU16(index, count)){
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_RANGE;
    }


    status->requestPDU[0] = function;
    modbusWriteBigEndian(&status->requestPDU[1], index);
    modbusWriteBigEndian(&status->requestPDU[3], count);
    status->requestPDU_length = 5;

    return MODBUS_ERROR_OK;
}


static inline ModbusError modbusBuildRequest0506(
    ModbusMaster *status,
    uint8_t function,
    uint16_t index,
    uint16_t value)
{
    if (function != 5 && function != 6){
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_FUNCTION;
    }

    //线圈真转换为1
    if (function == 5)
        value = value ? 0xff00 : 0;


    status->requestPDU[0] = function;
    modbusWriteBigEndian(&status->requestPDU[1], index);
    modbusWriteBigEndian(&status->requestPDU[3], value);
    status->requestPDU_length = 5;

    return MODBUS_ERROR_OK;
}


static inline ModbusError modbusBuildRequest15(
    ModbusMaster *status,
    uint16_t index,
    uint16_t count,
    const uint8_t *values)
{
    //检查写的数量 按照协议是可以写0个寄存器的 但是写0个没有意义 这里校验读0个不通过
    if (count == 0 || count > MODBUS_WRITE_15_REGISTER_MAX){
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_COUNT;
    }

    //检查范围
    if (modbusCheckRangeU16(index, count)){
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_RANGE;
    }

    uint8_t dataLength = modbusBitsToBytes(count);

    //n是满位字节数量，r是最后一个字节使用的位数
    uint8_t n = count >> 3;
    uint8_t r = count & 7;

    //复制满字节
    for (uint8_t i = 0; i < n; i++)
        status->requestPDU[6 + i] = values[i];

    //复制剩余的位数
    if (r)
    {
        status->requestPDU[6 + n] = 0;
        for (uint8_t i = 0; i < r; i++)
            modbusWriteBits(
                &status->requestPDU[6 + n],
                i,
                modbusReadBits(values + n, i));
    }

    status->requestPDU[0] = 15;
    modbusWriteBigEndian(&status->requestPDU[1], index);
    modbusWriteBigEndian(&status->requestPDU[3], count);
    status->requestPDU[5] = dataLength;
    status->requestPDU_length = 6 + dataLength;

    return MODBUS_ERROR_OK;
}


static inline ModbusError modbusBuildRequest16(
    ModbusMaster *status,
    uint16_t index,
    uint16_t count,
    const uint16_t *values)
{
    //检查写的数量 按照协议是可以写0个寄存器的 但是写0个没有意义 这里校验读0个不通过
    if (count == 0 || count > MODBUS_WRITE_16_REGISTER_MAX){
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_COUNT;
    }

    //检查范围
    if (modbusCheckRangeU16(index, count)){
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_RANGE;
    }

    uint8_t dataLength = count << 1;

    //复制数据到帧
    for (uint8_t i = 0; i < (uint8_t)count; i++)
        modbusWriteBigEndian(&status->requestPDU[6 + (i << 1)], values[i]);

    status->requestPDU[0] = 16;
    modbusWriteBigEndian(&status->requestPDU[1], index);
    modbusWriteBigEndian(&status->requestPDU[3], count);
    status->requestPDU[5] = dataLength;
    status->requestPDU_length = dataLength + 6;
    return MODBUS_ERROR_OK;
}





static inline ModbusError modbusParseResponse01020304(
    ModbusMaster *status,
    uint8_t address,
    uint8_t function,
    const uint8_t *requestPDU,
    uint8_t requestPDULength,
    const uint8_t *responsePDU,
    uint8_t responsePDULength)
{
    //检查PDU长度
    if (requestPDULength != 5) {
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_LENGTH;
    }
    if (responsePDULength < 3){
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_LENGTH;
    }

    //判断数据类型
    uint8_t bits;
    uint16_t maxCount;
    ModbusDataType datatype;
    switch (function)
    {
        case 1:
            datatype = MODBUS_COIL;
            maxCount = MODBUS_READ_0102_REGISTER_MAX;
            bits = 1;
            break;

        case 2:
            datatype = MODBUS_DISCRETE_INPUT;
            maxCount = MODBUS_READ_0102_REGISTER_MAX;
            bits = 1;
            break;

        case 3:
            datatype = MODBUS_HOLDING_REGISTER;
            maxCount = MODBUS_READ_0304_REGISTER_MAX;
            bits = 16;
            break;

        case 4:
            datatype = MODBUS_INPUT_REGISTER;
            maxCount = MODBUS_READ_0304_REGISTER_MAX;
            bits = 16;
            break;

        default:
            MASTER_DEBUG_PRINTF;
            return MODBUS_ERROR_FUNCTION;
    }

    uint16_t index = modbusReadBigEndian(&requestPDU[1]);
    uint16_t count = modbusReadBigEndian(&requestPDU[3]);

    //检查寄存器数量范围
    if (count == 0 || count > maxCount){
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_COUNT;
    }


    //检查寄存器超限范围
    if (modbusCheckRangeU16(index, count)){
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_RANGE;
    }

    //应该接收到的字节数
    uint8_t expected = (bits == 16) ? (count << 1) : modbusBitsToBytes(count);

    //检查字节数匹配 expected + 2是因为包含功能码的表示长度的字节
    if (responsePDU[1] != expected || responsePDULength != expected + 2){
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_LENGTH;
    }

    //准备回调参数
    ModbusDataCallbackArgs cargs = {
        .type = datatype,
        .index = 0,
        .value = 0,
        .function = function,
        .address = address,
    };

    //数据回调
    if (status->resultCallback) {
        if (status->resultCallback(status,RESPONSE_OK,0) == CALLBACK_RESULT_SKIP_DATA_CALLBACK) {
            return MODBUS_ERROR_OK;
        }
    }
    for (uint16_t i = 0; i < count; i++)
    {
        cargs.index = index + i;
        if (bits == 1)
            cargs.value = modbusReadBits(&responsePDU[2], i);
        else
            cargs.value = modbusReadBigEndian(&responsePDU[2 + (i << 1)]);

        if (status->dataCallback) {
            status->dataCallback(status, &cargs);
        }else {
            defaultMasterdataCallback(status, &cargs);
        }
    }

    return MODBUS_ERROR_OK;
}


static inline ModbusError modbusParseResponse0506(
    ModbusMaster *status,
    uint8_t address,
    uint8_t function,
    const uint8_t *requestPDU,
    uint8_t requestPDULength,
    const uint8_t *responsePDU,
    uint8_t responsePDULength)
{
    //检查PDU长度 0506功能码的发送帧和接收帧是一样的
    if (requestPDULength != 5) {
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_LENGTH;
    };
    if (responsePDULength != 5) {
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_LENGTH;
    };

    //校验寄存器地址
    if (modbusReadBigEndian(&requestPDU[1]) != modbusReadBigEndian(&responsePDU[1])){
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_INDEX;
    }


    //校验值
    if (modbusReadBigEndian(&requestPDU[3]) != modbusReadBigEndian(&responsePDU[3])){
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_VALUE;
    }

    //结果回调
    if (status->resultCallback) {
        status->resultCallback(status,RESPONSE_OK,0);
    }
    return MODBUS_ERROR_OK;
}


static inline ModbusError modbusParseResponse1516(
    ModbusMaster *status,
    uint8_t address,
    uint8_t function,
    const uint8_t *requestPDU,
    uint8_t requestPDULength,
    const uint8_t *responsePDU,
    uint8_t responsePDULength)
{
    //检查长度 1516请求和响应帧长度不一致
    if (requestPDULength < 7) {
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_LENGTH;
    }
    if (responsePDULength != 5) {
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_LENGTH;
    }

    uint16_t index = modbusReadBigEndian(&requestPDU[1]);
    uint16_t count = modbusReadBigEndian(&requestPDU[3]);

    //检查寄存器地址是否一样
    if (index != modbusReadBigEndian(&responsePDU[1])){
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_INDEX;
    }

    //检查寄存器数量是否一样
    if (count != modbusReadBigEndian(&responsePDU[3])){
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_COUNT;
    }

    //校验写范围
    uint16_t maxCount = (function == 15) ? MODBUS_WRITE_15_REGISTER_MAX : MODBUS_WRITE_16_REGISTER_MAX;
    if (count == 0 || count > maxCount){
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_COUNT;
    }

    //校验预期字节数
    uint16_t expected = function == 15 ? modbusBitsToBytes(count) : (count << 1);
    if (requestPDULength != expected + 6){
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_LENGTH;
    }

    //检验范围
    if (modbusCheckRangeU16(index, count)){
        MASTER_DEBUG_PRINTF;
        return MODBUS_ERROR_RANGE;
    }

    //结果回调
    if (status->resultCallback) {
        status->resultCallback(status,RESPONSE_OK,0);
    }
    return MODBUS_ERROR_OK;
}




#endif /* MODBUS_MASTER_FUNCTION_H */
