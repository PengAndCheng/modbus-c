
#ifndef MODBUS_SLAVE_FUNCTION_H
#define MODBUS_SLAVE_FUNCTION_H

#include "modbus-slave-callback.h"

#define MODBUS_SLAVE_CHECK_CALLBACK     1




static inline ModbusError modbusParseRequest01020304(
    ModbusSlave *status,
    uint8_t function,
    const uint8_t *requestPDU,
    uint8_t requestPDULength)
{
    //功能内检查参数值 5就是PDU请求长度
    if (requestPDULength != 5){
        SLAVE_DEBUG_PRINTF;
        return MODBUS_ERROR_LENGTH;
    }

    ModbusDataType datatype;
    uint16_t maxCount;
    uint8_t isCoilType;
    switch (function)
    {
        case 1:
            datatype = MODBUS_COIL;
            maxCount = MODBUS_READ_0102_REGISTER_MAX;
            isCoilType = 1;
            break;

        case 2:
            datatype = MODBUS_DISCRETE_INPUT;
            maxCount = MODBUS_READ_0102_REGISTER_MAX;
            isCoilType = 1;
            break;

        case 3:
            datatype = MODBUS_HOLDING_REGISTER;
            maxCount = MODBUS_READ_0304_REGISTER_MAX;
            isCoilType = 0;
            break;

        case 4:
            datatype = MODBUS_INPUT_REGISTER;
            maxCount = MODBUS_READ_0304_REGISTER_MAX;
            isCoilType = 0;
            break;

        default:
            SLAVE_DEBUG_PRINTF;
            return MODBUS_ERROR_FUNCTION;
            break;
    }

    uint16_t index = modbusReadBigEndian(&requestPDU[1]);
    uint16_t count = modbusReadBigEndian(&requestPDU[3]);

    //检查寄存器数量 好像0也是合法要返回的 需要校验
    if (count == 0 || count > maxCount){
        SLAVE_DEBUG_PRINTF;
        return MODBUS_ERROR_COUNT;
    }

    //寄存器地址超过65535
    if (modbusCheckRangeU16(index, count)){
        SLAVE_DEBUG_PRINTF;
        return MODBUS_ERROR_RANGE;
    }

    //准备回调参数
    ModbusRegisterCallbackResult cres;
    ModbusRegisterCallbackArgs cargs = {
        .type = datatype,
        .query = MODBUS_REGQ_R_CHECK,
        .index = 0,
        .value = 0,
        .function = function,
    };

    //检查所有寄存器是否可以被读取
#if MODBUS_SLAVE_CHECK_CALLBACK
    for (uint16_t i = 0; i < count; i++)
    {
        cargs.index = index + i;
        ModbusError fail;
        if (status->slaveRegisterCallback) {
            fail = status->slaveRegisterCallback(status, &cargs, &cres);
        }else {
            fail = defaultSlaveRegisterCallback(status, &cargs, &cres);
        }
        if (fail) {
            SLAVE_DEBUG_PRINTF;
            return fail;
        }
        if (cres.exceptionCode) {
            SLAVE_DEBUG_PRINTF;
            return cres.exceptionCode;
        }
    }
#endif /* #if MODBUS_SLAVE_CHECK_CALLBACK */

    //进行响应构造
    uint8_t dataLength = (isCoilType ? modbusBitsToBytes(count) : (count << 1));
    //功能码+字节数+PDU长度等于数据长度
    status->responsePDU_length = 2 + dataLength;
    status->responsePDU[0] = function;
    status->responsePDU[1] = dataLength;

    //初始化数据区
    for (uint8_t i = 0; i < dataLength; i++)
        status->responsePDU[2 + i] = 0;

    cargs.query = MODBUS_REGQ_R;
    for (uint16_t i = 0; i < count; i++)
    {
        cargs.index = index + i;
        if (status->slaveRegisterCallback) {
            status->slaveRegisterCallback(status, &cargs, &cres);
        }else {
            defaultSlaveRegisterCallback(status, &cargs, &cres);
        }

        if (isCoilType)
            modbusWriteBits(&status->responsePDU[2], i, cres.value != 0);
        else
            modbusWriteBigEndian(&status->responsePDU[2 + (i << 1)], cres.value);
    }

    return MODBUS_ERROR_OK;
}






static inline ModbusError modbusParseRequest0506(
    ModbusSlave *status,
    uint8_t function,
    const uint8_t *requestPDU,
    uint8_t requestPDULength)
{
    //检查PDU长度
    if (requestPDULength != 5){
        SLAVE_DEBUG_PRINTF;
        return MODBUS_ERROR_LENGTH;
    }

    ModbusDataType datatype = function == 5 ? MODBUS_COIL : MODBUS_HOLDING_REGISTER;
    uint16_t index = modbusReadBigEndian(&requestPDU[1]);
    uint16_t value = modbusReadBigEndian(&requestPDU[3]);

    //线圈的1值是0xFF00. 65280
    if (datatype == MODBUS_COIL && value != 0x0000 && value != 0xFF00){
        SLAVE_DEBUG_PRINTF;
        return MODBUS_ERROR_VALUE;
    }

    //准备毁掉参数
    ModbusRegisterCallbackResult cres;
    ModbusRegisterCallbackArgs cargs = {
        .type = datatype,
        .query = MODBUS_REGQ_W_CHECK,
        .index = index,
        .value = (uint16_t)((datatype == MODBUS_COIL) ? (value != 0) : value),
        .function = function,
    };

    //检查回调
#if MODBUS_SLAVE_CHECK_CALLBACK
    ModbusError fail;
    if (status->slaveRegisterCallback) {
        fail = status->slaveRegisterCallback(status, &cargs, &cres);
    }else {
        fail = defaultSlaveRegisterCallback(status, &cargs, &cres);
    }
    if (fail) {
        SLAVE_DEBUG_PRINTF;
        return fail;
    }
    if (cres.exceptionCode) {
        SLAVE_DEBUG_PRINTF;
        return cres.exceptionCode;
    }
#endif /* #if MODBUS_SLAVE_CHECK_CALLBACK */

    //写数据
    cargs.query = MODBUS_REGQ_W;
    if (status->slaveRegisterCallback) {
        status->slaveRegisterCallback(status, &cargs, &cres);
    }else {
        defaultSlaveRegisterCallback(status, &cargs, &cres);
    }

    //创建响应 0506功能码的响应帧和请求帧一样
    status->responsePDU_length = 5;

    status->responsePDU[0] = function;
    modbusWriteBigEndian(&status->responsePDU[1], index);
    modbusWriteBigEndian(&status->responsePDU[3], value);

    return MODBUS_ERROR_OK;
}



static inline ModbusError modbusParseRequest1516(
    ModbusSlave *status,
    uint8_t function,
    const uint8_t *requestPDU,
    uint8_t requestPDULength)
{
    //检查长度 1516功能码的长度是动态的
    if (requestPDULength < 6) {
        SLAVE_DEBUG_PRINTF;
        return MODBUS_ERROR_LENGTH;
    }

    //获取寄存器地址和寄存器个数
    ModbusDataType datatype = function == 15 ? MODBUS_COIL : MODBUS_HOLDING_REGISTER;
    uint16_t maxCount = datatype == MODBUS_COIL ? MODBUS_WRITE_15_REGISTER_MAX : MODBUS_WRITE_16_REGISTER_MAX;
    uint16_t index = modbusReadBigEndian(&requestPDU[1]);
    uint16_t count = modbusReadBigEndian(&requestPDU[3]);
    uint8_t declaredLength = requestPDU[5];

    //检查声明的长度是否正确
    if (declaredLength == 0 || declaredLength != requestPDULength - 6){
        SLAVE_DEBUG_PRINTF;
        return MODBUS_ERROR_LENGTH;
    }

    //检查数量
    if (count == 0
        || count > maxCount
        || declaredLength != (datatype == MODBUS_COIL ? modbusBitsToBytes(count) : (count << 1))){
        SLAVE_DEBUG_PRINTF;
        return MODBUS_ERROR_COUNT;
    }

    //地址范围检查
    if (modbusCheckRangeU16(index, count)){
        SLAVE_DEBUG_PRINTF;
        return MODBUS_ERROR_RANGE;
    }

    // Prepare callback args
    ModbusRegisterCallbackResult cres;
    ModbusRegisterCallbackArgs cargs = {
        .type = datatype,
        .query = MODBUS_REGQ_W_CHECK,
        .index = 0,
        .value = 0,
        .function = function,
    };

    //检查写入访问
#if MODBUS_SLAVE_CHECK_CALLBACK
    for (uint16_t i = 0; i < count; i++)
    {
        cargs.index = index + i;
        cargs.value = datatype == MODBUS_COIL ? modbusReadBits(&requestPDU[6], i) : modbusReadLittleEndian(&requestPDU[6 + (i << 1)]);
        ModbusError fail;
        if (status->slaveRegisterCallback) {
            fail = status->slaveRegisterCallback(status, &cargs, &cres);
        }else {
            fail = defaultSlaveRegisterCallback(status, &cargs, &cres);
        }
        if (fail) {
            SLAVE_DEBUG_PRINTF;
            return fail;
        }
        if (cres.exceptionCode) {
            SLAVE_DEBUG_PRINTF;
            return cres.exceptionCode;
        }
    }
#endif /* #if MODBUS_SLAVE_CHECK_CALLBACK */

    //写
    cargs.query = MODBUS_REGQ_W;
    for (uint16_t i = 0; i < count; i++)
    {
        cargs.index = index + i;
        cargs.value = datatype == MODBUS_COIL ? modbusReadBits(&requestPDU[6], i) : modbusReadLittleEndian(&requestPDU[6 + (i << 1)]);
        if (status->slaveRegisterCallback) {
            status->slaveRegisterCallback(status, &cargs, &cres);
        }else {
            defaultSlaveRegisterCallback(status, &cargs, &cres);
        }
    }

    //构造响应
    status->responsePDU_length = 5;
    status->responsePDU[0] = function;
    modbusWriteBigEndian(&status->responsePDU[1], index);
    modbusWriteBigEndian(&status->responsePDU[3], count);

    return MODBUS_ERROR_OK;
}






#endif /* MODBUS_SLAVE_FUNCTION_H */
