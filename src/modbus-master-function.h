
#ifndef MODBUS_MASTER_FUNCTION_H
#define MODBUS_MASTER_FUNCTION_H



inline ModbusError modbusBuildRequest01020304(
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
            return MODBUS_ERROR_FUNCTION;
    }

    //检查读取的数量 按照协议是可以读取0个寄存器的 但是读0个没有意义 这里校验读0个不通过
    if (count == 0 || count > maxCount)
        return MODBUS_ERROR_COUNT;

    //检查读取的范围是否超限
    if (modbusCheckRangeU16(index, count))
        return MODBUS_ERROR_RANGE;

    status->requestPDU[0] = function;
    modbusWriteBigEndian(&status->requestPDU[1], index);
    modbusWriteBigEndian(&status->requestPDU[3], count);
    status->requestPDU_length = 5;

    return MODBUS_ERROR_OK;
}


inline ModbusError modbusBuildRequest0506(
    ModbusMaster *status,
    uint8_t function,
    uint16_t index,
    uint16_t value)
{
    if (function != 5 && function != 6)
        return MODBUS_ERROR_FUNCTION;

    //线圈真转换为1
    if (function == 5)
        value = value ? 0xff00 : 0;


    status->requestPDU[0] = function;
    modbusWriteBigEndian(&status->requestPDU[1], index);
    modbusWriteBigEndian(&status->requestPDU[3], value);
    status->requestPDU_length = 5;

    return MODBUS_ERROR_OK;
}


inline ModbusError modbusBuildRequest15(
    ModbusMaster *status,
    uint16_t index,
    uint16_t count,
    const uint8_t *values)
{
    //检查写的数量 按照协议是可以写0个寄存器的 但是写0个没有意义 这里校验读0个不通过
    if (count == 0 || count > MODBUS_WRITE_15_REGISTER_MAX)
        return MODBUS_ERROR_COUNT;

    //检查范围
    if (modbusCheckRangeU16(index, count))
        return MODBUS_ERROR_RANGE;

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


inline ModbusError modbusBuildRequest16(
    ModbusMaster *status,
    uint16_t index,
    uint16_t count,
    const uint16_t *values)
{
    //检查写的数量 按照协议是可以写0个寄存器的 但是写0个没有意义 这里校验读0个不通过
    if (count == 0 || count > MODBUS_WRITE_16_REGISTER_MAX)
        return MODBUS_ERROR_COUNT;

    //检查范围
    if (modbusCheckRangeU16(index, count))
        return MODBUS_ERROR_RANGE;

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



#endif /* MODBUS_MASTER_FUNCTION_H */
