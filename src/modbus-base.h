
#ifndef MODBUS_BASE_H
#define MODBUS_BASE_H


#include <stdint.h>
#include <stdio.h>




#define BROADCAST_ADDRESS   0

#define MODBUS_PDU_MIN      1
#define MODBUS_PDU_MAX      253

#define MODBUS_RTU_ADU_MIN     4
#define MODBUS_RTU_ADU_MAX     256  //协议里限定的 固定值
#define MODBUS_RTU_ADU_PADDING 3    //从机地址+CRC 非PDU数据
#define MODBUS_RTU_PDU_OFFSET  1    //前面从机地址不算

#define MODBUS_TCP_ADU_MIN     8
#define MODBUS_TCP_ADU_MAX     260
#define MODBUS_TCP_ADU_PADDING 7    //非PDU数据
#define MODBUS_TCP_PDU_OFFSET  7


#define MODBUS_TCP_TRANSACTIONID_OFFSET  0
#define MODBUS_TCP_PROTOCOLID_OFFSET     2
#define MODBUS_TCP_MESSAGELENGTH_OFFSET  4
#define MODBUS_TCP_UNITID_OFFSET         6

#define MODBUS_READ_0102_REGISTER_MAX    2000   //一次读线圈最大个数
#define MODBUS_READ_0304_REGISTER_MAX    125    //一次读寄存器最大个数
#define MODBUS_WRITE_15_REGISTER_MAX     1968
#define MODBUS_WRITE_16_REGISTER_MAX     123



typedef enum ModbusDataType
{
    MODBUS_HOLDING_REGISTER = 1, //保持寄存器
    MODBUS_INPUT_REGISTER = 2,   //输入寄存器
    MODBUS_COIL = 4,             //线圈
    MODBUS_DISCRETE_INPUT = 8    //离散线圈
} ModbusDataType;

typedef enum ModbusLineType
{
    RTU,
    TCP,
} ModbusLineType;


typedef enum ModbusError
{

    MODBUS_OK = 0,
    MODBUS_ERROR_OK = 0,


    MODBUS_ERROR_LENGTH,

    MODBUS_ERROR_FUNCTION,

    MODBUS_ERROR_COUNT,

    MODBUS_ERROR_INDEX,

    MODBUS_ERROR_VALUE,

    MODBUS_ERROR_RANGE,

    MODBUS_ERROR_CRC,

    MODBUS_ERROR_BAD_PROTOCOL,

    MODBUS_ERROR_BAD_TRANSACTION,

    MODBUS_ERROR_ADDRESS,

    MODBUS_ERROR_OTHER,





    MODBUS_FUNCTION_EXCEPTIONAL_BASE = 100,
} ModbusError;








/**
 *
 * @param dest 线圈的数据数组地址
 * @param n 数组中的第几个位 modbus中数组为LSB(最低有效字节)开始，位lsb(最低有效位)开始
 * @return 0 or 1
 */
static inline uint8_t modbusReadBits(const uint8_t *dest, uint16_t n)
{
    return (dest[n >> 3] & (1 << (n & 7))) != 0;
}

/**
 * 以上的反用
 * @param dest
 * @param n
 * @param value
 */
static inline void modbusWriteBits(uint8_t *dest, uint16_t n, uint8_t value)
{
    if (value)
        dest[n >> 3] |= (1 << (n & 7));
    else
        dest[n >> 3] &= ~(1 << (n & 7));
}

/**
 *
 * @param n 为多少个位
 * @return 返回装载n个位需要多少个字节，就是除以8的近1法
 */
static inline uint16_t modbusBitsToBytes(uint16_t n)
{
    return (n + 7) >> 3;
}



static inline uint16_t modbusReadLittleEndian(const uint8_t *p)
{
    uint8_t lo = *p;
    uint8_t hi = *(p + 1);
    return (uint16_t) lo | ((uint16_t) hi << 8);
}


static inline uint16_t modbusWriteLittleEndian(uint8_t *p, uint16_t val)
{
    *p = val & 0xff;
    *(p + 1) = val >> 8;
    return val;
}


static inline uint16_t modbusReadBigEndian(const uint8_t *p)
{
    uint8_t lo = *(p + 1);
    uint8_t hi = *p;
    return (uint16_t) lo | ((uint16_t) hi << 8);
}

static inline uint16_t modbusWriteBigEndian(uint8_t *p, uint16_t val)
{
    *p = val >> 8;
    *(p + 1) = val & 0xff;
    return val;
}

/**
 * 检查modbus寄存器范围 比如：读寄存器地址65535只能读一个寄存器 不能读两个
 * @param index
 * @param count
 * @return
 */
static inline uint8_t modbusCheckRangeU16(uint16_t index, uint16_t count)
{
#ifndef UINT16_MAX
#define UINT16_MAX 0xFFFF
#endif
    return index > UINT16_MAX - count + 1;
}


uint16_t modbusCRC(const uint8_t *data, uint16_t length);


#endif /* MODBUS_BASE_H */
