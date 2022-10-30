
#include "modbus-base.h"




uint16_t modbusCRC(const uint8_t *data, uint16_t length)
{
    //此函数可以针对CPU改为硬件CRC
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= (uint16_t) data[i];
        for (uint8_t j = 8; j != 0; j--)
        {
            if ((crc & 0x0001) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
                crc >>= 1;
        }
    }

    return crc;
}
