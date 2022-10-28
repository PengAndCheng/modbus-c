
#include <modbus-slave-rtu-input-align-exec.h>


typedef enum {
    slave_rtu_request_frame_length_align=1,                        //帧长度刚好合适

    slave_rtu_request_frame_align_error_length_short=-1,           //帧长度不够
    slave_rtu_request_frame_align_error_slave_address=-2,          //从机地址不对
    slave_rtu_request_frame_align_error_function_code=-3,          //功能码不对
    slave_rtu_request_frame_align_error_crc=-4,                    //CRC不对

    slave_rtu_request_frame_align_error_recv_old=-100,              //检查完成帧后没有新数据到来
    slave_rtu_request_frame_align_error_count=-101,                 //执行次数异常
    slave_rtu_request_frame_align_error_end=-102,                   //函数执行错误

}slave_rtu_request_frame_align_error_e;

//请求帧允许功能码
static uint8_t slave_rtu_request_frame_align_function_code[] = {1,2,3,4,5,6,15,16};
//请求01功能码帧尺寸
#define FC010203040506_FRAME_SIZE 8
#define FC05_FRAME_SIZE 8
#define FC06_FRAME_SIZE 8
//FC15是动态长度
//FC16是动态长度

#define SLAVE_ADDRESS_OFFSET        0
#define FUNCTION_CODE_OFFSET        1
#define REGISTER_ADDRESS_OFFSET     2
#define REGISTER_ADDRESS_NB_OFFSET  4


int modbus_slave_rtu_input_align_exec(modbus_slave_rtu_input_queue* queue, ModbusSlave *status, uint8_t slave_address){
    //变量尽量定义的goto标签前
    if (queue->recv_head != 0) {
        return slave_rtu_request_frame_align_error_recv_old;
    }

    int count = 0;
    uint16_t recvLen;
    uint8_t* request;
    uint16_t requestLen;
    slave_rtu_request_frame_align_error_e flag = 0;
    int temp;

    __begin:
    count++;
    if (count > queue->buf_size) {
        return slave_rtu_request_frame_align_error_count;
    }
    recvLen = queue->recv_end - queue->recv_head;
    request = &queue->buf[queue->recv_head];
    requestLen = 0;

    //1 判断从机地址及其长度
    if (recvLen < SLAVE_ADDRESS_OFFSET + 1)
    {
        return slave_rtu_request_frame_align_error_length_short;
    }
#if modbus_slave_rtu_request_frame_align_debug
    if (count > 0) {
        printf("modbus_slave_rtu_request_frame_align: count=%d, flag=%d, recvLen=%d.\r\nrecv:",count,flag,recvLen);
        for (int i = 0; i < recvLen; ++i) {
            printf(" %02X",request[i]);
        }
        printf(" .\r\n");
    }
#endif /* #if modbus_slave_rtu_request_frame_align_debug */
    if (request[SLAVE_ADDRESS_OFFSET] == slave_address || request[SLAVE_ADDRESS_OFFSET] == BROADCAST_ADDRESS)
    {
        //合法
    }else {
        flag = slave_rtu_request_frame_align_error_slave_address;
        queue->recv_head = queue->recv_head + SLAVE_ADDRESS_OFFSET + 1;
        goto __begin;
    }

    //2 判断功能码及其长度
    if (recvLen < FUNCTION_CODE_OFFSET + 1)
    {
#if modbus_slave_rtu_request_frame_align_debug
        printf("recvLen < FUNCTION_CODE_OFFSET + 1.\r\n");
#endif /* #if modbus_slave_rtu_request_frame_align_debug */
        return slave_rtu_request_frame_align_error_length_short;
    }
    uint8_t FC = request[FUNCTION_CODE_OFFSET];
    int FC_ok = -1;
    for (int i = 0; i < sizeof(slave_rtu_request_frame_align_function_code); i++)
    {
        if (slave_rtu_request_frame_align_function_code[i]==FC)
        {
            FC_ok=1;
            break;
        }
    }
    if (FC_ok != 1)
    {
        flag = slave_rtu_request_frame_align_error_function_code;
        queue->recv_head = queue->recv_head + FUNCTION_CODE_OFFSET + 1;
        goto __begin;
    }

    //判断寄存器地址及长度
    /* 没用到寄存器地址参数 不计算
    if (recvLen < 4)
    {
        return slave_rtu_request_frame_align_error_length_short;
    }
    uint16_t RA=request[REGISTER_ADDRESS_OFFSET]<<8 | request[REGISTER_ADDRESS_OFFSET+1];
    */

    //判断寄存器个数及长度
    if (recvLen < 6)
    {
#if modbus_slave_rtu_request_frame_align_debug
        printf("recvLen < 6.\r\n");
#endif /* #if modbus_slave_rtu_request_frame_align_debug */
        return slave_rtu_request_frame_align_error_length_short;
    }
    uint16_t RANB=request[REGISTER_ADDRESS_NB_OFFSET]<<8 | request[REGISTER_ADDRESS_NB_OFFSET+1];

    //按功能吗判断其余长度
    if (FC == 1 || FC == 2 || FC==3 || FC==4 || FC==5 || FC==6)
    {
        if (recvLen < FC010203040506_FRAME_SIZE)
        {
#if modbus_slave_rtu_request_frame_align_debug
            printf("recvLen < FC010203040506_FRAME_SIZE.\r\n");
#endif /* #if modbus_slave_rtu_request_frame_align_debug */
            return slave_rtu_request_frame_align_error_length_short;
        }else {
            //CRC校验
            uint8_t* crc;
            crc = &request[FC010203040506_FRAME_SIZE - 2];
            if (modbusCRC(request, FC010203040506_FRAME_SIZE-2) == modbusReadLittleEndian(crc)) {
                requestLen = FC010203040506_FRAME_SIZE;
                flag = slave_rtu_request_frame_length_align;
                goto __align;
            }else {
                flag = slave_rtu_request_frame_align_error_crc;
                queue->recv_head = queue->recv_head + FC010203040506_FRAME_SIZE;
                goto __begin;
            }
        }
    }else if (FC == 15)
    {
        uint8_t bytes = (RANB+7)/8;//8位对其进一法
        int FC15_FLEN = 8 + 1 + bytes;//8为常规 1为字节数 bytes为数据

        if (recvLen < FC15_FLEN)
        {
#if modbus_slave_rtu_request_frame_align_debug
            printf("recvLen < FC15_FLEN.\r\n");
#endif /* #if modbus_slave_rtu_request_frame_align_debug */
            return slave_rtu_request_frame_align_error_length_short;
        }else
        {
            //CRC校验
            uint8_t* crc;
            crc = &request[FC15_FLEN - 2];
            if (modbusCRC(request, FC15_FLEN-2) == modbusReadLittleEndian(crc)) {
                requestLen = FC15_FLEN;
                flag = slave_rtu_request_frame_length_align;
                goto __align;
            }else {
                flag = slave_rtu_request_frame_align_error_crc;
                queue->recv_head = queue->recv_head + FC15_FLEN;
                goto __begin;
            }
        }
    }else if (FC == 16)
    {
        uint8_t bytes = RANB*2;//一个寄存器两字节
        int FC16_FLEN = 8 + 1 + bytes;//8为常规 1为字节数 bytes为数据

        if (recvLen < FC16_FLEN)
        {
#if modbus_slave_rtu_request_frame_align_debug
            printf("recvLen < FC16_FLEN.\r\n");
#endif /* #if modbus_slave_rtu_request_frame_align_debug */
            return slave_rtu_request_frame_align_error_length_short;
        }else
        {
            //CRC校验
            uint8_t* crc;
            crc = &request[FC16_FLEN - 2];
            if (modbusCRC(request, FC16_FLEN-2) == modbusReadLittleEndian(crc)) {
                requestLen = FC16_FLEN;
                flag = slave_rtu_request_frame_length_align;
                goto __align;
            }else {
                flag = slave_rtu_request_frame_align_error_crc;
                queue->recv_head = queue->recv_head + FC16_FLEN;
                goto __begin;
            }
        }
    }else {
#if modbus_slave_rtu_request_frame_align_debug
        printf("FC error.\r\n");
#endif /* #if modbus_slave_rtu_request_frame_align_debug */
    }

    //这里可以添加CRC校验功能 不添加直接返回帧整齐
#if modbus_slave_rtu_request_frame_align_debug
    printf("slave_rtu_request_frame_align_error_end.\r\n");
#endif /* #if modbus_slave_rtu_request_frame_align_debug */
    return slave_rtu_request_frame_align_error_end;

    __align:
    temp = modbusParseRequestRTU(status, slave_address, request, requestLen, 0);
    if (temp) {
#if modbus_slave_rtu_request_frame_align_debug
        printf("modbusParseRequestRTU = %d, requestLen=%d.\r\n",temp,requestLen);
#endif /* #if modbus_slave_rtu_request_frame_align_debug */
    }
    (void)temp;
    queue->recv_head = queue->recv_head + requestLen;
    return flag;
}







void modbus_slave_rtu_input_queue_init(modbus_slave_rtu_input_queue* queue, uint8_t* buf, uint16_t buf_size){
    queue->buf = buf;
    queue->buf_size = buf_size;
    queue->recv_head = 0;
    queue->recv_end = 0;
}

void modbus_slave_rtu_input(modbus_slave_rtu_input_queue* queue, uint8_t* data, uint16_t len){
    //buf前空白 数组执行左移
    if (queue->recv_head > queue->buf_size) {
        //数据异常 将其归为
        queue->recv_head = 0;
    }
    if (queue->recv_head != 0) {
        int nb = queue->recv_end - queue->recv_head;
        for (int i = 0; i < nb; ++i) {
            queue->buf[i] = queue->buf[queue->recv_head+i];
        }
        queue->recv_end = queue->recv_end - queue->recv_head;
        queue->recv_head = 0;
    }

    //将数据压入缓存区 该函数可以用在中断当中
    for (int i = 0; i < len; ++i) {
        if (queue->recv_end < queue->buf_size) {
            queue->buf[queue->recv_end] = data[i];
            queue->recv_end = queue->recv_end + 1;
        }else {
            //数据满 或者recv_end异常将其归为
            queue->recv_end = queue->buf_size;
        }
    }
}

int modbus_slave_rtu_output(ModbusSlave *status, output fn){
    if (fn && status->response_length) {
        int ret = fn(status->response,status->response_length);
        status->response_length = 0;
        return ret;
    }
    return 0;
}
