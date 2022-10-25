
#ifndef MODBUS_SLAVE_RTU_REQUEST_FRAME_ALIGN_H
#define MODBUS_SLAVE_RTU_REQUEST_FRAME_ALIGN_H

#define modbus_slave_rtu_request_frame_align_debug 1

typedef struct modbus_slave_rtu_request_frame_buf
{
    uint8_t* buf;
    uint16_t buf_size;
    uint16_t recv_head; //类似于循环列队中的头，但是此列队减少复杂度，每次进行左移避免end溢出
    uint16_t recv_end;  //类似于循环列队中的尾，
} modbus_slave_rtu_request_frame_buf;

//串口输出回调函数类型
typedef int (*modbus_slave_rtu_response_output)(uint8_t* response, uint16_t length);
//初始化串口接收缓冲区
void modbus_slave_rtu_request_frame_buf_init(modbus_slave_rtu_request_frame_buf* msrrfb, uint8_t* buf, uint16_t buf_size);
//串口数据进入缓冲区 可以用在中断中
void modbus_slave_rtu_request_input(modbus_slave_rtu_request_frame_buf* msrrfb, uint8_t* data, uint16_t len);
//检查数据帧对其 尽量不放在中断中 作为数据消费者轮询缓冲区
int modbus_slave_rtu_request_frame_align(ModbusSlave *status,modbus_slave_rtu_request_frame_buf* msrrfb, uint8_t slave_address,modbus_slave_rtu_response_output output);

#endif /* MODBUS_SLAVE_RTU_REQUEST_FRAME_ALIGN_H */
