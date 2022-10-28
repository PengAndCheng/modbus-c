
#ifndef MODBUS_SLAVE_RTU_INPUT_ALIGN_EXEC_H
#define MODBUS_SLAVE_RTU_INPUT_ALIGN_EXEC_H

#include "modbus-slave.h"

#define modbus_slave_rtu_request_frame_align_debug      1

typedef int (*output)(uint8_t* data, int lenght);

typedef struct modbus_slave_rtu_input_queue
{
    uint8_t* buf;
    uint16_t buf_size;
    uint16_t recv_head; //类似于循环列队中的头，但是此列队减少复杂度，每次进行左移避免end溢出
    uint16_t recv_end;  //类似于循环列队中的尾，
} modbus_slave_rtu_input_queue;

//串口输出回调函数类型
typedef int (*modbus_slave_rtu_response_output)(uint8_t* response, uint16_t length);
//初始化串口接收缓冲区
void modbus_slave_rtu_input_queue_init(modbus_slave_rtu_input_queue* queue, uint8_t* buf, uint16_t buf_size);
//串口数据进入缓冲区 可以用在中断中
void modbus_slave_rtu_input(modbus_slave_rtu_input_queue* queue, uint8_t* data, uint16_t len);
//检查数据帧对其 尽量不放在中断中 作为数据消费者轮询缓冲区
int modbus_slave_rtu_input_align_exec(modbus_slave_rtu_input_queue* queue, ModbusSlave *status, uint8_t slave_address);
//输出数据
int modbus_slave_rtu_output(ModbusSlave *status, output fn);




#endif /* MODBUS_SLAVE_RTU_INPUT_ALIGN_EXEC_H */
