
#include "modbus-slave.h"
#include "modbus-slave-rtu-input-align-exec.h"


#include "board_uart.h"

static ModbusSlave MS;
static uint8_t request[256];
static uint8_t response[256];
static modbus_slave_rtu_input_queue queue;


void uart1_input_callback(uint8_t* bytes, int lenght){
    modbus_slave_rtu_input(&queue,bytes,lenght);
}

void MS_init(void){

    //初始化modbus从机 主要提供一个数组缓存区 NULL为回调函数，不注册回调函数会使用modbus-slave-callback.h中的默认回调函数
    modbusSlaveInit(&MS,response, sizeof(response),NULL);
    //初始化串口接收需要的缓存列队
    modbus_slave_rtu_input_queue_init(&queue,request,sizeof(response));

    //初始化串口
    uart1_init();
    //注册串口接收函数
    uart1_register_input_callback(uart1_input_callback);
}

void MS_exec(void){
    //检查是否接收到完整的一帧请求 完整则执行从机功能，
    modbus_slave_rtu_input_align_exec(&queue,&MS,1);
    //将以上存在执行输出结果，使用注册的输出函数将响应发出
    modbus_slave_rtu_output(&MS,uart1_output);
}


void MS_main(void){
    MS_init();
    while(1){
        MS_exec();
    }
}

