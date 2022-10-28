
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
    uart1_init();
    uart1_register_input_callback(uart1_input_callback);

    modbusSlaveInit(&MS,response, sizeof(response),NULL);
    modbus_slave_rtu_input_queue_init(&queue,request,sizeof(response));
}

void MS_exec(void){
    modbus_slave_rtu_input_align_exec(&queue,&MS,1);
    modbus_slave_rtu_output(&MS,uart1_output);
}

