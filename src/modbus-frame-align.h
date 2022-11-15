
#ifndef MODBUS_FRAME_ALIGN_H
#define MODBUS_FRAME_ALIGN_H



typedef enum FrameType
{
    rtu_request_frame,
    rtu_response_frame,
    tcp_request_frame,
    tcp_response_frame,
}FrameType;

typedef enum FrameError
{
    queue_empty = 0,
    align = 1,

    short_1,
    short_tcp_3,
    short_tcp_4,
    short_tcp_5,
    short_tcp_6,
    short_tcp_7,

    exec_error = -404,
}FrameError;


//帧列队
typedef struct FrameQueue
{
    uint8_t *data;
    int max;
    int head;
    int end;
}FrameQueue;

//帧列队
typedef struct Frame
{
    FrameType type;
    uint8_t* frame_head;
    int frame_lenght;
}Frame;



#endif /* MODBUS_FRAME_ALIGN_H */
