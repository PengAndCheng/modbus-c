#include "stdio.h"

#include "modbus-frame-align.h"


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



const uint8_t FunctionCode[] = {1,2,3,4,5,6,15,16,0x81,0x82,0x83,0x84,0x8F,0x90};
const uint8_t ExceptionCode[] = {0x81,0x82,0x83,0x84,0x8F,0x90};


void frameQueueInit(FrameQueue* Q, uint8_t* data, int data_size){
    Q->data = data;
    Q->max = data_size;
    Q->head = 0;
    Q->end = 0;
}

//将列队的数据头部对齐的0位置 线程不安全
void frameQueueHeadAlignIndex0(FrameQueue* Q){
    //异常归零
    if (Q->head > Q->end || Q->head >= Q->max || Q->end >= Q->max) {
        Q->head = Q->end = 0;
    }

    //head非零归零
    if (Q->head != 0) {
        int nb = Q->end - Q->head;
        for (int i = 0; i < nb; ++i) {
            Q->data[i] = Q->data[Q->head+i];
            Q->end = nb;
            Q->head = 0;
        }
    }
}

static inline int frameQueueDataLenght(FrameQueue* Q){
    return Q->end - Q->head;
}

static inline uint8_t* frameQueueDataHead(FrameQueue* Q){
    return &Q->data[Q->head];
}

static inline frameQueueReduceHead(FrameQueue* Q,int nb){
    Q->head = Q->head + nb;
}


static inline int isFunctionCode(uint8_t fcc){
    for (int i = 0; i < sizeof(FunctionCode); ++i) {
        if (FunctionCode[i]==fcc) {
            return 1;
        }
    }
    return 0;
}

static inline int isExceptionCode(uint8_t fcc){
    for (int i = 0; i < sizeof(ExceptionCode); ++i) {
        if (ExceptionCode[i]==fcc) {
            return 1;
        }
    }
    return 0;
}



FrameError frameAlignParse(FrameQueue* Q,Frame* ret){
    //将用到大量goto 变量定义在这里
    int qlen;
    uint8_t *frame;

    __start:
    qlen = frameQueueDataLenght(Q);
    if (qlen == 0) {
        return queue_empty;
    }
    frame = frameQueueDataHead(Q);
    ret->isOK = 0;

    //只有一个字节不分析
    if (qlen == 1) {
        return short_1;
    }

    if (qlen >= 2) {
        if (isFunctionCode(frame[1])) {
            //可能是RTU



        }else {
            //可能是TCP
            if (qlen >= 3 && frame[2] == 0) {
                //满足协议标识符第一个字节
                if (qlen == 3) {
                    return short_tcp_3;
                }else if (qlen >= 4 && frame[3] == 0) {
                    //满足协议标识符第二个字节
                    if (qlen == 4) {
                        return short_tcp_4;
                    }else if (qlen == 5) {
                        return short_tcp_5;
                    }else if (qlen >= 6) {
                        //获取MBAP中的长度
                        uint16_t MBAP_lenght = frame[4]<<8 | frame[5];
                        if ( MBAP_lenght>=(MODBUS_TCP_ADU_MIN - 6) && MBAP_lenght + 6 <= MODBUS_TCP_ADU_MAX) {
                            //MBAP_lenght指示正确
                            if (qlen == 6) {
                                return short_tcp_6;
                            }else if (qlen == 7) {
                                return short_tcp_7;
                            }else if (qlen >= 8) {
                                if (isFunctionCode(frame[7])) {
                                    //缩小到功能码范围内
                                    if (frame[7] == 01 || frame[7] == 02 || frame[7] == 03 || frame[7] == 04) {
                                        if (7 + 2 <= qlen) {
                                            int read_bytes = frame[8];
                                            if (read_bytes + 1 == MBAP_lenght - 1) {
                                                //强制认为这是响应帧了
                                                //当01 02读取17-24个寄存器时 依然存在二义性 但是读取17-24个寄存器时read_bytes必定为0 所以取消二议性
                                                //当03 04响应帧必为偶数
                                                if (MBAP_lenght + 6 <= qlen) {
                                                    ret->frame_head = frame;
                                                    ret->frame_lenght = MBAP_lenght + 6;
                                                    ret->type = tcp_response_01020304;
                                                    ret->isOK = 1;
                                                    return tcp_response_01020304;
                                                }else {
                                                    return short_tcp_pdu_response_01020304;
                                                }
                                            }else if (MBAP_lenght + 6 == 7 + 5) {
                                                //满足请求帧长度
                                                if (MBAP_lenght + 6 <= qlen) {
                                                    ret->frame_head = frame;
                                                    ret->frame_lenght = MBAP_lenght + 6;
                                                    ret->type = tcp_request_01020304;
                                                    ret->isOK = 1;
                                                    return tcp_request_01020304;
                                                }else {
                                                    return short_tcp_pdu_request_01020304;
                                                }
                                            }else {
                                                //PDU长度对得上
                                                goto __reduce_byte;
                                            }
                                        }else {
                                            return short_tcp_pdu_01020304;
                                        }
                                    }else if (frame[7] == 05 || frame[7] == 06) {
                                        //收发一样 且长度固定
                                        if (MBAP_lenght + 6 != 7+5) {
                                            //PDU长度对不上
                                            goto __reduce_byte;
                                        }else {
                                            //PDU长度对得上
                                            if (MBAP_lenght + 6 <= qlen) {
                                                ret->frame_head = frame;
                                                ret->frame_lenght = MBAP_lenght + 6;
                                                ret->type = tcp_request_0506;
                                                ret->isOK = 1;
                                                return tcp_request_0506;
                                            }else {
                                                return short_tcp_pdu_request_0506;
                                            }
                                        }
                                    }else if (frame[7] == 15 || frame[7] == 16) {
                                        if (MBAP_lenght - 1 == 5) {
                                            //必为响应帧 响应长度固定且短于请求
                                            if (MBAP_lenght + 6 <= qlen) {
                                                ret->frame_head = frame;
                                                ret->frame_lenght = MBAP_lenght + 6;
                                                ret->type = tcp_response_1516;
                                                ret->isOK = 1;
                                                return tcp_response_1516;
                                            }else {
                                                return short_tcp_pdu_response_1516;
                                            }
                                        }else {
                                            if (13 <= qlen) {
                                                int write_bytes = frame[12];
                                                if (write_bytes + 6 == MBAP_lenght - 1) {
                                                    //PDU长度对得上
                                                    if (MBAP_lenght + 6 <= qlen) {
                                                        ret->frame_head = frame;
                                                        ret->frame_lenght = MBAP_lenght + 6;
                                                        ret->type = tcp_request_1516;
                                                        ret->isOK = 1;
                                                        return tcp_request_1516;
                                                    }else {
                                                        return short_tcp_pdu_request_1516;
                                                    }
                                                }else {
                                                    //PDU长度对不上
                                                    goto __reduce_byte;
                                                }
                                            }else{
                                                return short_tcp_pdu_request_1516;
                                            }
                                        }
                                    }else if (isExceptionCode(frame[7])) {
                                        if (MBAP_lenght != 3) {
                                            //PDU长度对不上
                                            goto __reduce_byte;
                                        }else {
                                            if (MBAP_lenght + 6 <= qlen) {
                                                ret->frame_head = frame;
                                                ret->frame_lenght = MBAP_lenght + 6;
                                                ret->type = tcp_response_ExceptionCode;
                                                ret->isOK = 1;
                                                return tcp_response_ExceptionCode;
                                            }else {
                                                return short_tcp_pdu_ExceptionCode;
                                            }
                                        }
                                    }
                                }else {
                                    //功能码异常  重新从协议标识符处扫描
                                    goto __reduce_byte;
                                }
                            }
                        }else {
                            //异常超帧或帧不足 重新从协议标识符处扫描
                            goto __reduce_byte;
                        }
                    }else {
                        //不可能执行到这里的
                        goto __exec_error;
                    }
                }else {
                    //不满足协议标识符第二字节
                    goto __reduce_byte;
                }
            }else {
                //不满足协议标识符第一字节
                goto __reduce_byte;
            }
        }
    }

    __exec_error:
    return exec_error;

    __reduce_byte:
    frameQueueReduceHead(Q,1);
    goto __start;
}





