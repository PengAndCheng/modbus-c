#include "stdio.h"

#include "modbus-frame-align.h"


#define MODBUS_TCP_ADU_MIN     8
#define MODBUS_TCP_ADU_MAX     260

const uint8_t FunctionCode[] = {1,2,3,4,5,6,15,16,0x81,0x82,0x83,0x84};


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


FrameError frameAlignParse(FrameQueue* Q){
    //将用到大量goto 变量定义在这里
    int qlen;
    uint8_t *frame;
    int reduce_bytes;

    __start:
    qlen = frameQueueDataLenght(Q);
    if (qlen == 0) {
        return queue_empty;
    }
    frame = frameQueueDataHead(Q);



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

                                }else {
                                    //功能码异常  重新从协议标识符处扫描
                                    reduce_bytes = 4;
                                    goto __reduce_bytes;
                                }
                            }
                        }else {
                            //异常超帧或帧不足 重新从协议标识符处扫描
                            reduce_bytes = 4;
                            goto __reduce_bytes;
                        }
                    }else {
                        //不可能执行到这里的
                        goto __exec_error;
                    }
                }else {
                    //不满足协议标识符第二字节
                    reduce_bytes = 4;
                    goto __reduce_bytes;
                }
            }else {
                //不满足协议标识符第一字节
                reduce_bytes = 3;
                goto __reduce_bytes;
            }
        }
    }

    __exec_error:
    return exec_error;

    __reduce_bytes:
    frameQueueReduceHead(Q,reduce_bytes);
    goto __start;
}





