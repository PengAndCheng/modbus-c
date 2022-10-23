# modbus-c
## 前言
###### 本人从事嵌入式开发modbus过程中，经过对比各种c语言的modbus库总结出的一套c语言modbus功能。在modbus开发过程中曾经遇到许多问题，本c功能都将其解决，也算是本modbus-c的一方优点，其优点如下：
###### 1、本c功能使用大量内联函数，使程序代码更加整体高效，寄存器的数据读写均提供内联的回调函数，做到模块化和高内聚低耦合。
###### 2、程序以模块化后，可以快速在MCU上多串口实现多从机和多主机。
###### 3、不依赖于内存池或者内存堆，仅仅在初始化时提供一个数组buf。
###### 4、不依赖于RTOS系统，即使是modbus master也不依赖操作系统，使用的是轮询状态机，为什么这会成为一优点？我在实际开发中RTOS确实在编程模型上带来了优势，但是使用RTOS时必会设计成事件驱动，中断中使用系统特性的信号量，邮箱，列队，MUC性能不上来时会遇到中断问题，系统tick中断和可悬挂中断的调度掌握也是需要对MCU特别了解，由于国产化CPU的代替，代替CPU的库和现有RTOS不是很匹配，比如freeRTOS和Cortex-M3是绝配，能做到不关中断仅仅屏蔽部分中断，但是同在Cortex-M3的国产车规级MCU中的官方匹配freeRTOS中就没有这个特性；在risc-v的MUC中差异更大。
###### 5、极大的优势：本库包含了一个对数据帧接收是否整齐的检查，放弃了RTU协议规定的3.5个字符间隔为帧边界。在实际开发中使用3.5字符鉴定帧时，当碰到串口MDA数据空闲接收时T3.5模式失效，因为DMA一帧中断多次，后面次数的字符多，首次开启定时器时逻辑错了；遇到性能不是很好的MCU时3.5字符不是很严格也失效，一帧分成两帧发都可能；特别是在4G透传中，4G字符一个接着一个，一帧中两个字符间隔时间延时可能上秒。本modbus库中包含对接收帧的对齐检查，在实际应用中，本人使用时配合循环列队，从未出现过丢失数据帧。
###### 6、modbus数据解析过程，都将关键数据的指针给出，适应了网关功能的开发，也适应了TCP和RTU互转，本人在实际开发网关中也是极度依赖了这套指针解析，减小数据拷贝，性能稳定。