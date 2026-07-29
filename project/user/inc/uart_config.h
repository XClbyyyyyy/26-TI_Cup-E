/*********************************************************************************************************************
* 本文件基于逐飞科技 MSPM0G3519 开源库开发
* 
* 文件名称          uart_config
* 备注信息          串口配置头文件
********************************************************************************************************************/

#ifndef _UART_CONFIG_H_
#define _UART_CONFIG_H_

#include "zf_common_headfile.h"
#include <stdarg.h>  // 可变参数支持

//-------------------------------------------------------------------------------------------------------------------
// 串口用途定义
//-------------------------------------------------------------------------------------------------------------------
// UART0: 串口调试(230400, 优先级7, TX=A10/RX=A11) 通道0=state, 通道1=left_speed, 通道2=right_speed, 通道3=Kp
// UART1: 摄像头(921600, 优先级2, TX=A8/RX=A9)
//   POS,mode,seq,x,y,distance_mm\n  ASCII协议
//   发"1\n"切换模式1(中心点), 发"2\n"切换模式2(圆周点)
// UART3: 步进电机(115200, 优先级1, TX=A14/RX=A13)
// UART6: 串口屏(230400, 优先级4, TX=B22/RX=B21)

//-------------------------------------------------------------------------------------------------------------------
// 引脚定义
//-------------------------------------------------------------------------------------------------------------------
#define UART0_TX_PIN          UART0_TX_A10
#define UART0_RX_PIN          UART0_RX_A11

#define UART1_TX_PIN          UART1_TX_A8
#define UART1_RX_PIN          UART1_RX_A9

#define UART3_TX_PIN          UART3_TX_A14
#define UART3_RX_PIN          UART3_RX_A13

#define UART4_TX_PIN          UART4_TX_B17
#define UART4_RX_PIN          UART4_RX_B18

#define UART5_TX_PIN          UART5_TX_A1
#define UART5_RX_PIN          UART5_RX_A0

#define UART6_TX_PIN          UART6_TX_B22
#define UART6_RX_PIN          UART6_RX_B21

//-------------------------------------------------------------------------------------------------------------------
// 串口接收结构体定义
//-------------------------------------------------------------------------------------------------------------------
typedef struct
{
  uint8 state;       // 状态机状态: 0=等待帧头, 1=接收字符串, 2=接收参数包
  uint8 rx_buf[256]; // 接收缓冲区: 存储帧头和帧尾之间的数据
  uint8 rx_len;      // 当前接收长度
  uint8 data_len;    // 数据长度: 预期接收的数据长度(备用)
  uint8 frame_ready; // 帧接收完成标志: 1=一帧数据接收完成, 0=未完成
} uart_rx_struct;

//-------------------------------------------------------------------------------------------------------------------
// 外部变量声明
//-------------------------------------------------------------------------------------------------------------------
extern uart_rx_struct uart0_rx;
extern uart_rx_struct uart1_rx;
extern uart_rx_struct uart3_rx;
extern uart_rx_struct uart6_rx;
extern float param_data[8];        // 参数调试数据: 通道0-7

extern uint16 camera_target_x;     // 摄像头目标X坐标
extern uint16 camera_target_y;     // 摄像头目标Y坐标
extern int16 camera_move_x;        // 摄像头X偏移
extern int16 camera_move_y;        // 摄像头Y偏移
extern uint8 camera_mode;          // 摄像头模式: 1=中心点, 2=圆周点
extern uint16 camera_sequence;     // 帧序号(递增)
extern int16 camera_distance;      // 测距结果(mm), -1=无效
extern uint8 camera_target_valid;  // 摄像头有效靶标标志: 0=无靶, 1=有靶

extern uint8 state;                // 运行状态: 0=待机, 1=小车运动, 2=镜头运动, 3=两者, 4=其他
extern uint8 last_state;           // 上一次运行状态
//-------------------------------------------------------------------------------------------------------------------
// 函数声明
//-------------------------------------------------------------------------------------------------------------------
void uart_config_init(void);

void uart0_init_debug(void);
void uart0_rx_callback(uint32 state, void *ptr);
void uart0_process_data(void);

void uart1_init_camera(void);
void uart1_rx_callback(uint32 state, void *ptr);
void uart1_process_data(void);
void uart1_clear_frame(void);

void uart3_init_motor(void);
void uart3_rx_callback(uint32 state, void *ptr);
void uart3_process_data(void);

void uart4_rx_callback(uint32 state, void *ptr);
void uart5_rx_callback(uint32 state, void *ptr);

void uart6_init_screen(void);
void uart6_rx_callback(uint32 state, void *ptr);
void uart6_process_data(void);

void uart_printf(uart_index_enum uart_index, const char *format, ...);



#endif
