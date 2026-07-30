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
// UART4: 串口屏(230400, 优先级4, TX=B17/RX=B18)，帧格式为 0x5B-类型-数值-0x5D。
//   类型1：数值直接设置 state；类型2、3：数值作为 int8 增量调整 XYspeed、Tspeed。

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

//---------------------------------------------------------------------------------------------------------------
// 摄像头碎片计划数据结构定义
// 摄像头发送的源碎片与目标碎片坐标均为相对机械原点的距离，单位：mm。
// X、Y 轴电机每转一圈移动 40 mm，3200 脉冲对应一圈，即 80 脉冲/mm。
//---------------------------------------------------------------------------------------------------------------
typedef struct
{
  uint8 piece_id;         // 碎片编号，取值范围：0-3。
  float source_x_mm;      // 源碎片面积质心相对机械原点的 X 坐标，单位：mm。
  float source_y_mm;      // 源碎片面积质心相对机械原点的 Y 坐标，单位：mm。
  float target_x_mm;      // 目标碎片面积质心相对机械原点的 X 坐标，单位：mm。
  float target_y_mm;      // 目标碎片面积质心相对机械原点的 Y 坐标，单位：mm。
  float take_move_x_mm;   // 从原点或上一块目标位置移动到源碎片的 X 位移，单位：mm。
  float take_move_y_mm;   // 从原点或上一块目标位置移动到源碎片的 Y 位移，单位：mm。
  float put_move_x_mm;    // 从源碎片移动到当前目标位置的 X 位移，单位：mm。
  float put_move_y_mm;    // 从源碎片移动到当前目标位置的 Y 位移，单位：mm。
  uint32 take_move_x_pulse; // 从原点或上一块目标位置移动到源碎片所需的 X 轴脉冲数。
  uint32 take_move_y_pulse; // 从原点或上一块目标位置移动到源碎片所需的 Y 轴脉冲数。
  uint32 put_move_x_pulse;  // 从源碎片移动到当前目标位置所需的 X 轴脉冲数。
  uint32 put_move_y_pulse;  // 从源碎片移动到当前目标位置所需的 Y 轴脉冲数。
  float rotation_deg;     // 从源姿态转到目标姿态的角度，单位：度。
  uint8 Dir_x;            // 移动到源碎片的 X 轴方向，0=负方向，1=正方向。
  uint8 Dir_y;            // 移动到源碎片的 Y 轴方向，0=负方向，1=正方向。
  uint8 put_dir_x;        // 从源碎片移动到目标位置的 X 轴方向，0=负方向，1=正方向。
  uint8 put_dir_y;        // 从源碎片移动到目标位置的 Y 轴方向，0=负方向，1=正方向。
} camera_data_struct;
//-------------------------------------------------------------------------------------------------------------------
// 外部变量声明
//-------------------------------------------------------------------------------------------------------------------
extern uart_rx_struct uart0_rx;
extern uart_rx_struct uart1_rx;
extern uart_rx_struct uart3_rx;
extern uart_rx_struct uart4_rx;

extern camera_data_struct camera_data[4];  // 已完成校验的摄像头连续执行计划，按 piece_id 升序排列。
extern uint32 camera_plan_sequence;         // 已完成校验计划的确认编号。
extern float camera_rectangle_width_mm;     // 已完成校验计划的严格矩形宽度，单位：mm。
extern float camera_rectangle_height_mm;    // 已完成校验计划的严格矩形高度，单位：mm。
extern uint8 camera_piece_count;            // 已完成校验计划中的碎片数量，取值范围：1-4。
extern uint8 camera_plan_ready_flag;        // 已完成校验计划就绪标志，0=无新计划，1=有新计划。
extern uint8 direction;                    // 无线串口 I1 设置的 Y 轴方向：0=负向，1=正向。

extern volatile uint8 state;       // 运行状态: 0=待机, 1=小车运动, 2=镜头运动, 3=两者, 4=其他；可由 UART4 中断更新。
extern uint8 last_state;           // 上一次运行状态
extern uint16 XYspeed;             // X、Y 轴速度，单位：RPM；UART4 可按有符号增量调整。
extern uint16 Tspeed;              // T 轴电机轴速度，单位：RPM；UART4 可按有符号增量调整。
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

void uart3_init_motor(void);
void uart3_rx_callback(uint32 state, void *ptr);
void uart3_process_data(void);

void uart5_rx_callback(uint32 state, void *ptr);

void uart4_init_screen(void);
void uart4_rx_callback(uint32 interrupt_state, void *ptr);
void uart4_process_data(void);

void uart_printf(uart_index_enum uart_index, const char *format, ...);



#endif
