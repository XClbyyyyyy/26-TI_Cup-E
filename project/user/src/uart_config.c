/*********************************************************************************************************************
* 本文件基于逐飞科技 MSPM0G3519 开源库开发
* 
* 文件名称          uart_config
* 备注信息          串口配置（UART0、UART1、UART3、UART6）
********************************************************************************************************************/

#include "uart_config.h"
#include "timer_config.h"  // get_system_time_ms

//-------------------------------------------------------------------------------------------------------------------
// 接收结构体定义
//-------------------------------------------------------------------------------------------------------------------
uart_rx_struct uart0_rx;  // UART0 接收结构体 - 串口调试
uart_rx_struct uart1_rx;  // UART1 接收结构体 - 摄像头
uart_rx_struct uart3_rx;  // UART3 接收结构体 - 步进电机
uart_rx_struct uart6_rx;  // UART6 接收结构体 - 串口屏
float param_data[8];      // 参数调试数据: 通道0-7

uint16 camera_target_x;     // 摄像头目标X坐标
uint16 camera_target_y;     // 摄像头目标Y坐标
int16 camera_move_x;        // 摄像头偏移X坐标
int16 camera_move_y;        // 摄像头偏移Y坐标
uint8 camera_mode;          // 摄像头模式: 1=中心点, 2=圆周点
uint16 camera_sequence;     // 帧序号(递增)
int16 camera_distance;      // 测距结果(mm), -1=无效
uint8 camera_target_valid;  // 摄像头有效靶标标志: 0=无靶, 1=有靶

// UART1最新完整帧缓存：中断始终覆盖旧帧，主循环只处理最后收到的一帧。
// volatile表示这些变量会在中断中修改，避免编译器把主循环中的读取结果缓存起来。
static volatile uint8 uart1_frame_buf[129];  // 中断保存的最新POS文本，128字节数据+1字节结束符空间
static volatile uint8 uart1_frame_len;       // 最新完整帧的有效数据长度，不包含换行符
static volatile uint8 uart1_frame_ready;     // 1=有可处理的新完整帧，0=没有新帧

uint8 state = 0;            // 运行状态: 0=待机, 1=小车运动, 2=镜头运动, 3=两者, 4=其他
uint8 last_state=0;         // 上一次运行状态
extern uint8 count;         // 计数器

//====================================================================================================================
// UART0 - 串口调试
//====================================================================================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART0 初始化
//-------------------------------------------------------------------------------------------------------------------
void uart0_init_debug(void)
{
  uart_init(UART_0, 230400, UART0_TX_PIN, UART0_RX_PIN);
  uart_set_callback(UART_0, uart0_rx_callback, NULL);
  uart_set_interrupt_config(UART_0, UART_INTERRUPT_CONFIG_RX_ENABLE);
  NVIC_SetPriority(UART0_INT_IRQn, 7);

  uart0_rx.state = 0;  // 状态0: 等待帧头
  uart0_rx.rx_len = 0;
  uart0_rx.frame_ready = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART0 接收回调函数（固定长度参数包）
// 状态机逻辑：
//   state=0：等待参数包帧头 0x55。
//   state=1：接收帧头后的 7 个参数字节，累计 8 字节后置接收完成标志。
//-------------------------------------------------------------------------------------------------------------------
void uart0_rx_callback(uint32 state, void *ptr)
{
  uint8 temp_data = 0;  // 临时保存接收到的字节。

  if(state == UART_INTERRUPT_STATE_RX)
  {
    while(uart_query_byte(UART_0, &temp_data) == 1)
    {
      if(uart0_rx.state == 0)
      {
        if(temp_data == 0x55)
        {
          uart0_rx.rx_buf[0] = temp_data;  // 保存参数包帧头。
          uart0_rx.rx_len = 1;
          uart0_rx.state = 1;  // 开始接收参数包剩余的 7 个字节。
        }
      }
      else
      {
        uart0_rx.rx_buf[uart0_rx.rx_len] = temp_data;
        uart0_rx.rx_len++;

        if(uart0_rx.rx_len >= 8)
        {
          uart0_rx.frame_ready = 1;  // 参数包接收完成。
          uart0_rx.state = 0;
        }
      }
    }
  }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析参数调试数据包
// 备注信息     数据包格式: 55 XX XX CH [4字节float]
//-------------------------------------------------------------------------------------------------------------------
static void uart0_parse_param(uint8 *data)
{
  uint8 channel = 0;  // 通道号
  union               // float/uint8 共用体
  {
    float f_val;
    uint8 b_val[4];
  } value;

  channel = data[3];  // 协议字节值: 0x01=通道0, 0x02=通道1, ...

  value.b_val[0] = data[4];
  value.b_val[1] = data[5];
  value.b_val[2] = data[6];
  value.b_val[3] = data[7];

  if(channel >= 1 && channel <= 8)  // 有效通道范围
  {
    param_data[channel - 1] = value.f_val;  // 字节值减1得到通道号
  }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART0 数据处理函数
// 备注信息     通道0: state, 通道1: 标准速度, 通道2: Kd, 通道3: Kp, 通道4: circle
//-------------------------------------------------------------------------------------------------------------------
void uart0_process_data(void)
{
  if(uart0_rx.frame_ready == 1)
  {
    uart_printf(UART_0, "Receive\r\n");  // 接收到数据回传确认

    if(uart0_rx.rx_buf[0] == 0x55)  // 参数包格式
    {
      uart0_parse_param(uart0_rx.rx_buf);
      
      // 根据通道号更新对应变量
      if(uart0_rx.rx_buf[3] == 1)  // 通道0: state
      {
        last_state = state;           // 保存上一次运行状态
        state = (uint8)param_data[0];
      }
    }
    uart0_rx.frame_ready = 0;
  }
}

//====================================================================================================================
// UART1 - 摄像头
//====================================================================================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART1 初始化
//-------------------------------------------------------------------------------------------------------------------
void uart1_init_camera(void)
{
  uart_init(UART_1, 921600, UART1_TX_PIN, UART1_RX_PIN);
  uart_set_callback(UART_1, uart1_rx_callback, NULL);
  uart_set_interrupt_config(UART_1, UART_INTERRUPT_CONFIG_RX_ENABLE);
  // 摄像头使用921600波特率；发生接收错误时也进入UART1中断，读空FIFO后继续接收。
  DL_UART_Main_enableInterrupt(UART1,
                               DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR    |
                               DL_UART_MAIN_INTERRUPT_BREAK_ERROR      |
                               DL_UART_MAIN_INTERRUPT_PARITY_ERROR     |
                               DL_UART_MAIN_INTERRUPT_FRAMING_ERROR    |
                               DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
                               DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
  NVIC_SetPriority(UART1_INT_IRQn, 0);
  
  uart1_rx.rx_len = 0;
  uart1_rx.frame_ready = 0;
  uart1_frame_len = 0;
  uart1_frame_ready = 0;
  camera_mode = 0;
  camera_sequence = 0;
  camera_target_x = 0;
  camera_target_y = 0;
  camera_move_x = 0;
  camera_move_y = 0;
  camera_distance = -1;
  camera_target_valid = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART1 接收回调函数 (ASCII行模式)
// 接收逻辑:
//   协议格式: POS,mode,seq,x,y,distance_mm\n
//   接收时先写入普通接收缓冲；收到换行后复制为最新完整帧，旧完整帧允许被覆盖。
//   主循环随后复制该完整帧到本地数组解析，避免被下一帧接收过程改写。
//   缓冲区溢出时丢弃该行重新开始
//-------------------------------------------------------------------------------------------------------------------
void uart1_rx_callback(uint32 state, void *ptr)
{
  uint8 temp_data = 0;  // 临时存储接收到的字节

  if(state == UART_INTERRUPT_STATE_RX)
  {
    // 摄像头波特率为921600，单次中断可能已经累计多个字节，必须读空接收FIFO。
    while(uart_query_byte(UART_1, &temp_data) == 1)
    {
      if(temp_data == '\n')  // 行结束符
      {
        if(uart1_rx.rx_len > 0)
        {
          // 一行接收完成：覆盖保存最新完整帧，主循环未处理的旧帧可直接丢弃。
          for(uint8 i = 0; i < uart1_rx.rx_len; i++)
            uart1_frame_buf[i] = uart1_rx.rx_buf[i];
          uart1_frame_len = uart1_rx.rx_len;
          uart1_frame_ready = 1;  // 通知主循环有新帧可取
        }
        uart1_rx.rx_len = 0;
      }
      else  // 累积字符
      {
        if(uart1_rx.rx_len < 128)  // 防止溢出
        {
          uart1_rx.rx_buf[uart1_rx.rx_len] = temp_data;
          uart1_rx.rx_len = uart1_rx.rx_len + 1;
        }
        else
          uart1_rx.rx_len = 0;  // 溢出, 丢弃重新开始
      }
    }
  }
}

//---------------------------------------------------------------------------------------------------------------
// 函数简介     清除UART1已完成帧状态
//---------------------------------------------------------------------------------------------------------------
void uart1_clear_frame(void)
{
  __disable_irq();
  uart1_frame_ready = 0;
  uart1_frame_len = 0;
  uart1_rx.rx_len = 0;
  __enable_irq();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART1 数据处理函数
// 备注信息     解析 ASCII 格式: POS,<mode>,<seq>,<x>,<y>,<dist>
//               仅当 sequence 递增时更新坐标, 防止重复数据
//-------------------------------------------------------------------------------------------------------------------
void uart1_process_data(void)
{
  uint8 frame_buf[129];
  uint8 frame_len;

  // 复制共享帧缓冲期间禁止中断，防止ISR覆盖数据而得到混合帧。
  // 临界区只包含129字节以内的复制，不在其中执行串口输出或字符串解析。
  __disable_irq();
  if(uart1_frame_ready == 0)
  {
    __enable_irq();
    return;  // 没有新帧，继续使用上一次已更新的目标坐标
  }

  frame_len = uart1_frame_len;
  for(uint8 i = 0; i < frame_len; i++)
    frame_buf[i] = uart1_frame_buf[i];  // 复制到局部数组，后续解析不受ISR影响
  uart1_frame_ready = 0;                // 当前最新帧已取走
  __enable_irq();                       // 尽快恢复UART1接收中断

  frame_buf[frame_len] = '\0';  // 局部副本补结束符，作为C字符串解析

  // 每收到一条完整摄像头帧，立即转发原始POS文本到上位机。
    uart_printf(UART_0, "%s\r\n", frame_buf);

  // 手动解析: POS,mode,seq,x,y,dist
  uint8 *p = frame_buf;
    if(p[0] != 'P' || p[1] != 'O' || p[2] != 'S' || p[3] != ',')
      return;
    p = p + 4;  // 跳过 "POS,"

    // 解析 mode
    uint8 mode = 0;
    while(*p >= '0' && *p <= '9')
    {
      mode = mode * 10 + (*p - '0');
      p++;
    }
    if(*p != ',') return;
    p++;

    // 解析 sequence
    uint16 seq = 0;
    while(*p >= '0' && *p <= '9')
    {
      seq = seq * 10 + (*p - '0');
      p++;
    }
    if(*p != ',') return;
    p++;

    // 解析 x
    uint16 x = 0;
    while(*p >= '0' && *p <= '9')
    {
      x = x * 10 + (*p - '0');
      p++;
    }
    if(*p != ',') return;
    p++;

    // 解析 y
    uint16 y = 0;
    while(*p >= '0' && *p <= '9')
    {
      y = y * 10 + (*p - '0');
      p++;
    }
    if(*p != ',') return;
    p++;

    // 解析 distance (可能为负: -1)
    int16 dist = 0;
    int8 sign = 1;
    if(*p == '-')
    {
      sign = -1;
      p++;
    }
    while(*p >= '0' && *p <= '9')
    {
      dist = dist * 10 + (*p - '0');
      p++;
    }
    dist = dist * sign;

    // 最新完整POS帧始终覆盖旧坐标；帧序号仅用于记录，不作为丢弃条件。
    camera_mode = mode;
    camera_sequence = seq;
    camera_target_x = x;
    camera_target_y = y;

    camera_move_x = (int16)x - 320 + 15;
    camera_move_y = (int16)y - 240 + 5 ;
    camera_distance = dist;
    if(dist >= 0)
      camera_target_valid = 1;  // 距离非负表示本帧检测到有效靶标
    else
      camera_target_valid = 0;  // 距离为-1表示本帧无有效靶标

    // 调试输出：该POS帧对应的云台实际控制偏移。
    uart_printf(UART_0, "mode:%d seq:%d move_x:%d move_y:%d dist:%d valid:%d\r\n", camera_mode, camera_sequence, camera_move_x, camera_move_y, camera_distance, camera_target_valid);
}

//====================================================================================================================
// UART3 - 步进电机
//====================================================================================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART3 初始化
//-------------------------------------------------------------------------------------------------------------------
void uart3_init_motor(void)
{
  uart_init(UART_3, 115200, UART3_TX_PIN, UART3_RX_PIN);
  uart_set_callback(UART_3, uart3_rx_callback, NULL);
  uart_set_interrupt_config(UART_3, UART_INTERRUPT_CONFIG_RX_ENABLE);
  NVIC_SetPriority(UART3_INT_IRQn, 1);
  
  uart3_rx.state = 0;  // 状态0: 等待帧头
  uart3_rx.rx_len = 0;
  uart3_rx.frame_ready = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART3 接收回调函数 (状态机模式)
// 状态机逻辑:
//   state=0: 等待帧头 '['
//   state=1: 接收数据直到收到帧尾 ']', 收到帧尾后置 frame_ready=1
//-------------------------------------------------------------------------------------------------------------------
void uart3_rx_callback(uint32 state, void *ptr)
{
  uint8 temp_data = 0;  // 临时存储接收到的字节
  
  if(state == UART_INTERRUPT_STATE_RX)
  {
    if(uart_query_byte(UART_3, &temp_data) == 1)
    {
      if(uart3_rx.state == 0)  // 状态0: 等待帧头
      {
        if(temp_data == '[')
        {
          uart3_rx.state = 1;  // 收到帧头, 进入状态1
          uart3_rx.rx_len = 0;
        }
      }
      else  // 状态1: 接收数据
      {
        if(temp_data == ']')
        {
          uart3_rx.frame_ready = 1;  // 收到帧尾, 标志帧接收完成
          uart3_rx.state = 0;
        }
        else
        {
            uart3_rx.rx_buf[uart3_rx.rx_len] = temp_data;
            uart3_rx.rx_len = uart3_rx.rx_len + 1;
        }
      }
    }
  }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART3 数据处理函数
//-------------------------------------------------------------------------------------------------------------------
void uart3_process_data(void)
{
  if(uart3_rx.frame_ready == 1)
  {
    // 此处添加数据处理代码
    
    uart3_rx.frame_ready = 0;
  }
}

//====================================================================================================================
// UART4/UART5 - 预留接收回调
//====================================================================================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART4 接收回调预留函数
// 备注信息     当前不处理 UART4 接收数据。
//-------------------------------------------------------------------------------------------------------------------
void uart4_rx_callback(uint32 state, void *ptr)
{
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART5 接收回调预留函数
// 备注信息     当前不处理 UART5 接收数据。
//-------------------------------------------------------------------------------------------------------------------
void uart5_rx_callback(uint32 state, void *ptr)
{
}

//====================================================================================================================
// UART6 - 串口屏
//====================================================================================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART6 初始化
//-------------------------------------------------------------------------------------------------------------------
void uart6_init_screen(void)
{
  uart_init(UART_6, 230400, UART6_TX_PIN, UART6_RX_PIN);
  uart_set_callback(UART_6, uart6_rx_callback, NULL);
  uart_set_interrupt_config(UART_6, UART_INTERRUPT_CONFIG_RX_ENABLE);
  NVIC_SetPriority(UART6_INT_IRQn, 2);

  uart6_rx.state = 0;  // 状态0: 等待帧头
  uart6_rx.rx_len = 0;
  uart6_rx.frame_ready = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART6 接收回调函数 (固定四字节帧)
// 状态机逻辑:
//   state=0: 等待帧头0x5B
//   state=1: 接收运行状态state
//   state=2: 接收目标圈数circle
//   state=3: 等待并验证帧尾0x5D
//-------------------------------------------------------------------------------------------------------------------
void uart6_rx_callback(uint32 state, void *ptr)
{
  uint8 temp_data = 0;  // 临时存储接收到的字节

  if(state == UART_INTERRUPT_STATE_RX)
  {
    if(uart_query_byte(UART_6, &temp_data) == 1)
    {
      if(uart6_rx.state == 0)
      {
        if(temp_data == 0x5B)
        {
          uart6_rx.state = 1;  // 收到帧头后等待state
          uart6_rx.rx_len = 0;
        }
      }
      else if(uart6_rx.state == 1)
      {
        uart6_rx.rx_buf[0] = temp_data;  // 保存state字节
        uart6_rx.rx_len = 1;
        uart6_rx.state = 2;
      }
      else if(uart6_rx.state == 2)
      {
        uart6_rx.rx_buf[1] = temp_data;  // 保存count字节
        uart6_rx.rx_len = 2;
        uart6_rx.state = 3;
      }
      else
      {
        if(temp_data == 0x5D)
          uart6_rx.frame_ready = 1;  // 帧头、两个数据字节和帧尾均正确

        if(temp_data == 0x5B)
        {
          uart6_rx.state = 1;  // 帧尾错误但当前字节是新帧头，立即重新同步
          uart6_rx.rx_len = 0;
        }
        else
          uart6_rx.state = 0;
      }
    }
  }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART6 数据处理函数
// 备注信息     仅处理格式为0x5B-state-circle-0x5D的有效固定帧
//-------------------------------------------------------------------------------------------------------------------
void uart6_process_data(void)
{
  if(uart6_rx.frame_ready == 1)
  {
    if(uart6_rx.rx_len == 2)
    {
      last_state = state;           // 保存上一次运行状态
      state = uart6_rx.rx_buf[0];   // 第2字节为运行状态
    }
    uart6_rx.frame_ready = 0;
  }
}

//====================================================================================================================
// 串口总初始化函数
//====================================================================================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     串口总初始化函数
//-------------------------------------------------------------------------------------------------------------------
void uart_config_init(void)
{
  uart0_init_debug();  
  uart1_init_camera();  
  uart3_init_motor();
  uart6_init_screen();
}

//====================================================================================================================
// 串口格式化输出函数
//====================================================================================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     串口格式化输出函数 (类似 printf)
// 参数说明     uart_index    串口号
// 参数说明     format        格式化字符串
// 参数说明     ...           可变参数
// 返回参数     void
// 使用示例     uart_printf(UART_0, "value=%d", 100);
//-------------------------------------------------------------------------------------------------------------------
void uart_printf(uart_index_enum uart_index, const char *format, ...)
{
  char uart_printf_buf[256];  // 格式化输出缓冲区
  va_list arg;                // 可变参数列表
  
  va_start(arg, format);      // 开始可变参数
  vsnprintf(uart_printf_buf, sizeof(uart_printf_buf), format, arg);
  va_end(arg);                // 结束可变参数
  
  uart_write_string(uart_index, uart_printf_buf);
}
