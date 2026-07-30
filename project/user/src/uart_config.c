/*********************************************************************************************************************
* 本文件基于逐飞科技 MSPM0G3519 开源库开发
* 
* 文件名称          uart_config
* 备注信息          串口配置（UART0、UART1、UART3、UART4）
********************************************************************************************************************/

#include "uart_config.h"
#include "zf_device_stepper_motor.h"

//-------------------------------------------------------------------------------------------------------------------
// 接收结构体定义
//-------------------------------------------------------------------------------------------------------------------
uart_rx_struct uart0_rx;  // UART0 接收结构体 - 串口调试
uart_rx_struct uart1_rx;  // UART1 接收结构体 - 摄像头
uart_rx_struct uart3_rx;  // UART3 接收结构体 - 步进电机
uart_rx_struct uart4_rx;  // UART4 接收结构体 - 串口屏

camera_data_struct camera_data[4];          // 已完成校验的摄像头连续执行计划，按 piece_id 升序排列。
uint32 camera_plan_sequence = 0;             // 已完成校验计划的确认编号。
float camera_rectangle_width_mm = 0.0F;      // 已完成校验计划的严格矩形宽度，单位：mm。
float camera_rectangle_height_mm = 0.0F;     // 已完成校验计划的严格矩形高度，单位：mm。
uint8 camera_piece_count = 0;                // 已完成校验计划中的碎片数量，取值范围：0-4。
uint8 camera_plan_ready_flag = 0;            // 已完成校验计划就绪标志，0=无新计划，1=有新计划。

static uint8 camera_line_queue[8][128];      // UART1 接收行队列，最多缓存 8 行、每行最多 127 个 ASCII 字符。
static uint8 camera_line_length[8];          // UART1 接收行队列中每一行的有效字节数。
static volatile uint8 camera_line_write_index; // UART1 接收中断写入行队列使用的下标，取值范围：0-7。
static volatile uint8 camera_line_read_index;  // 主循环读取行队列使用的下标，取值范围：0-7。
static volatile uint8 camera_line_count;        // 行队列中的待处理行数，取值范围：0-8。
static camera_data_struct camera_plan_buffer[4]; // 当前暂存批次的碎片计划，按 piece_id 索引。
static uint8 camera_piece_received_flag[4];      // 当前暂存批次中各碎片的接收标志，0=未接收，1=已接收。
static uint32 camera_pending_sequence;            // 当前暂存批次的确认编号。
static float camera_pending_rectangle_width_mm;   // 当前暂存批次的严格矩形宽度，单位：mm。
static float camera_pending_rectangle_height_mm;  // 当前暂存批次的严格矩形高度，单位：mm。
static uint8 camera_pending_piece_count;          // 当前暂存批次已经接收的不同碎片数量，取值范围：0-4。
static uint8 camera_pending_active_flag;           // 当前暂存批次有效标志，0=无有效 PLAN，1=已收到有效 PLAN。

volatile uint8 state = 0;   // 运行状态: 0=待机, 1=小车运动, 2=镜头运动, 3=两者, 4=其他；UART4 中断可修改。
uint8 last_state=0;         // 上一次运行状态
uint16 XYspeed = 1200;      // X、Y 轴基础速度，单位：RPM。
uint16 Tspeed = 300;        // T 轴电机轴基础速度，单位：RPM。
uint8 direction = 0;         // 无线串口 I1 设置的 Y 轴方向：0=负向，1=正向。
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
// 函数简介     UART0 接收回调函数（无线串口 ASCII 指令）
// 状态机逻辑：
//   state=0：等待帧头字符 'I'。
//   state=1：接收指令内容，收到换行符 '\n' 后置接收完成标志。
// 指令格式：
//   I1:<0或1>\n：设置 Y 轴方向。
//   I0:<脉冲数>\n：按当前方向驱动 Y 轴。
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
        if(temp_data == 'I')
        {
          uart0_rx.rx_buf[0] = temp_data;  // 保存指令帧头。
          uart0_rx.rx_len = 1;
          uart0_rx.state = 1;  // 开始接收指令剩余字符。
        }
      }
      else
      {
        if(temp_data == '\n')
        {
          uart0_rx.frame_ready = 1;  // 指令帧接收完成。
          uart0_rx.state = 0;
        }
        else if(uart0_rx.rx_len < 255)
        {
          uart0_rx.rx_buf[uart0_rx.rx_len] = temp_data;
          uart0_rx.rx_len++;
        }
        else
        {
          uart0_rx.rx_len = 0;  // 指令超出接收缓冲区，丢弃当前帧。
          uart0_rx.state = 0;
        }
      }
    }
  }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析无线串口 ASCII 指令并执行 Y 轴控制
// 备注信息     I1 仅接受方向值 0 或 1；I0 接受无符号十进制脉冲数。
//-------------------------------------------------------------------------------------------------------------------
static void uart0_parse_wireless_command(void)
{
  uint8 input_index = 0;       // 无线串口输入编号，仅支持 0 和 1。
  uint8 character_index = 0;   // 当前解析的字符下标。
  uint8 digit = 0;             // 当前十进制数字。
  uint8 value_valid_flag = 1;  // 指令数值是否在 uint32 有效范围内：1=有效，0=无效。
  uint32 pulse_count = 0;      // I0 指令中的 Y 轴运动脉冲数。

  if(uart0_rx.rx_len < 4)
    return;  // 最短有效指令为 I0:0。

  if(uart0_rx.rx_buf[0] != 'I' || uart0_rx.rx_buf[2] != ':')
    return;  // 帧头或分隔符不符合协议。

  if(uart0_rx.rx_buf[1] != '0' && uart0_rx.rx_buf[1] != '1')
    return;  // 仅支持 I0 和 I1 两个输入编号。

  input_index = uart0_rx.rx_buf[1] - '0';

  for(character_index = 3; character_index < uart0_rx.rx_len; character_index++)
  {
    if(uart0_rx.rx_buf[character_index] < '0' || uart0_rx.rx_buf[character_index] > '9')
    {
      value_valid_flag = 0;
      break;
    }

    digit = uart0_rx.rx_buf[character_index] - '0';
    if(pulse_count > 429496729U || (pulse_count == 429496729U && digit > 5U))
    {
      value_valid_flag = 0;
      break;
    }

    pulse_count = pulse_count * 10U + digit;
  }

  if(value_valid_flag == 0)
    return;  // 数值包含非数字字符或超出 uint32 范围。

  if(input_index == 1)
  {
    if(pulse_count == 0U || pulse_count == 1U)
      direction = (uint8)pulse_count;
  }
  else if(pulse_count > 0U)
  {
    stepper_pos_control(STEPPER_ADDR_Y, direction, 1200, 0, pulse_count, 0);
  }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART0 数据处理函数
// 备注信息     处理以换行结束的无线串口 ASCII 指令。
//-------------------------------------------------------------------------------------------------------------------
void uart0_process_data(void)
{
  if(uart0_rx.frame_ready == 1)
  {
    uart_printf(UART_0, "Receive\r\n");  // 接收到数据回传确认。
    uart0_parse_wireless_command();
    uart0_rx.frame_ready = 0;
  }
}
//====================================================================================================================
// UART1 - 摄像头
//====================================================================================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART1 初始化，配置为摄像头计划协议的 921600 波特率串口。
//-------------------------------------------------------------------------------------------------------------------
void uart1_init_camera(void)
{
  uart_init(UART_1, 921600, UART1_TX_PIN, UART1_RX_PIN);
  uart_set_callback(UART_1, uart1_rx_callback, NULL);
  uart_set_interrupt_config(UART_1, UART_INTERRUPT_CONFIG_RX_ENABLE);
  NVIC_SetPriority(UART1_INT_IRQn, 0);

  uart1_rx.state = 0;        // 状态0：接收当前行；状态1：丢弃超长行直到换行符。
  uart1_rx.rx_len = 0;       // 当前接收长度清零。
  uart1_rx.frame_ready = 0;  // 当前无完整帧。
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     比较一行协议数据是否以指定关键字开头。
// 参数说明     line         待比较的 ASCII 行。
// 参数说明     line_length  待比较的 ASCII 行长度。
// 参数说明     text         需要匹配的关键字。
// 返回参数     1=匹配，0=不匹配。
//-------------------------------------------------------------------------------------------------------------------
static uint8 camera_line_starts_with(const uint8 *line, uint8 line_length, const char *text)
{
  uint8 character_index = 0;  // 当前比较的字符下标。

  while(text[character_index] != '\0')
  {
    if(character_index >= line_length)
      return 0;

    if(line[character_index] != (uint8)text[character_index])
      return 0;

    character_index++;
  }

  return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从 ASCII 字段读取无符号十进制整数，读取后下标停在分隔符或行尾。
// 参数说明     line         待解析的 ASCII 行。
// 参数说明     line_length  待解析的 ASCII 行长度。
// 参数说明     index_ptr    输入输出参数，当前字段起始下标和解析结束下标。
// 参数说明     value_ptr    输出参数，解析得到的 uint32 数值。
// 返回参数     1=解析成功，0=字段格式或范围错误。
//-------------------------------------------------------------------------------------------------------------------
static uint8 camera_read_uint32(const uint8 *line, uint8 line_length, uint8 *index_ptr, uint32 *value_ptr)
{
  uint8 digit_count = 0;  // 当前字段中已经读取的十进制数字数量。
  uint8 digit = 0;        // 当前读取到的单个十进制数字。

  *value_ptr = 0;
  while(*index_ptr < line_length)
  {
    if(line[*index_ptr] < '0' || line[*index_ptr] > '9')
      break;

    digit = line[*index_ptr] - '0';
    if(*value_ptr > 429496729U || (*value_ptr == 429496729U && digit > 5U))
      return 0;

    *value_ptr = *value_ptr * 10U + digit;
    (*index_ptr)++;
    digit_count++;
  }

  if(digit_count == 0)
    return 0;

  return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从 ASCII 字段读取十进制浮点数，支持可选正负号和小数点。
// 参数说明     line         待解析的 ASCII 行。
// 参数说明     line_length  待解析的 ASCII 行长度。
// 参数说明     index_ptr    输入输出参数，当前字段起始下标和解析结束下标。
// 参数说明     value_ptr    输出参数，解析得到的浮点数。
// 返回参数     1=解析成功，0=字段格式错误。
//-------------------------------------------------------------------------------------------------------------------
static uint8 camera_read_float(const uint8 *line, uint8 line_length, uint8 *index_ptr, float *value_ptr)
{
  uint8 digit_count = 0;       // 当前字段中已经读取的十进制数字数量。
  uint8 decimal_flag = 0;      // 小数点是否已经出现，0=未出现，1=已经出现。
  float sign = 1.0F;           // 当前数值的正负号。
  float value = 0.0F;          // 当前累积得到的无符号数值。
  float decimal_scale = 0.1F;  // 小数部分当前数字的权重。

  if(*index_ptr < line_length && line[*index_ptr] == '-')
  {
    sign = -1.0F;
    (*index_ptr)++;
  }
  else if(*index_ptr < line_length && line[*index_ptr] == '+')
    (*index_ptr)++;

  while(*index_ptr < line_length)
  {
    if(line[*index_ptr] == '.')
    {
      if(decimal_flag == 1)
        return 0;

      decimal_flag = 1;
      (*index_ptr)++;
    }
    else if(line[*index_ptr] >= '0' && line[*index_ptr] <= '9')
    {
      if(decimal_flag == 0)
        value = value * 10.0F + (float)(line[*index_ptr] - '0');
      else
      {
        value += (float)(line[*index_ptr] - '0') * decimal_scale;
        decimal_scale *= 0.1F;
      }

      (*index_ptr)++;
      digit_count++;
    }
    else
      break;
  }

  if(digit_count == 0)
    return 0;

  *value_ptr = sign * value;
  return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查并跳过一个字段分隔逗号。
// 参数说明     line         待检查的 ASCII 行。
// 参数说明     line_length  待检查的 ASCII 行长度。
// 参数说明     index_ptr    输入输出参数，当前下标和跳过逗号后的下标。
// 返回参数     1=存在并跳过逗号，0=缺少逗号。
//-------------------------------------------------------------------------------------------------------------------
static uint8 camera_skip_comma(const uint8 *line, uint8 line_length, uint8 *index_ptr)
{
  if(*index_ptr >= line_length || line[*index_ptr] != ',')
    return 0;

  (*index_ptr)++;
  return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     开始新的摄像头暂存批次，覆盖同编号重传或未完成的旧批次。
// 状态机逻辑     收到有效 PLAN 后进入状态1，后续仅接收相同 sequence 的 PIECE 和 DONE。
// 参数说明     sequence             当前计划确认编号。
// 参数说明     rectangle_width_mm   严格矩形宽度，单位：mm。
// 参数说明     rectangle_height_mm  严格矩形高度，单位：mm。
//-------------------------------------------------------------------------------------------------------------------
static void camera_start_plan(uint32 sequence, float rectangle_width_mm, float rectangle_height_mm)
{
  uint8 piece_index = 0;  // 初始化暂存碎片标志时使用的数组下标。

  camera_pending_sequence = sequence;
  camera_pending_rectangle_width_mm = rectangle_width_mm;
  camera_pending_rectangle_height_mm = rectangle_height_mm;
  camera_pending_piece_count = 0;
  camera_pending_active_flag = 1;  // 状态1：已收到 PLAN，等待同编号 PIECE 和 DONE。
  camera_plan_ready_flag = 0;      // 新批次开始后，不允许继续使用上一批次的数据。

  for(piece_index = 0; piece_index < 4; piece_index++)
    camera_piece_received_flag[piece_index] = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析 PLAN 行并建立暂存批次。
// 返回参数     1=PLAN 有效，0=PLAN 格式或范围错误。
//-------------------------------------------------------------------------------------------------------------------
static uint8 camera_parse_plan(const uint8 *line, uint8 line_length)
{
  uint8 character_index = 5;         // 跳过 "PLAN," 后的当前字符下标。
  uint32 sequence = 0;               // 本行解析得到的计划确认编号。
  float rectangle_width_mm = 0.0F;   // 本行解析得到的严格矩形宽度，单位：mm。
  float rectangle_height_mm = 0.0F;  // 本行解析得到的严格矩形高度，单位：mm。

  if(camera_read_uint32(line, line_length, &character_index, &sequence) == 0)
    return 0;

  if(camera_skip_comma(line, line_length, &character_index) == 0)
    return 0;

  if(camera_read_float(line, line_length, &character_index, &rectangle_width_mm) == 0)
    return 0;

  if(camera_skip_comma(line, line_length, &character_index) == 0)
    return 0;

  if(camera_read_float(line, line_length, &character_index, &rectangle_height_mm) == 0)
    return 0;

  if(character_index != line_length)
    return 0;

  if(rectangle_width_mm <= 0.0F || rectangle_height_mm <= 0.0F)
    return 0;

  camera_start_plan(sequence, rectangle_width_mm, rectangle_height_mm);
  return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析 PIECE 行并暂存同编号碎片计划，重复 piece_id 覆盖旧数据。
// 返回参数     1=PIECE 有效，0=PIECE 格式、编号或状态错误。
//-------------------------------------------------------------------------------------------------------------------
static uint8 camera_parse_piece(const uint8 *line, uint8 line_length)
{
  uint8 character_index = 6;      // 跳过 "PIECE," 后的当前字符下标。
  uint32 sequence = 0;            // 本行解析得到的计划确认编号。
  uint32 piece_id = 0;            // 本行解析得到的碎片编号。
  camera_data_struct piece_data;  // 本行解析得到的完整碎片计划。

  if(camera_read_uint32(line, line_length, &character_index, &sequence) == 0)
    return 0;

  if(camera_skip_comma(line, line_length, &character_index) == 0)
    return 0;

  if(camera_read_uint32(line, line_length, &character_index, &piece_id) == 0)
    return 0;

  if(piece_id >= 4U || camera_pending_active_flag == 0 || sequence != camera_pending_sequence)
    return 0;

  if(camera_skip_comma(line, line_length, &character_index) == 0)
    return 0;

  if(camera_read_float(line, line_length, &character_index, &piece_data.source_x_mm) == 0)
    return 0;

  if(camera_skip_comma(line, line_length, &character_index) == 0)
    return 0;

  if(camera_read_float(line, line_length, &character_index, &piece_data.source_y_mm) == 0)
    return 0;

  if(camera_skip_comma(line, line_length, &character_index) == 0)
    return 0;

  if(camera_read_float(line, line_length, &character_index, &piece_data.target_x_mm) == 0)
    return 0;

  if(camera_skip_comma(line, line_length, &character_index) == 0)
    return 0;

  if(camera_read_float(line, line_length, &character_index, &piece_data.target_y_mm) == 0)
    return 0;

  if(camera_skip_comma(line, line_length, &character_index) == 0)
    return 0;

  if(camera_read_float(line, line_length, &character_index, &piece_data.rotation_deg) == 0)
    return 0;

  if(character_index != line_length)
    return 0;

  piece_data.piece_id = (uint8)piece_id;
  piece_data.take_move_x_mm = piece_data.source_x_mm;  // 第一块默认从机械原点出发，完整计划提交后会更新为连续差分位移。
  piece_data.take_move_y_mm = piece_data.source_y_mm;
  piece_data.put_move_x_mm = piece_data.target_x_mm - piece_data.source_x_mm;
  piece_data.put_move_y_mm = piece_data.target_y_mm - piece_data.source_y_mm;

  if(camera_piece_received_flag[piece_id] == 0)
    camera_pending_piece_count++;

  camera_plan_buffer[piece_id] = piece_data;
  camera_piece_received_flag[piece_id] = 1;
  return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将相对机械原点的距离换算为带方向的电机脉冲值。
// 参数说明     distance_mm  相对机械原点或上一位置的距离，单位：mm。
// 返回参数     带方向的脉冲值，X/Y 轴均为 80 脉冲/mm。
//-------------------------------------------------------------------------------------------------------------------
static int32 camera_mm_to_pulse(float distance_mm)
{
  if(distance_mm < 0.0F)
    return -(int32)(-distance_mm * 80.0F + 0.5F);

  return (int32)(distance_mm * 80.0F + 0.5F);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按 piece_id 升序生成连续 XY 运动数据。
// 状态机逻辑     第 0 块从原点移动到源碎片；后续碎片从上一块目标位置移动到当前源碎片。
//-------------------------------------------------------------------------------------------------------------------
static void camera_build_continuous_motion_data(uint8 piece_count)
{
  uint8 piece_index = 0;             // 当前按执行顺序处理的碎片下标。
  float previous_target_x_mm = 0.0F; // 上一块目标位置相对 X 原点的距离，单位：mm。
  float previous_target_y_mm = 0.0F; // 上一块目标位置相对 Y 原点的距离，单位：mm。
  int32 previous_target_x_pulse = 0; // 上一块目标位置相对 X 原点的脉冲坐标。
  int32 previous_target_y_pulse = 0; // 上一块目标位置相对 Y 原点的脉冲坐标。
  int32 source_x_pulse = 0;          // 当前源碎片相对 X 原点的脉冲坐标。
  int32 source_y_pulse = 0;          // 当前源碎片相对 Y 原点的脉冲坐标。
  int32 target_x_pulse = 0;          // 当前目标碎片相对 X 原点的脉冲坐标。
  int32 target_y_pulse = 0;          // 当前目标碎片相对 Y 原点的脉冲坐标。
  int32 move_x_pulse = 0;            // 当前待执行 X 轴带方向脉冲值。
  int32 move_y_pulse = 0;            // 当前待执行 Y 轴带方向脉冲值。

  for(piece_index = 0; piece_index < piece_count; piece_index++)
  {
    source_x_pulse = camera_mm_to_pulse(camera_data[piece_index].source_x_mm);
    source_y_pulse = camera_mm_to_pulse(camera_data[piece_index].source_y_mm);
    target_x_pulse = camera_mm_to_pulse(camera_data[piece_index].target_x_mm);
    target_y_pulse = camera_mm_to_pulse(camera_data[piece_index].target_y_mm);

    camera_data[piece_index].take_move_x_mm = camera_data[piece_index].source_x_mm - previous_target_x_mm;
    camera_data[piece_index].take_move_y_mm = camera_data[piece_index].source_y_mm - previous_target_y_mm;
    camera_data[piece_index].put_move_x_mm = camera_data[piece_index].target_x_mm - camera_data[piece_index].source_x_mm;
    camera_data[piece_index].put_move_y_mm = camera_data[piece_index].target_y_mm - camera_data[piece_index].source_y_mm;

    move_x_pulse = source_x_pulse - previous_target_x_pulse;
    if(move_x_pulse < 0)
    {
      camera_data[piece_index].Dir_x = 0;
      camera_data[piece_index].take_move_x_pulse = (uint32)(-move_x_pulse);
    }
    else
    {
      camera_data[piece_index].Dir_x = 1;
      camera_data[piece_index].take_move_x_pulse = (uint32)move_x_pulse;
    }

    move_y_pulse = source_y_pulse - previous_target_y_pulse;
    if(move_y_pulse < 0)
    {
      camera_data[piece_index].Dir_y = 0;
      camera_data[piece_index].take_move_y_pulse = (uint32)(-move_y_pulse);
    }
    else
    {
      camera_data[piece_index].Dir_y = 1;
      camera_data[piece_index].take_move_y_pulse = (uint32)move_y_pulse;
    }

    move_x_pulse = target_x_pulse - source_x_pulse;
    if(move_x_pulse < 0)
    {
      camera_data[piece_index].put_dir_x = 0;
      camera_data[piece_index].put_move_x_pulse = (uint32)(-move_x_pulse);
    }
    else
    {
      camera_data[piece_index].put_dir_x = 1;
      camera_data[piece_index].put_move_x_pulse = (uint32)move_x_pulse;
    }

    move_y_pulse = target_y_pulse - source_y_pulse;
    if(move_y_pulse < 0)
    {
      camera_data[piece_index].put_dir_y = 0;
      camera_data[piece_index].put_move_y_pulse = (uint32)(-move_y_pulse);
    }
    else
    {
      camera_data[piece_index].put_dir_y = 1;
      camera_data[piece_index].put_move_y_pulse = (uint32)move_y_pulse;
    }

    previous_target_x_pulse = target_x_pulse;
    previous_target_y_pulse = target_y_pulse;
    previous_target_x_mm = camera_data[piece_index].target_x_mm;
    previous_target_y_mm = camera_data[piece_index].target_y_mm;
  }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析 DONE 行并提交已完成的同编号计划，随后回发 ACK。
// 状态机逻辑     状态1收到有效 DONE 后提交计划并回到状态0，等待下一条 PLAN。
// 返回参数     1=DONE 有效并已提交，0=DONE 格式、编号或状态错误。
//-------------------------------------------------------------------------------------------------------------------
static uint8 camera_parse_done(const uint8 *line, uint8 line_length)
{
  uint8 character_index = 5;  // 跳过 "DONE," 后的当前字符下标。
  uint8 piece_index = 0;      // 遍历按 piece_id 暂存的碎片计划时使用的数组下标。
  uint8 execution_index = 0;  // 按 piece_id 升序写入连续执行计划时使用的数组下标。
  uint32 sequence = 0;        // 本行解析得到的计划确认编号。

  if(camera_read_uint32(line, line_length, &character_index, &sequence) == 0)
    return 0;

  if(character_index != line_length)
    return 0;

  if(camera_pending_active_flag == 0 || sequence != camera_pending_sequence || camera_pending_piece_count == 0)
    return 0;

  for(piece_index = 0; piece_index < 4; piece_index++)
  {
    if(camera_piece_received_flag[piece_index] == 1)
    {
      camera_data[execution_index] = camera_plan_buffer[piece_index];
      execution_index++;
    }
  }

  camera_build_continuous_motion_data(execution_index);

  camera_plan_sequence = camera_pending_sequence;
  camera_rectangle_width_mm = camera_pending_rectangle_width_mm;
  camera_rectangle_height_mm = camera_pending_rectangle_height_mm;
  camera_piece_count = execution_index;
  camera_plan_ready_flag = 1;  // 已收到完整批次，可由机械臂执行层读取。
  camera_pending_active_flag = 0;  // 状态0：当前批次已经提交，等待下一条 PLAN。
  uart_printf(UART_1, "ACK,%u\n", camera_plan_sequence);
  return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据行关键字分派 PLAN、PIECE 和 DONE 的解析。
// 参数说明     line         待处理的完整 ASCII 协议行，不包含换行符。
// 参数说明     line_length  待处理行的有效字节数。
//-------------------------------------------------------------------------------------------------------------------
static void camera_process_line(const uint8 *line, uint8 line_length)
{
  if(camera_line_starts_with(line, line_length, "PLAN,") == 1)
    camera_parse_plan(line, line_length);
  else if(camera_line_starts_with(line, line_length, "PIECE,") == 1)
    camera_parse_piece(line, line_length);
  else if(camera_line_starts_with(line, line_length, "DONE,") == 1)
    camera_parse_done(line, line_length);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART1 接收回调函数，将完整 ASCII 行写入队列供主循环解析。
// 状态机逻辑     状态0累积当前行；状态1丢弃超长行；收到换行符后回到状态0。
//-------------------------------------------------------------------------------------------------------------------
void uart1_rx_callback(uint32 state, void *ptr)
{
  uint8 temp_data = 0;  // 当前从 UART1 接收 FIFO 读取的字节。
  uint8 character_index = 0;  // 复制完整行到队列时使用的字符下标。

  (void)ptr;
  if(state != UART_INTERRUPT_STATE_RX)
    return;

  while(uart_query_byte(UART_1, &temp_data) == 1)
  {
    if(temp_data == '\r')
      continue;

    if(temp_data == '\n')
    {
      if(uart1_rx.state == 0 && uart1_rx.rx_len > 0 && camera_line_count < 8)
      {
        for(character_index = 0; character_index < uart1_rx.rx_len; character_index++)
          camera_line_queue[camera_line_write_index][character_index] = uart1_rx.rx_buf[character_index];

        camera_line_length[camera_line_write_index] = uart1_rx.rx_len;
        camera_line_write_index++;
        if(camera_line_write_index >= 8)
          camera_line_write_index = 0;

        camera_line_count++;
      }

      uart1_rx.state = 0;   // 新行从状态0开始接收。
      uart1_rx.rx_len = 0;
    }
    else if(uart1_rx.state == 0)
    {
      if(uart1_rx.rx_len < 127)
      {
        uart1_rx.rx_buf[uart1_rx.rx_len] = temp_data;
        uart1_rx.rx_len++;
      }
      else
        uart1_rx.state = 1;  // 状态1：当前行超长，丢弃到下一个换行符。
    }
  }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART1 数据处理函数，从行队列取出并解析摄像头计划协议。
// 状态机逻辑     仅当 PLAN、同编号 PIECE 和同编号 DONE 全部校验通过后，才更新 camera_data 并发送 ACK。
//-------------------------------------------------------------------------------------------------------------------
void uart1_process_data(void)
{
  uint8 line[128];        // 从中断行队列取出的本地 ASCII 行副本。
  uint8 line_length = 0;  // 本地 ASCII 行副本的有效字节数。
  uint8 character_index = 0;  // 复制行队列数据时使用的字符下标。

  while(1)
  {
    __disable_irq();
    if(camera_line_count == 0)
    {
      __enable_irq();
      return;
    }

    line_length = camera_line_length[camera_line_read_index];
    for(character_index = 0; character_index < line_length; character_index++)
      line[character_index] = camera_line_queue[camera_line_read_index][character_index];

    camera_line_read_index++;
    if(camera_line_read_index >= 8)
      camera_line_read_index = 0;

    camera_line_count--;
    __enable_irq();

    camera_process_line(line, line_length);
  }
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
// UART5 - 预留接收回调
//====================================================================================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART5 接收回调预留函数
// 备注信息     当前不处理 UART5 接收数据。
//-------------------------------------------------------------------------------------------------------------------
void uart5_rx_callback(uint32 state, void *ptr)
{
}

//====================================================================================================================
// UART4 - 串口屏
//====================================================================================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART4 串口屏初始化
//-------------------------------------------------------------------------------------------------------------------
void uart4_init_screen(void)
{
  uart_init(UART_4, 230400, UART4_TX_PIN, UART4_RX_PIN);
  uart_set_callback(UART_4, uart4_rx_callback, NULL);
  uart_set_interrupt_config(UART_4, UART_INTERRUPT_CONFIG_RX_ENABLE);
  NVIC_SetPriority(UART4_INT_IRQn, 2);

  uart4_rx.state = 0;  // 状态0: 等待帧头
  uart4_rx.rx_len = 0;
  uart4_rx.frame_ready = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART4 串口屏接收回调函数 (固定四字节帧)
// 状态机逻辑:
//   state=0: 等待帧头0x5B
//   state=1: 接收设置类型，1=state、2=XYspeed、3=Tspeed
//   state=2: 接收设置值；速度类型按 int8 有符号增量解析
//   state=3: 等待并验证帧尾0x5D
//-------------------------------------------------------------------------------------------------------------------
void uart4_rx_callback(uint32 interrupt_state, void *ptr)
{
  uint8 temp_data = 0;  // 临时存储接收到的字节

  if(interrupt_state == UART_INTERRUPT_STATE_RX)
  {
    if(uart_query_byte(UART_4, &temp_data) == 1)
    {
      if(uart4_rx.state == 0)
      {
        if(temp_data == 0x5B)
        {
          uart4_rx.state = 1;  // 收到帧头后等待设置类型
          uart4_rx.rx_len = 0;
        }
      }
      else if(uart4_rx.state == 1)
      {
        uart4_rx.rx_buf[0] = temp_data;  // 保存设置类型字节
        uart4_rx.rx_len = 1;
        uart4_rx.state = 2;
      }
      else if(uart4_rx.state == 2)
      {
        uart4_rx.rx_buf[1] = temp_data;  // 保存设置值字节
        uart4_rx.rx_len = 2;
        uart4_rx.state = 3;
      }
      else
      {
        if(temp_data == 0x5D)
        {
          if(uart4_rx.rx_buf[0] == 1)
          {
            last_state = state;                 // 保存切换前的运行状态。
            state = uart4_rx.rx_buf[1];         // 类型1：在完整帧接收中断内直接设置运行状态。
          }
          uart4_rx.frame_ready = 1;  // 帧头、两个数据字节和帧尾均正确
        }

        if(temp_data == 0x5B)
        {
          uart4_rx.state = 1;  // 帧尾错误但当前字节是新帧头，立即重新同步
          uart4_rx.rx_len = 0;
        }
        else
          uart4_rx.state = 0;
      }
    }
  }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART4 串口屏数据处理函数
// 备注信息     仅处理格式为 0x5B-类型-数值-0x5D 的有效固定帧。
//              类型1已在接收中断内设置 state；类型2、3将数值按 int8 加到当前速度。
//-------------------------------------------------------------------------------------------------------------------
void uart4_process_data(void)
{
  int16 speed_adjustment;  // 串口屏速度增量，取值范围：-128 至 127 RPM。

  if(uart4_rx.frame_ready == 1)
  {
    if(uart4_rx.rx_len == 2)
    {
      if(uart4_rx.rx_buf[0] == 2)
      {
        speed_adjustment = (int8)uart4_rx.rx_buf[1];
        if(speed_adjustment < 0)
        {
          if(XYspeed > (uint16)(-speed_adjustment))
            XYspeed += speed_adjustment;
          else
            XYspeed = 1;                     // 速度至少为 1 RPM，避免下溢或零速度。
        }
        else if((uint32)XYspeed + (uint16)speed_adjustment <= 65535)
          XYspeed += (uint16)speed_adjustment;
        else
          XYspeed = 65535;                   // 类型2：限制在 uint16 可表示的最大速度。
      }
      else if(uart4_rx.rx_buf[0] == 3)
      {
        speed_adjustment = (int8)uart4_rx.rx_buf[1];
        if(speed_adjustment < 0)
        {
          if(Tspeed > (uint16)(-speed_adjustment))
            Tspeed += speed_adjustment;
          else
            Tspeed = 1;                      // 速度至少为 1 RPM，避免下溢或零速度。
        }
        else if((uint32)Tspeed + (uint16)speed_adjustment <= 65535)
          Tspeed += (uint16)speed_adjustment;
        else
          Tspeed = 65535;                    // 类型3：限制在 uint16 可表示的最大速度。
      }
    }
    uart4_rx.frame_ready = 0;
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
  uart4_init_screen();
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
