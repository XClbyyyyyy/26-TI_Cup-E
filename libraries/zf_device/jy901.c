/*********************************************************************************************************************
* 本文件基于逐飞科技 MSPM0G3519 开源库开发
* 
* 文件名称          jy901
* 备注信息          JY901姿态传感器驱动
********************************************************************************************************************/

#include "jy901.h"

//-------------------------------------------------------------------------------------------------------------------
// 全局变量定义
//-------------------------------------------------------------------------------------------------------------------
jy901_rx_struct jy901_rx;
jy901_data_struct jy901_data;
static float gyro_z_bias = 1.0f;  // Z轴零漂(°/s)

// 滤波缓冲区
static float gyro_x_buf[10];    // X轴角速度缓冲区
static float gyro_y_buf[10];    // Y轴角速度缓冲区
static float gyro_z_buf[10];    // Z轴角速度缓冲区
static float angle_x_buf[10];   // X轴角度缓冲区
static float angle_y_buf[10];   // Y轴角度缓冲区
static float angle_z_buf[10];   // Z轴角度缓冲区
static float angle_mid_buf[3];                 // Z轴角度三帧中值滤波缓冲区(°)
static uint8 gyro_filter_index = 0;            // 角速度缓冲区索引
static uint8 angle_filter_index = 0;           // 角度缓冲区索引
static uint8 angle_mid_index = 0;              // Z轴中值滤波缓冲区索引
static uint8 angle_mid_count = 0;              // 已收到的Z轴中值滤波数据数量
static uint8 angle_filter_count = 0;           // Number of valid angle frames in the average filter
static float angle_raw = 0.0f;                 // 滤波后的Z轴原始角度(°)
static float angle_zero = 0.0f;                // 待机校准得到的Z轴零点角度(°)
static float angle_drift_offset = 0.0f;         // Z轴累计角度零漂补偿量(单位：°)。
static uint8 angle_valid = 0;                  // Z轴零点有效标志: 0=未校零, 1=已校零

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     JY901 初始化
//-------------------------------------------------------------------------------------------------------------------
void jy901_init(void)
{
  uint8 i = 0;

  jy901_rx.state = 0;      // 状态0: 等待帧头
  jy901_rx.rx_len = 0;
  jy901_rx.gyro_ready = 0;
  jy901_rx.angle_ready = 0;

  jy901_data.gyro_x = 0.0f;
  jy901_data.gyro_y = 0.0f;
  jy901_data.gyro_z = 0.0f;
  jy901_data.angle_x = 0.0f;
  jy901_data.angle_y = 0.0f;
  jy901_data.angle_z = 0.0f;

  // 初始化滤波缓冲区
  for(i = 0; i < 10; i++)
  {
    gyro_x_buf[i] = 0.0f;
    gyro_y_buf[i] = 0.0f;
    gyro_z_buf[i] = 0.0f;
    angle_x_buf[i] = 0.0f;
    angle_y_buf[i] = 0.0f;
    angle_z_buf[i] = 0.0f;
  }
  gyro_filter_index = 0;
  angle_filter_index = 0;
  angle_mid_index = 0;
  angle_mid_count = 0;
  angle_filter_count = 0;
  angle_mid_buf[0] = 0.0f;
  angle_mid_buf[1] = 0.0f;
  angle_mid_buf[2] = 0.0f;
  angle_raw = 0.0f;
  angle_zero = 0.0f;
  angle_drift_offset = 0.0f;
  angle_valid = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     JY901 接收回调函数(状态机模式)
// 状态机逻辑:
//   state=0: 等待帧头 0x55
//   state=1: 等待类型字节(0x52或0x53)
//   state=2: 接收9字节(8字节数据 + 1字节校验)
//-------------------------------------------------------------------------------------------------------------------
void jy901_rx_callback(uint32 state, void *ptr)
{
  uint8 temp_data = 0;  // 串口中断本次读到的字节
  uint8 checksum = 0;   // 当前完整帧的校验和
  uint8 i = 0;          // 帧缓冲拷贝索引

  if(state == UART_INTERRUPT_STATE_RX)
  {
    if(uart_query_byte(UART_4, &temp_data) == 1)
    {
      if(jy901_rx.state == 0)  // 状态0: 等待帧头
      {
        if(temp_data == 0x55)
          jy901_rx.state = 1;  // 收到帧头后等待类型
      }
      else if(jy901_rx.state == 1)  // 状态1: 等待帧类型
      {
        if((temp_data == 0x52) || (temp_data == 0x53))
        {
          jy901_rx.data_type = temp_data;
          jy901_rx.rx_len = 0;
          jy901_rx.state = 2;  // 仅接收角速度和欧拉角帧
        }
        else
          jy901_rx.state = 0;  // 非目标类型直接重新寻找帧头
      }
      else  // 状态2: 接收8字节数据和1字节校验
      {
        jy901_rx.rx_buf[jy901_rx.rx_len] = temp_data;
        jy901_rx.rx_len++;

        if(jy901_rx.rx_len >= 9)
        {
          checksum = 0x55 + jy901_rx.data_type;
          for(i = 0; i < 8; i++)
            checksum += jy901_rx.rx_buf[i];

          if(checksum == jy901_rx.rx_buf[8])
          {
            if(jy901_rx.data_type == 0x52)
            {
              if(jy901_rx.gyro_ready == 0)
              {
                for(i = 0; i < 9; i++)
                  jy901_rx.gyro_buf[i] = jy901_rx.rx_buf[i];  // 复制已校验的角速度帧

                jy901_rx.gyro_ready = 1;  // 缓冲写完后再通知主循环解析
              }
            }
            else if(jy901_rx.data_type == 0x53)
            {
              if(jy901_rx.angle_ready == 0)
              {
                for(i = 0; i < 9; i++)
                  jy901_rx.angle_buf[i] = jy901_rx.rx_buf[i];  // 复制已校验的欧拉角帧

                jy901_rx.angle_ready = 1;  // 缓冲写完后再通知主循环解析
              }
            }
          }
          jy901_rx.state = 0;  // 当前帧处理完成，等待下一个帧头
        }
      }
    }
  }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算平均值
// 参数说明     buf       数据缓冲区
// 返回参数     平均值
//-------------------------------------------------------------------------------------------------------------------
static float jy901_calc_average(float *buf)
{
  float sum = 0.0f;
  uint8 i = 0;

  for(i = 0; i < 10; i++)
  {
    sum += buf[i];
  }

  return sum / 10;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算三帧角度的中值
// 参数说明     buf       三帧Z轴角度缓冲区(°)
// 返回参数     单帧跳变过滤后的Z轴角度(°)
//-------------------------------------------------------------------------------------------------------------------
static float jy901_calc_median(float *buf)
{
  float min_value = buf[0];  // 三帧角度中的最小值(°)
  float max_value = buf[0];  // 三帧角度中的最大值(°)
  uint8 i = 0;               // 三帧角度缓冲区索引

  for(i = 1; i < 3; i++)
  {
    if(buf[i] < min_value)
      min_value = buf[i];

    if(buf[i] > max_value)
      max_value = buf[i];
  }

  return buf[0] + buf[1] + buf[2] - min_value - max_value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析角速度数据
// 参数说明     data      数据缓冲区
//-------------------------------------------------------------------------------------------------------------------
static void jy901_parse_gyro(uint8 *data)
{
  int16 raw_x = 0;
  int16 raw_y = 0;
  int16 raw_z = 0;
  float temp_x = 0.0f;
  float temp_y = 0.0f;
  float temp_z = 0.0f;

  raw_x = (int16)((uint16)data[1] << 8 | data[0]);
  raw_y = (int16)((uint16)data[3] << 8 | data[2]);
  raw_z = (int16)((uint16)data[5] << 8 | data[4]);

  // 转换为实际值,保留两位小数(单位: °/s)
  temp_x = (float)raw_x * 2000.0f / 32768.0f;
  temp_y = (float)raw_y * 2000.0f / 32768.0f;
  temp_z = (float)raw_z * 2000.0f / 32768.0f - 1.0f;

  // 存入缓冲区
  gyro_x_buf[gyro_filter_index] = temp_x;
  gyro_y_buf[gyro_filter_index] = temp_y;
  gyro_z_buf[gyro_filter_index] = temp_z;

  // 更新索引
  gyro_filter_index = (gyro_filter_index + 1) % 10;

  // 计算平均值
  jy901_data.gyro_x = jy901_calc_average(gyro_x_buf);
  jy901_data.gyro_y = jy901_calc_average(gyro_y_buf);
  jy901_data.gyro_z = jy901_calc_average(gyro_z_buf);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析角度数据
// 参数说明     data      数据缓冲区
//-------------------------------------------------------------------------------------------------------------------
static void jy901_parse_angle(uint8 *data)
{
  int16 raw_x = 0;
  int16 raw_y = 0;
  int16 raw_z = 0;
  float temp_x = 0.0f;
  float temp_y = 0.0f;
  float temp_z = 0.0f;

  raw_x = (int16)((uint16)data[1] << 8 | data[0]);
  raw_y = (int16)((uint16)data[3] << 8 | data[2]);
  raw_z = (int16)((uint16)data[5] << 8 | data[4]);

  // 转换为实际值,保留两位小数(单位: °)
  temp_x = (float)raw_x * 180.0f / 32768.0f;
  temp_y = (float)raw_y * 180.0f / 32768.0f;
  temp_z = (float)raw_z * 180.0f / 32768.0f;

  // 先进行三帧中值滤波，丢弃单帧异常跳变，避免误触发85°转弯退出条件
  angle_mid_buf[angle_mid_index] = temp_z;
  angle_mid_index = (angle_mid_index + 1) % 3;

  if(angle_mid_count < 3)
    angle_mid_count++;

  if(angle_mid_count == 3)
    temp_z = jy901_calc_median(angle_mid_buf);

  // 存入缓冲区
  angle_x_buf[angle_filter_index] = temp_x;
  angle_y_buf[angle_filter_index] = temp_y;
  angle_z_buf[angle_filter_index] = temp_z;

  // 更新索引
  angle_filter_index = (angle_filter_index + 1) % 10;
  if(angle_filter_count < 10)
    angle_filter_count++;

  // 计算平均值
  jy901_data.angle_x = jy901_calc_average(angle_x_buf);
  jy901_data.angle_y = jy901_calc_average(angle_y_buf);
  angle_raw = jy901_calc_average(angle_z_buf);  // 保留滤波后的原始Z轴角度，供待机校零使用

  if(angle_valid == 0 && angle_filter_count == 10)
    jy901_zero_angle();  // Set the initial zero after the average filter is full.

  if(angle_valid == 1)
  {
    jy901_data.angle_z = angle_raw - angle_zero - angle_drift_offset;  // 累计补偿Z轴正向角度零漂。
    if(jy901_data.angle_z > 180.0f)
      jy901_data.angle_z -= 360.0f;  // 处理+180°边界环绕
    else if(jy901_data.angle_z < -180.0f)
      jy901_data.angle_z += 360.0f;  // 处理-180°边界环绕
    angle_drift_offset += 0.001058f;  // JY901为100Hz时，每帧固定累计补偿0.001058°。
  }
  else
    jy901_data.angle_z = angle_raw;  // 尚未校零时保留原始角度
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将当前Z轴原始角度设为零点
// 备注信息     待机时持续调用以抑制静止零漂，开始运动后不再调用以固定零点。
//-------------------------------------------------------------------------------------------------------------------
void jy901_zero_angle(void)
{
  angle_zero = angle_raw;
  angle_drift_offset = 0.0f;  // 从当前零点开始重新累计角度零漂补偿。
  angle_valid = 1;
  jy901_data.angle_z = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     JY901 数据处理函数
//-------------------------------------------------------------------------------------------------------------------
void jy901_process_data(void)
{
  uint8 gyro_data[9];   // 从中断缓冲复制的角速度帧
  uint8 angle_data[9];  // 从中断缓冲复制的欧拉角帧
  uint8 i = 0;          // 帧缓冲复制索引

  if(jy901_rx.gyro_ready == 1)
  {
    for(i = 0; i < 9; i++)
      gyro_data[i] = jy901_rx.gyro_buf[i];  // 先拷贝到本地，防止中断继续写入时覆盖

    jy901_rx.gyro_ready = 0;  // 本地复制完成后允许中断接收下一帧
    jy901_parse_gyro(gyro_data);
  }

  if(jy901_rx.angle_ready == 1)
  {
    for(i = 0; i < 9; i++)
      angle_data[i] = jy901_rx.angle_buf[i];  // 先拷贝到本地，防止中断继续写入时覆盖

    jy901_rx.angle_ready = 0;  // 本地复制完成后允许中断接收下一帧
    jy901_parse_angle(angle_data);
  }
}
