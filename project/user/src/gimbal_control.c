/*********************************************************************************************************************
* 本文件基于逐飞科技 MSPM0G3519 开源库开发
* 
* 文件名称          gimbal_control
* 备注信息          云台追踪控制
********************************************************************************************************************/

#include "gimbal_control.h"

// 可调PID参数(串口发送对应值即可实时修改)
float gimbal_Kp = 1.2f;          // 锁靶模式比例系数
float gimbal_Ki = 0.001f;        // 锁靶模式积分系数
float gimbal_Kd = 0.15f;         // 锁靶模式微分系数
uint16 lock_limit = 42;          // 锁靶模式X/Y轴常规速度上限(电机指令RPM)
float circle_kp = 0.3f;          // 画圆模式比例系数
float circle_ki = 0.0f;          // 画圆模式积分系数，视觉延迟下不累积积分
float circle_kd = 0.3f;         // 画圆模式微分系数
uint16 circle_limit = 30;        // 画圆模式X/Y轴常规速度上限(电机指令RPM)
float gimbal_turn_scale = 10.0f;  // 云台速度指令与实际转速的校准倍率
uint16 gimbal_turn_limit = 100;  // 转弯补偿速度上限(电机指令RPM)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     云台寻靶扫描函数
// 参数说明     turn_rate - 小车Z轴角速度(°/s)，正值对应小车左转
// 状态机逻辑   寻靶时始终保留60RPM基础扫描速度；仅在转弯及出弯加速阶段叠加反向补偿。
//-------------------------------------------------------------------------------------------------------------------
void gimbal_search(float turn_rate)
{
  float output = 60.0f;       // X轴扫描与转弯补偿合成后的有符号速度(RPM)
  float turn_output = 0.0f;   // 小车转弯对应的X轴补偿速度(RPM)
  uint16 speed = 0;           // 要发送给X轴电机的速度绝对值(RPM)
  uint8 dir = 1;              // X轴电机方向: 0=左, 1=右，基础扫描方向为右

  // 小车左转时云台向右补偿，右转时向左补偿；换算与锁靶追踪阶段完全一致。
  turn_output = turn_rate * gimbal_turn_scale / 6.0f;
  // 寻靶时补偿最大为±60RPM，避免补偿超过基础扫描速度而使扫描方向反转。
  if(turn_output > 60.0f)
    turn_output = 60.0f;
  else if(turn_output < -60.0f)
    turn_output = -60.0f;
  output += turn_output;

  // 基础扫描60RPM与补偿±60RPM合成后，寻靶速度限制在0至120RPM。
  if(output > 120.0f)
  {
    dir = 1;
    speed = 120;
  }
  else if(output < -120.0f)
  {
    dir = 0;
    speed = 120;
  }
  else if(output > 0.0f)
  {
    dir = 1;
    speed = (uint16)output;
  }
  else if(output < 0.0f)
  {
    dir = 0;
    speed = (uint16)(-output);
  }
  else
  {
    dir = 0;
    speed = 0;
  }

  motor_vel_control(GIMBAL_X_ADDR, dir, speed, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     云台追踪主函数
// 参数说明     pixel_err_x    - X轴像素偏移(目标X - 画面中心320)
// 参数说明     pixel_err_y    - Y轴像素偏移(目标Y - 画面中心240)
// 参数说明     turn_rate    - 小车Z轴角速度(°/s)，正值对应小车左转
// 参数说明     circle_mode  - PID模式: 0=锁靶, 1=画圆
// 备注信息     摄像头每帧调用一次，使用速度模式PID控制云台
//-------------------------------------------------------------------------------------------------------------------
void gimbal_track(int16 pixel_err_x, int16 pixel_err_y, float turn_rate, uint8 circle_mode)
{
  // 静态变量用于保存跨帧的PID状态：积分项累加历史误差，上一帧误差用于计算微分项。
  static float integral_x = 0.0f;  // X轴积分误差(像素帧)
  static float integral_y = 0.0f;  // Y轴积分误差(像素帧)
  static float last_err_x = 0.0f;  // 上一帧X轴误差(像素)
  static float last_err_y = 0.0f;  // 上一帧Y轴误差(像素)
  static uint8 last_mode = 0;      // 上一次PID模式: 0=锁靶, 1=画圆
  float kp = gimbal_Kp;            // 本次使用的比例系数
  float ki = gimbal_Ki;            // 本次使用的积分系数
  float kd = gimbal_Kd;            // 本次使用的微分系数
  float output = 0.0f;             // 当前轴合成后的有符号速度输出(RPM)
  float turn_output = 0.0f;        // 转角时的X轴角速度补偿输出(RPM)
  uint16 speed = 0;                // 要发给云台电机的速度绝对值(RPM)
  uint16 track_limit = lock_limit; // 本次X/Y轴常规跟踪速度上限(RPM)
  uint16 speed_limit = lock_limit; // 本次X轴包含转弯补偿后的速度上限(RPM)
  uint8 dir = 0;                   // 云台X轴电机方向: 0=左, 1=右
  uint8 deadband = 2;              // 当前像素死区(像素)

  // 锁靶和画圆切换时清除历史误差，避免上一种PID的积分和微分项带入下一种PID。
  if((circle_mode == 1 && last_mode == 0) || (circle_mode == 0 && last_mode == 1))
  {
    integral_x = 0.0f;
    integral_y = 0.0f;
    last_err_x = 0.0f;
    last_err_y = 0.0f;
    last_mode = circle_mode;
  }

  // 状态4画圆时切换到独立PID和更高的常规速度范围。
  if(circle_mode == 1)
  {
    kp = circle_kp;
    ki = circle_ki;
    kd = circle_kd;
    track_limit = circle_limit;
    speed_limit = circle_limit;
  }
  // 画圆模式固定使用±2像素死区，抑制视觉坐标跳动引起的反向波动。
  if(circle_mode == 1)
    deadband = 2;
  // 锁靶模式距离超过750mm时使用±1像素死区，其余距离使用±2像素抑制近距离抖动。
  else if(camera_distance > 750)
    deadband = 1;
  else
    deadband = 2;

  // X轴PID输出保留正负号：正值对应向右，负值对应向左。
  if(abs(pixel_err_x) <= deadband)
  {
    integral_x = 0.0f;
    last_err_x = 0.0f;
  }
  else
  {
    // 累积误差用于积分调节；限幅防止目标长时间偏离时积分项过大。
    integral_x += pixel_err_x;
    if(integral_x > 1000.0f)
      integral_x = 1000.0f;
    else if(integral_x < -1000.0f)
      integral_x = -1000.0f;

    // 正值表示目标在画面右侧，需要云台向右转动；负值则向左。
    output = kp * pixel_err_x + ki * integral_x + kd * (pixel_err_x - last_err_x);
    last_err_x = pixel_err_x;
  }

  // 仅转角状态会传入角速度：小车左转(正角速度)时云台向右补偿，右转时向左补偿。
  // 电机设定600RPM实测约60RPM，因此补偿转速需乘校准倍率后再发送。
  turn_output = turn_rate * gimbal_turn_scale / 6.0f;
  output += turn_output;

  if(turn_rate > 0.0f || turn_rate < 0.0f)
    speed_limit = gimbal_turn_limit;  // 转弯补偿使用独立上限，避免16RPM指令无法克服静摩擦

  // 将PID输出与转角补偿的合成速度限制在本次速度上限内，再转为电机方向和速度。
  if(output > (float)speed_limit)
  {
    dir = 1;
    speed = speed_limit;
  }
  else if(output < -(float)speed_limit)
  {
    dir = 0;
    speed = speed_limit;
  }
  else if(output > 0.0f)
  {
    dir = 1;
    speed = (uint16)output;
  }
  else if(output < 0.0f)
  {
    dir = 0;
    speed = (uint16)(-output);
  }
  else
  {
    dir = 0;
    speed = 0;
  }
  motor_vel_control(GIMBAL_X_ADDR, dir, speed, 0);

  // Y轴沿用与X轴相同的距离自适应死区。
  if(abs(pixel_err_y) <= deadband)
  {
    motor_vel_control(GIMBAL_Y_ADDR, 0, 0, 0);
    integral_y = 0.0f;
    last_err_y = 0.0f;
  }
  else
  {
    // Y轴独立累计积分并限幅，避免与水平轴控制相互影响。
    integral_y += pixel_err_y;
    if(integral_y > 100.0f)
      integral_y = 100.0f;
    else if(integral_y < -100.0f)
      integral_y = -100.0f;

    // 计算Y轴PID速度输出，并限制在当前模式的常规速度范围内。
    output = kp * pixel_err_y + ki * integral_y + kd * (pixel_err_y - last_err_y);
    last_err_y = pixel_err_y;
    if(output < 0.0f)
      output = -output;
    if(output > (float)track_limit)
      speed = track_limit;
    else
      speed = (uint16)output;

    // 俯仰轴安装方向与水平轴相反：目标在画面下方时 dir=0，上方时 dir=1。
    if(pixel_err_y > 0)
      dir = 0;
    else
      dir = 1;
    motor_vel_control(GIMBAL_Y_ADDR, dir, speed, 0);
  }
}
