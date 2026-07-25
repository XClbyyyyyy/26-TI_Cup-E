/*********************************************************************************************************************
* 本文件基于逐飞科技 MSPM0G3519 开源库开发
* 
* 文件名称          gimbal_control
* 备注信息          云台追踪控制头文件
********************************************************************************************************************/

#ifndef _GIMBAL_CONTROL_H_
#define _GIMBAL_CONTROL_H_

#include "zf_common_headfile.h"

// 云台步进电机地址(硬件接线决定)
#define GIMBAL_X_ADDR  0x03
#define GIMBAL_Y_ADDR  0x04

// 可调PID参数(全局变量, 方便串口调试时实时修改)
extern float gimbal_Kp;
extern float gimbal_Ki;
extern float gimbal_Kd;
extern uint16 lock_limit;
extern float circle_kp;
extern float circle_ki;
extern float circle_kd;
extern uint16 circle_limit;

void gimbal_search(float turn_rate);
void gimbal_track(int16 pixel_err_x, int16 pixel_err_y, float turn_rate, uint8 circle_mode);

#endif
