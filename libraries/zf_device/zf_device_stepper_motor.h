/*********************************************************************************************************************
* 本文件基于逐飞科技 MSPM0G3519 开源库开发
* 
* 文件名称          zf_device_stepper_motor
* 备注信息          步进电机驱动头文件 (ZDT-XS 系列闭环步进电机协议)
********************************************************************************************************************/

#ifndef _ZF_DEVICE_STEPPER_MOTOR_H_
#define _ZF_DEVICE_STEPPER_MOTOR_H_

#include "zf_common_typedef.h"

//-------------------------------------------------------------------------------------------------------------------
// 步进电机地址定义
//-------------------------------------------------------------------------------------------------------------------
#define STEPPER_ADDR_X    (0x01)  // X轴电机地址。
#define STEPPER_ADDR_Y    (0x02)  // Y轴电机地址。
#define STEPPER_ADDR_Z    (0x03)  // Z轴电机地址。
#define STEPPER_ADDR_T    (0x04)  // T轴电机地址。

//-------------------------------------------------------------------------------------------------------------------
// 步进电机方向定义
//-------------------------------------------------------------------------------------------------------------------
#define STEPPER_DIR_CW    (0x00)  // 顺时针方向
#define STEPPER_DIR_CCW   (0x01)  // 逆时针方向

//-------------------------------------------------------------------------------------------------------------------
// 函数声明
//-------------------------------------------------------------------------------------------------------------------
void stepper_init(void);  // 步进电机初始化

// 基础控制函数
void stepper_enable(uint8 addr, uint8 state);               // 使能控制: state=1使能, state=0失能
void stepper_reset(uint8 addr);                             // 复位电机
void stepper_stop(uint8 addr);                              // 急停
void stepper_set_pos_zero(uint8 addr);                      // 设置当前位置为零点
void stepper_origin_trigger_return(uint8 addr, uint8 origin_mode);  // 立即触发回零。

// 运动控制函数
void stepper_vel_control(uint8 addr, uint8 dir, uint16 vel, uint8 acc);                    // 速度模式
void stepper_pos_control(uint8 addr, uint8 dir, uint16 vel, uint8 acc, uint32 clk, uint8 raF);  // 位置模式



//-------------------------------------------------------------------------------------------------------------------
// 电机非阻塞命令队列函数
//-------------------------------------------------------------------------------------------------------------------
void motor_process(void);  // 每次最多发送一条命令；一次性位置与回零命令按调用顺序发送

// 基础控制函数
void motor_vel_control(uint8 addr, uint8 dir, uint16 vel, uint8 acc);  // 速度模式
void motor_enable(uint8 addr, uint8 state);  // 使能控制: state=0失能, state=1使能
void motor_stop(uint8 addr);  // 急停
void motor_pos_control(uint8 addr, uint8 dir, uint16 vel, uint8 acc, uint32 clk, uint8 raF);  // 位置模式

// 一次性回零控制函数（连续调用时按调用顺序发送）
void motor_origin_set_o(uint8 addr, uint8 store_flag);  // 设置单圈回零零点位置
void motor_origin_trigger_return(uint8 addr, uint8 origin_mode);  // 触发回零
#endif
