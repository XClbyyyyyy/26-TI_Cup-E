/*********************************************************************************************************************
* MSPM0G3519 Opensource Library 即（MSPM0G3519 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2026 SEEKFREE 逐飞科技
* 
* 本文件是 MSPM0G3519 开源库的一部分
* 
* MSPM0G3519 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK 5.38
* 适用平台          MSPM0G3519
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2026-06-1        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"

/**
 * @brief 程序入口，完成基础初始化后进入主循环。
 */
//PB7无法输出PWM波，暂时不使用此引脚
volatile uint8 count=0;											//计数器

int main(void)
{
  clock_init(SYSTEM_CLOCK_80M); 								 // 配置 80 MHz 系统时钟，供后续外设和延时功能使用。
  debug_init();                 								 // 初始化 UART0 调试串口，作为逐飞助手的通信接口。

  timer_config_init();  						
	// 启动 1 ms 系统计时定时器。
  uart_config_init();  															 // UART 初始化
	gpio_init(A21, GPO, GPIO_LOW, GPO_PUSH_PULL);			 // A21 推挽输出，打开蜂鸣器
	gpio_init(B7, GPO, GPIO_LOW, GPO_PUSH_PULL);			 // PB7 推挽输出，打开电磁铁

	uint8 take_flag=0,put_flag=0;				//取放标志位

	uint16 Zspeed=600;									//Z轴速度
  while (true)
  {		
		uart0_process_data();							// 处理调试串口数据，更新参数变量。
		uart1_process_data();							// 预留摄像头数据处理入口。
		uart4_process_data();							// 处理串口屏数据，更新运行状态和目标圈数。
				
		//X轴和Y轴一起运动→完毕后Z轴运动→完毕后立即打开电磁铁视为取物成功
		//取物成功后XYT轴开始运动，Z轴回到原来的高度→XY运动完毕后Z轴开始运动→完毕后关闭电磁铁视为放物成功
		//放物成功后回到原点开始进行第二步，循环直到取完拼图
		//XY轴电机转一圈4cm
		//X轴逆时针为正，Z轴逆时针为正，Y左顺时针为正，Y右逆时针为正
		//X轴从零点运动到纸面中心为12000脉冲，Y轴9000脉冲
		if(state == 1 && camera_plan_ready_flag == 1)
		{
			if(count < camera_piece_count)
			{
				if(take_flag == 1 && put_flag == 1)
				{
					take_flag = 0;
					put_flag = 0;
					count++;																								
					stepper_origin_trigger_return(STEPPER_ADDR_X,2);				//X轴回零
					system_delay_ms(5);	
					stepper_origin_trigger_return(STEPPER_ADDR_Y,2);				//Y轴回零
					system_delay_ms(5);	
					stepper_origin_trigger_return(STEPPER_ADDR_Z,0);				//Z轴回零
					system_delay_ms(2000);
				}
				else if(take_flag == 1 && put_flag == 0)
				{
					stepper_origin_trigger_return(STEPPER_ADDR_Z,0);				//Z轴回零
					system_delay_ms(5);
					stepper_pos_control(STEPPER_ADDR_X,camera_data[count].put_dir_x,XYspeed,0,camera_data[count].put_move_x_pulse,0);
					system_delay_ms(5);
					stepper_pos_control(STEPPER_ADDR_Y,camera_data[count].put_dir_y,XYspeed,0,camera_data[count].put_move_y_pulse,0);
					system_delay_ms(2000);
					stepper_pos_control(STEPPER_ADDR_Z,0,Zspeed,0,800,0);		//Z轴运动
					system_delay_ms(250);
					
					gpio_set_level(B7, GPIO_LOW);	//电磁铁放
					put_flag = 1;
				}
				else if(take_flag == 0 && put_flag == 0)
				{
					stepper_pos_control(STEPPER_ADDR_Y,1,1688,0,9000,0);
					system_delay_ms(5);
					stepper_pos_control(STEPPER_ADDR_X,0,2250,0,12000,0);
					system_delay_ms(2000);																//零点→纸张中心

					stepper_pos_control(STEPPER_ADDR_X,camera_data[count].Dir_x,XYspeed,0,camera_data[count].take_move_x_pulse,0);			
					system_delay_ms(5);
					stepper_pos_control(STEPPER_ADDR_Y,camera_data[count].Dir_y,XYspeed,0,camera_data[count].take_move_y_pulse,0);		
					system_delay_ms(5);
					stepper_pos_control(STEPPER_ADDR_Z,0,Zspeed,0,800,0);															
					system_delay_ms(250);																	//纸张中心→碎片中心
					take_flag = 1;																				
					gpio_set_level(B7, GPIO_HIGH);	//电磁铁吸

					stepper_pos_control(STEPPER_ADDR_X,1-camera_data[count].Dir_x,XYspeed,0,camera_data[count].take_move_x_pulse,0);			
					system_delay_ms(5);
					stepper_pos_control(STEPPER_ADDR_Y,1-camera_data[count].Dir_y,XYspeed,0,camera_data[count].take_move_y_pulse,0);		
					system_delay_ms(5);																		//碎片中心→纸张中心
					if(camera_data[count].rotation_deg < 0.0F)
						stepper_pos_control(STEPPER_ADDR_T,0,Tspeed,0,(uint16)(-camera_data[count].rotation_deg * 3200.0F / 360.0F + 0.5F),0);	//负角度顺时针旋转。
					else if(camera_data[count].rotation_deg > 0.0F)
						stepper_pos_control(STEPPER_ADDR_T,1,Tspeed,0,(uint16)(camera_data[count].rotation_deg * 3200.0F / 360.0F + 0.5F),0);	//正角度逆时针旋转。
					system_delay_ms(5);
				}
			} 
			else
			{
				count = 0;
				camera_plan_ready_flag = 0;						//当前计划完成，允许下一批有效计划重新启动。
				state = 0;
				gpio_toggle_level(A21);	//蜂鸣器响
				system_delay_ms(500);
				gpio_toggle_level(A21);	//蜂鸣器停
			}
		}

		static uint32 last_send_time=0;
		if(get_system_time_ms() - last_send_time >= 1000)							// 每秒发送一次调试信息
		{
			last_send_time = get_system_time_ms();

			uart_printf(UART_0,"direction:%d\r\n",direction);
			uart_printf(UART_4,"show.n2.val=%d\xff\xff\xff",state);	
			uart_printf(UART_4,"test.n0.val=%d\xff\xff\xff",XYspeed);
			uart_printf(UART_4,"test.n1.val=%d\xff\xff\xff",Tspeed);
			uart_printf(UART_4,"show.n1.val=%d\xff\xff\xff",camera_plan_ready_flag);
		}
  }
}
