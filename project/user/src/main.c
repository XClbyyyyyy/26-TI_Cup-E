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
int main(void)
{
  clock_init(SYSTEM_CLOCK_80M); 						 // 配置 80 MHz 系统时钟，供后续外设和延时功能使用。
  debug_init();                 						 // 初始化 UART0 调试串口，作为逐飞助手的通信接口。

  timer_config_init();  										 // 启动 1 ms 系统计时定时器。
  uart_config_init();  											 // UART 初始化

	uint8 code_time = 0;								//代码运行时间
	uint8 delay_time = 0;								//延时计数
	uint8 take_flag=0,put_flag=0;				//取放标志位
	uint8 stop_flag=0;								//急停标志位
	uint8 count=0;											//计数器
  while (true)
  {
		uint32 start_time = get_system_time_ms();
		
		uart0_process_data();							// 处理调试串口数据，更新参数变量。
		uart1_process_data();							// 处理摄像头数据，更新目标坐标和偏移量。
		uart6_process_data();							// 处理串口屏数据，更新运行状态和目标圈数。

		if(last_state != state)					  //串口屏显示
		{
			if(state == 0 && last_state != 0)
				stop_flag = 0;
			last_state = state; 
			uart_printf(UART_6,"show.n2.val=%d\xff\xff\xff",state);					
		}

		if(state == 0)
		{
			if(stop_flag == 0)
			{
				stop_flag = 1;
				stepper_stop(STEPPER_ADDR_X);		//X轴停止
				system_delay_ms(5);
				stepper_stop(STEPPER_ADDR_Y);		//Y轴停止
				system_delay_ms(5);
				stepper_stop(STEPPER_ADDR_Z);		//Z轴停止
				system_delay_ms(5);	
			}
		}
		//X轴和Y轴一起运动→完毕后Z轴运动→完毕后立即打开电磁铁视为取物成功
		//取物成功后XYT轴开始运动Z轴回到原来的高度→XY运动完毕后Z轴开始运动，完毕后关闭电磁铁视为放物成功
		//放物成功后回到原点开始进行第二步，循环直到取完拼图
		if(state == 1)
		{
			if(count < 4)
			{
				if(take_flag == 1 && put_flag == 1)
				{
					take_flag = 0;
					put_flag = 0;
					count++;
					stepper_origin_trigger_return(STEPPER_ADDR_X,0);		//X轴回零
					system_delay_ms(5);
					stepper_origin_trigger_return(STEPPER_ADDR_Y,0);		//Y轴回零
					system_delay_ms(5);
					stepper_origin_trigger_return(STEPPER_ADDR_Z,0);		//Z轴回零
					system_delay_ms(5);
				}
				else if(take_flag == 1 && put_flag == 0)
				{
					put_flag = 1;
				}
				else if(take_flag == 0 && put_flag == 0)
				{
					take_flag = 1;
				}
			}
			else
			{
				count = 0;
				state = 0;
			}
		}
		static uint32 last_send_time=0;
		if(get_system_time_ms() - last_send_time >= 1000)					// 每秒发送一次调试信息
		{
			last_send_time = get_system_time_ms();

			uart_printf(UART_0,"code_time:%d\r\n",code_time);
		}
		code_time = get_system_time_ms() - start_time;
  }
}
