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
* 文件名称          mian
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

volatile uint8 circle = 0;                 								// 目标圈数，供串口配置和定时器逻辑使用。
volatile uint8 out_of_line = 0;     											// 出线标志，供定时器逻辑读取。

/**
 * @brief 程序入口，完成基础初始化后进入主循环。
 */
int main(void)
{
  clock_init(SYSTEM_CLOCK_80M); 						 // 配置 80 MHz 系统时钟，供后续外设和延时功能使用。
  debug_init();                 						 // 初始化 UART0 调试串口，作为逐飞助手的通信接口。

	gpio_init(A14,GPO,GPIO_LOW,GPO_PUSH_PULL); // 初始化 A14 为推挽输出，点亮板载LED
  hc595_8digit_init(); 											 // 初始化数码管 GPIO 和显示缓存。
	jy901_init(); 													 	 // 初始化 JY901 姿态传感器，供后续读取角速度和角度数据。
	for(uint8 i = 0;i < 8;i++)
		hc595_8digit_buffer[i]=i;
  timer_config_init();  										 // 启动 1 ms 系统计时定时器。
  uart_config_init();  											 // UART 初始化。
	
	uint8 code_time = 0;								//代码运行时间
  while (true)
  {
		uint32 start_time = get_system_time_ms();
		
		hc595_8digit_display();						// 刷新数码管显示。
		uart0_process_data();							// 处理调试串口数据，更新参数变量。
		uart1_process_data();							// 处理摄像头数据，更新目标坐标和偏移量。
		uart3_process_data();							// 处理步进电机数据，更新步进电机状态和目标位置。
		jy901_process_data();							// 处理姿态传感器数据，更新角速度和角度。
		uart5_process_data();							// 处理灰度传感器数据，更新灰度传感器状态和偏移量。
		uart6_process_data();							// 处理串口屏数据，更新运行状态和目标圈数。

		if(last_state != state)
		{
			last_state = state; 
			uart_printf(UART_6,"show.n2.val=%d\xff\xff\xff",state);					//串口屏显示
			uart_printf(UART_6,"show.n3.val=%d\xff\xff\xff",circle);
		}
		if(state == 0)
		{

		}

		static uint32 last_send_time=0;
		if(get_system_time_ms() - last_send_time >= 1000)
		{
			last_send_time = get_system_time_ms();

			uart_printf(UART_0,"code_time:%d\r\n",code_time);
			uart_printf(UART_0,"jy901_data.angle_z:%.2f\r\n", jy901_data.angle_z);
		}
		code_time = get_system_time_ms() - start_time;
  }
}
