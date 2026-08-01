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
volatile uint8 count = 0; // 计数器

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据位置模式的脉冲数和电机轴速度计算最短完成等待时间。
// 状态机逻辑   无状态；仅用于在下一段运动命令发出前等待当前运动完成。
//-------------------------------------------------------------------------------------------------------------------
static uint32 stepper_move_delay_ms(uint32 pulse_count, uint16 speed_rpm)
{
	uint64 pulse_frequency; // 输出轴在当前转速下的每分钟脉冲数，单位：pulse/min。

	pulse_frequency = (uint64)3200U * (uint64)speed_rpm; // 每圈固定为 3200 脉冲，先按电机指令转速计算每分钟脉冲数。
	pulse_frequency /= 10U; // 10:1 减速仅将电机指令转速折算为输出轴转速。
	return (uint32)(((uint64)pulse_count * 60000U + pulse_frequency - 1U) / pulse_frequency) + 100U; // 额外预留 100 ms，确保 XY 轴完成定位。
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算两轴同时运动完成所需的等待时间。
// 状态机逻辑   无状态；以脉冲数较大的轴作为本段 X/Y 运动完成条件。
//-------------------------------------------------------------------------------------------------------------------
static uint32 stepper_xy_move_delay_ms(uint32 x_pulse_count, uint32 y_pulse_count, uint16 speed_rpm)
{
	if (x_pulse_count >= y_pulse_count)
		return stepper_move_delay_ms(x_pulse_count, speed_rpm);

	return stepper_move_delay_ms(y_pulse_count, speed_rpm);
}

int main(void)
{
	clock_init(SYSTEM_CLOCK_80M); // 配置 80 MHz 系统时钟，供后续外设和延时功能使用。
	debug_init();									// 初始化 UART0 调试串口，作为逐飞助手的通信接口。

	timer_config_init();
	// 启动 1 ms 系统计时定时器。
	uart_config_init();														// UART 初始化
	gpio_init(A21, GPO, GPIO_LOW, GPO_PUSH_PULL); // A21 推挽输出，打开蜂鸣器
	gpio_init(B7, GPO, GPIO_LOW, GPO_PUSH_PULL);	// PB7 推挽输出，打开电磁铁

	uint8 take_flag = 0, put_flag = 0;					// 取放标志位
	static uint8 camera_last_request_state = 0; // 上一次已经处理摄像头模式请求的运行状态。
	uint8 camera_request_state = 0;             // 本轮读取的状态请求值，用于保证发送与记录使用同一状态。
	while (true)
	{
		uart4_process_manual_move(); // 执行串口屏请求的单轴点动。
		uart0_process_data();				 // 处理调试串口数据，更新参数变量。
		camera_request_state = state;
  if ((camera_request_state == 1 || camera_request_state == 2 || camera_request_state == 3) && camera_request_state != camera_last_request_state)
  {
    stepper_pos_control(STEPPER_ADDR_X, 0, 3000, 0, 23500, 0); // X 电机以 3000 RPM 顺时针转动 23500 脉冲。
    system_delay_ms(stepper_move_delay_ms(23500, 3000) + 500U); // 按 3000 RPM 等待 X 电机完成运动，并额外预留 1 秒。
  }
		if (camera_request_state == 1 || camera_request_state == 2 || camera_request_state == 3) // 状态切换后仅请求一次对应摄像头模式。
		{
			if (camera_request_state != camera_last_request_state)
			{
				camera_prepare_new_request(); // 发送新请求前丢弃摄像头持续发送的旧数据。
				uart_printf(UART_1, "%d\n", camera_request_state);
				uart_printf(UART_0, "%d\n", camera_request_state);
				camera_enable_plan_receive(); // 状态命令发出后，仅接收本次的一份完整计划。
			}
		}
		camera_last_request_state = camera_request_state;
		uart1_process_data();				 // 仅在摄像头接收窗口开启时解析计划数据。

		// X轴和Y轴一起运动→完毕后Z轴运动→完毕后立即打开电磁铁视为取物成功
		// 取物成功后XYT轴开始运动，Z轴回到原来的高度→XY运动完毕后Z轴开始运动→完毕后关闭电磁铁视为放物成功
		// 放物成功后回到原点开始进行第二步，循环直到取完拼图
		// XY轴电机转一圈4cm
		// XYZ轴逆时针为正
		// 连续运动基准为机械原点；纸张中心相对原点的固定坐标已在摄像头数据换算中叠加。
		if (screen_stop_origin_request_flag == 1)
		{
			screen_stop_origin_request_flag = 0;							// 当前回零请求已由主循环接管。
			stepper_origin_trigger_return(STEPPER_ADDR_X, 2); // 触发 X 轴多圈碰撞回零。
			system_delay_ms(5);
			stepper_origin_trigger_return(STEPPER_ADDR_Y, 2); // 触发 Y 轴多圈碰撞回零。
			system_delay_ms(5);
			stepper_origin_trigger_return(STEPPER_ADDR_Z, 0); // 触发 Z 轴单圈就近回零。
			system_delay_ms(5);
			stepper_origin_trigger_return(STEPPER_ADDR_T, 0); // 触发 T 轴单圈就近回零。
		}

		if (state != 0 && camera_plan_ready_flag == 1)
		{
			if (count < camera_piece_count)
			{
				if (take_flag == 1 && put_flag == 1)
				{
					take_flag = 0;
					put_flag = 0;
					count++;
					stepper_pos_control(STEPPER_ADDR_Z, 1, Zspeed, 0, 500, 0); // Z轴上升
					system_delay_ms(450);
					stepper_origin_trigger_return(STEPPER_ADDR_T, 0);
					system_delay_ms(5);
					if (count >= camera_piece_count)
					{
						stepper_origin_trigger_return(STEPPER_ADDR_X, 2);
						system_delay_ms(5);
						stepper_origin_trigger_return(STEPPER_ADDR_Y, 2);
						system_delay_ms(5);
					}
				}
				else if (take_flag == 1 && put_flag == 0)
				{
					stepper_pos_control(STEPPER_ADDR_Z, 1, Zspeed, 0, 500, 0); // Z轴上升
					system_delay_ms(450);
					if (camera_data[count].rotation_deg < 0.0F)
					{
						stepper_pos_control(STEPPER_ADDR_T, 1, Tspeed, 0, (uint16)(-camera_data[count].rotation_deg * 3200.0F / 360.0F + 0.5F), 0); // 负角度逆时针旋转。
						system_delay_ms(5); // 等待 T 轴完成负角度旋转并预留 100 ms。
					}
					else if (camera_data[count].rotation_deg >= 0.0F)
					{
						stepper_pos_control(STEPPER_ADDR_T, 0, Tspeed, 0, (uint16)(camera_data[count].rotation_deg * 3200.0F / 360.0F + 0.5F), 0); // 正角度顺时针旋转。
						system_delay_ms(5); // 等待 T 轴完成正角度旋转并预留 100 ms。
					}
					stepper_pos_control(STEPPER_ADDR_Y, camera_data[count].put_dir_x, XYspeed, 0, camera_data[count].put_move_x_pulse, 0); // 摄像头 X 坐标由 Y 电机执行。
					system_delay_ms(5);
					stepper_pos_control(STEPPER_ADDR_X, camera_data[count].put_dir_y, XYspeed, 0, camera_data[count].put_move_y_pulse, 0); // 摄像头 Y 坐标由 X 电机执行。
					system_delay_ms(stepper_xy_move_delay_ms(camera_data[count].put_move_x_pulse, camera_data[count].put_move_y_pulse, XYspeed));
					stepper_pos_control(STEPPER_ADDR_Z, 0, Zspeed, 0, 500, 0); // Z轴下降
					system_delay_ms(450); // Z 轴固定 500 脉冲，输出轴 60 RPM 时理论约 375 ms，额外预留约 225 ms。
					gpio_set_level(B7, GPIO_LOW);																// 电磁铁放
					system_delay_ms(200);																				// Z 轴固定 500 脉冲，输出轴 60 RPM 时理论约 375 ms，额外预留约 225 ms。
					put_flag = 1;
				}
				else if (take_flag == 0 && put_flag == 0)
				{
					stepper_pos_control(STEPPER_ADDR_Y, camera_data[count].Dir_x, XYspeed, 0, camera_data[count].take_move_x_pulse, 0); // 摄像头 X 坐标由 Y 电机执行。
					system_delay_ms(5);
					stepper_pos_control(STEPPER_ADDR_X, camera_data[count].Dir_y, XYspeed, 0, camera_data[count].take_move_y_pulse, 0); // 摄像头 Y 坐标由 X 电机执行。
					system_delay_ms(stepper_xy_move_delay_ms(camera_data[count].take_move_x_pulse, camera_data[count].take_move_y_pulse, XYspeed));
					stepper_pos_control(STEPPER_ADDR_Z, 0, Zspeed, 0, 500, 0);
					system_delay_ms(450);					 // Z 轴固定500脉冲，60RPM时理论375 ms，额外预留225 ms。
					take_flag = 1;
					gpio_set_level(B7, GPIO_HIGH); // 电磁铁吸
					system_delay_ms(200);					
				}
			}
			else
			{
				count = 0;
				camera_plan_ready_flag = 0; // 当前计划完成，允许下一批有效计划重新启动。
				state = 0;
				gpio_toggle_level(A21); // 蜂鸣器响
				system_delay_ms(500);
				gpio_toggle_level(A21); // 蜂鸣器停
			}
		}

		static uint32 last_send_time = 0;
		if (get_system_time_ms() - last_send_time >= 1000) // 每秒发送一次调试信息
		{
			last_send_time = get_system_time_ms();

			uart_printf(UART_0, "state=%d\n", state);
			uart_printf(UART_4, "show.n2.val=%d\xff\xff\xff", state);
			uart_printf(UART_4, "show.n1.val=%d\xff\xff\xff", camera_plan_ready_flag);
		}
	}
}
