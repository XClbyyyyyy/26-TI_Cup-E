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
* 文件名称          zf_driver_encoder
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK 5.38
* 适用平台          MSPM0G3519
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2025-06-1        SeekFree            first version
********************************************************************************************************************/

#ifndef _zf_driver_encoder_h_
#define _zf_driver_encoder_h_

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"
#include "zf_driver_timer.h"


// 定义引脚要用的宏定义 根据各单片机不同 可以自由定义需要什么信息
// bit[11:0 ] 固定为引脚索引
// bit[15:12] 固定为引脚复用
// bit[19:16] 在 ENCODER 模块中为 ENCODER 索引

#define     ENCODER_PIN_INDEX_OFFSET    ( 0      )                                  // bit[11:0 ] 存储 GPIO 的索引号
#define     ENCODER_PIN_INDEX_MASK      ( 0x0FFF )                                  // 宽度 12bit 因此掩码为 0x0FFF

#define     ENCODER_PIN_AF_OFFSET       ( 12     )                                  // bit[15:12] 存储 GPIO 的复用功能索引
#define     ENCODER_PIN_AF_MASK         ( 0x0F   )                                  // 宽度 4bit 因此掩码为 0x0F

#define     ENCODER_INDEX_OFFSET        ( 16     )                                  // bit[19:16] 存储 SPI 索引
#define     ENCODER_INDEX_MASK          ( 0x0F   )                                  // 宽度 4bit 因此掩码为 0x0F


typedef enum
{
    TIMA0_ENCODER1_CH1_A0   = (TIM_A0  << ENCODER_INDEX_OFFSET) | (0x04 << ENCODER_PIN_AF_OFFSET) | (A0 ),
    TIMA0_ENCODER1_CH1_A2   = (TIM_A0  << ENCODER_INDEX_OFFSET) | (0x0B << ENCODER_PIN_AF_OFFSET) | (A2 ),
    TIMA0_ENCODER1_CH1_A8   = (TIM_A0  << ENCODER_INDEX_OFFSET) | (0x05 << ENCODER_PIN_AF_OFFSET) | (A8 ),
    TIMA0_ENCODER1_CH1_A21  = (TIM_A0  << ENCODER_INDEX_OFFSET) | (0x05 << ENCODER_PIN_AF_OFFSET) | (A21),
    TIMA0_ENCODER1_CH1_B3   = (TIM_A0  << ENCODER_INDEX_OFFSET) | (0x0A << ENCODER_PIN_AF_OFFSET) | (B3 ),
    TIMA0_ENCODER1_CH1_B8   = (TIM_A0  << ENCODER_INDEX_OFFSET) | (0x05 << ENCODER_PIN_AF_OFFSET) | (B8 ),
    TIMA0_ENCODER1_CH1_B14  = (TIM_A0  << ENCODER_INDEX_OFFSET) | (0x05 << ENCODER_PIN_AF_OFFSET) | (B14),
    TIMA0_ENCODER1_CH1_B26  = (TIM_A0  << ENCODER_INDEX_OFFSET) | (0x04 << ENCODER_PIN_AF_OFFSET) | (B26),
    
    TIMA1_ENCODER1_CH1_A10  = (TIM_A1  << ENCODER_INDEX_OFFSET) | (0x0B << ENCODER_PIN_AF_OFFSET) | (A10),
    TIMA1_ENCODER1_CH1_A15  = (TIM_A1  << ENCODER_INDEX_OFFSET) | (0x0B << ENCODER_PIN_AF_OFFSET) | (A15),
    TIMA1_ENCODER1_CH1_A17  = (TIM_A1  << ENCODER_INDEX_OFFSET) | (0x0A << ENCODER_PIN_AF_OFFSET) | (A17),
    TIMA1_ENCODER1_CH1_A28  = (TIM_A1  << ENCODER_INDEX_OFFSET) | (0x09 << ENCODER_PIN_AF_OFFSET) | (A28),
    TIMA1_ENCODER1_CH1_B0   = (TIM_A1  << ENCODER_INDEX_OFFSET) | (0x08 << ENCODER_PIN_AF_OFFSET) | (B0 ),
    TIMA1_ENCODER1_CH1_B2   = (TIM_A1  << ENCODER_INDEX_OFFSET) | (0x0C << ENCODER_PIN_AF_OFFSET) | (B2 ),
    TIMA1_ENCODER1_CH1_B4   = (TIM_A1  << ENCODER_INDEX_OFFSET) | (0x08 << ENCODER_PIN_AF_OFFSET) | (B4 ),
    TIMA1_ENCODER1_CH1_B17  = (TIM_A1  << ENCODER_INDEX_OFFSET) | (0x0B << ENCODER_PIN_AF_OFFSET) | (B17),
    TIMA1_ENCODER1_CH1_B26  = (TIM_A1  << ENCODER_INDEX_OFFSET) | (0x09 << ENCODER_PIN_AF_OFFSET) | (B26),
    
    TIMG0_ENCODER1_CH1_A0   = (TIM_G0  << ENCODER_INDEX_OFFSET) | (0x09 << ENCODER_PIN_AF_OFFSET) | (A0 ),
    TIMG0_ENCODER1_CH1_A5   = (TIM_G0  << ENCODER_INDEX_OFFSET) | (0x05 << ENCODER_PIN_AF_OFFSET) | (A5 ),
    TIMG0_ENCODER1_CH1_A10  = (TIM_G0  << ENCODER_INDEX_OFFSET) | (0x07 << ENCODER_PIN_AF_OFFSET) | (A10),
    TIMG0_ENCODER1_CH1_A12  = (TIM_G0  << ENCODER_INDEX_OFFSET) | (0x07 << ENCODER_PIN_AF_OFFSET) | (A12),
    TIMG0_ENCODER1_CH1_A19  = (TIM_G0  << ENCODER_INDEX_OFFSET) | (0x06 << ENCODER_PIN_AF_OFFSET) | (A19),
    TIMG0_ENCODER1_CH1_A23  = (TIM_G0  << ENCODER_INDEX_OFFSET) | (0x09 << ENCODER_PIN_AF_OFFSET) | (A23),
    TIMG0_ENCODER1_CH1_B0   = (TIM_G0  << ENCODER_INDEX_OFFSET) | (0x06 << ENCODER_PIN_AF_OFFSET) | (B0 ),
    TIMG0_ENCODER1_CH1_B4   = (TIM_G0  << ENCODER_INDEX_OFFSET) | (0x06 << ENCODER_PIN_AF_OFFSET) | (B4 ),
    TIMG0_ENCODER1_CH1_B10  = (TIM_G0  << ENCODER_INDEX_OFFSET) | (0x02 << ENCODER_PIN_AF_OFFSET) | (B10),
    TIMG0_ENCODER1_CH1_B17  = (TIM_G0  << ENCODER_INDEX_OFFSET) | (0x06 << ENCODER_PIN_AF_OFFSET) | (B17),
    
    TIMG6_ENCODER1_CH1_A5   = (TIM_G6  << ENCODER_INDEX_OFFSET) | (0x07 << ENCODER_PIN_AF_OFFSET) | (A5 ),
    TIMG6_ENCODER1_CH1_A21  = (TIM_G6  << ENCODER_INDEX_OFFSET) | (0x0B << ENCODER_PIN_AF_OFFSET) | (A21),
    TIMG6_ENCODER1_CH1_A29  = (TIM_G6  << ENCODER_INDEX_OFFSET) | (0x09 << ENCODER_PIN_AF_OFFSET) | (A29),
    TIMG6_ENCODER1_CH1_B2   = (TIM_G6  << ENCODER_INDEX_OFFSET) | (0x0D << ENCODER_PIN_AF_OFFSET) | (B2 ),
    TIMG6_ENCODER1_CH1_B6   = (TIM_G6  << ENCODER_INDEX_OFFSET) | (0x0B << ENCODER_PIN_AF_OFFSET) | (B6 ),
    TIMG6_ENCODER1_CH1_B10  = (TIM_G6  << ENCODER_INDEX_OFFSET) | (0x09 << ENCODER_PIN_AF_OFFSET) | (B10),
    TIMG6_ENCODER1_CH1_B26  = (TIM_G6  << ENCODER_INDEX_OFFSET) | (0x0B << ENCODER_PIN_AF_OFFSET) | (B26),
    
    TIMG7_ENCODER1_CH1_A3   = (TIM_G7  << ENCODER_INDEX_OFFSET) | (0x0D << ENCODER_PIN_AF_OFFSET) | (A3 ),
    TIMG7_ENCODER1_CH1_A17  = (TIM_G7  << ENCODER_INDEX_OFFSET) | (0x0B << ENCODER_PIN_AF_OFFSET) | (A17),
    TIMG7_ENCODER1_CH1_A23  = (TIM_G7  << ENCODER_INDEX_OFFSET) | (0x0B << ENCODER_PIN_AF_OFFSET) | (A23),
    TIMG7_ENCODER1_CH1_A26  = (TIM_G7  << ENCODER_INDEX_OFFSET) | (0x0B << ENCODER_PIN_AF_OFFSET) | (A26),
    TIMG7_ENCODER1_CH1_A28  = (TIM_G7  << ENCODER_INDEX_OFFSET) | (0x0B << ENCODER_PIN_AF_OFFSET) | (A28),
    TIMG7_ENCODER1_CH1_B15  = (TIM_G7  << ENCODER_INDEX_OFFSET) | (0x0B << ENCODER_PIN_AF_OFFSET) | (B15),
    
    TIMG8_ENCODER1_CH1_A1   = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x07 << ENCODER_PIN_AF_OFFSET) | (A1 ),
    TIMG8_ENCODER1_CH1_A3   = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x02 << ENCODER_PIN_AF_OFFSET) | (A3 ),
    TIMG8_ENCODER1_CH1_A5   = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x02 << ENCODER_PIN_AF_OFFSET) | (A5 ),
    TIMG8_ENCODER1_CH1_A7   = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x04 << ENCODER_PIN_AF_OFFSET) | (A7 ),
    TIMG8_ENCODER1_CH1_A17  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x07 << ENCODER_PIN_AF_OFFSET) | (A17),
    TIMG8_ENCODER1_CH1_A21  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x0A << ENCODER_PIN_AF_OFFSET) | (A21),
    TIMG8_ENCODER1_CH1_A23  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x06 << ENCODER_PIN_AF_OFFSET) | (A23),
    TIMG8_ENCODER1_CH1_A26  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x04 << ENCODER_PIN_AF_OFFSET) | (A26),
    TIMG8_ENCODER1_CH1_A29  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x04 << ENCODER_PIN_AF_OFFSET) | (A29),
    TIMG8_ENCODER1_CH1_B6   = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x05 << ENCODER_PIN_AF_OFFSET) | (B6 ),
    TIMG8_ENCODER1_CH1_B10  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x03 << ENCODER_PIN_AF_OFFSET) | (B10),
    TIMG8_ENCODER1_CH1_B15  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x05 << ENCODER_PIN_AF_OFFSET) | (B15),
    TIMG8_ENCODER1_CH1_B21  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x05 << ENCODER_PIN_AF_OFFSET) | (B21),
    
    TIMG9_ENCODER1_CH1_A3   = (TIM_G9  << ENCODER_INDEX_OFFSET) | (0x07 << ENCODER_PIN_AF_OFFSET) | (A3 ),
    TIMG9_ENCODER1_CH1_B7   = (TIM_G9  << ENCODER_INDEX_OFFSET) | (0x07 << ENCODER_PIN_AF_OFFSET) | (B7 ),
    
    TIMG12_ENCODER1_CH1_A0  = (TIM_G12 << ENCODER_INDEX_OFFSET) | (0x08 << ENCODER_PIN_AF_OFFSET) | (A0 ),
    TIMG12_ENCODER1_CH1_A10 = (TIM_G12 << ENCODER_INDEX_OFFSET) | (0x09 << ENCODER_PIN_AF_OFFSET) | (A10),
    TIMG12_ENCODER1_CH1_A14 = (TIM_G12 << ENCODER_INDEX_OFFSET) | (0x05 << ENCODER_PIN_AF_OFFSET) | (A14),
    TIMG12_ENCODER1_CH1_A15 = (TIM_G12 << ENCODER_INDEX_OFFSET) | (0x08 << ENCODER_PIN_AF_OFFSET) | (A15),
    TIMG12_ENCODER1_CH1_A17 = (TIM_G12 << ENCODER_INDEX_OFFSET) | (0x08 << ENCODER_PIN_AF_OFFSET) | (A17),
    TIMG12_ENCODER1_CH1_B2  = (TIM_G12 << ENCODER_INDEX_OFFSET) | (0x09 << ENCODER_PIN_AF_OFFSET) | (B2 ),
    TIMG12_ENCODER1_CH1_B6  = (TIM_G12 << ENCODER_INDEX_OFFSET) | (0x0A << ENCODER_PIN_AF_OFFSET) | (B6 ),
    TIMG12_ENCODER1_CH1_B13 = (TIM_G12 << ENCODER_INDEX_OFFSET) | (0x04 << ENCODER_PIN_AF_OFFSET) | (B13),
    TIMG12_ENCODER1_CH1_B20 = (TIM_G12 << ENCODER_INDEX_OFFSET) | (0x04 << ENCODER_PIN_AF_OFFSET) | (B20),
    
    TIMG14_ENCODER1_CH1_A29 = (TIM_G14 << ENCODER_INDEX_OFFSET) | (0x0B << ENCODER_PIN_AF_OFFSET) | (A29),
    TIMG14_ENCODER1_CH1_B2  = (TIM_G14 << ENCODER_INDEX_OFFSET) | (0x07 << ENCODER_PIN_AF_OFFSET) | (B2 ),
    TIMG14_ENCODER1_CH1_B12 = (TIM_G14 << ENCODER_INDEX_OFFSET) | (0x0A << ENCODER_PIN_AF_OFFSET) | (B12),
}encoder_channel1_enum;             
                                    
typedef enum                        
{                                   
    TIMG8_ENCODER1_CH2_A0   = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x07 << ENCODER_PIN_AF_OFFSET) | (A0 ),
    TIMG8_ENCODER1_CH2_A2   = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x02 << ENCODER_PIN_AF_OFFSET) | (A2 ),
    TIMG8_ENCODER1_CH2_A4   = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x02 << ENCODER_PIN_AF_OFFSET) | (A4 ),
    TIMG8_ENCODER1_CH2_A6   = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x02 << ENCODER_PIN_AF_OFFSET) | (A6 ),
    TIMG8_ENCODER1_CH2_A18  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x07 << ENCODER_PIN_AF_OFFSET) | (A18),
    TIMG8_ENCODER1_CH2_A22  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x0A << ENCODER_PIN_AF_OFFSET) | (A22),
    TIMG8_ENCODER1_CH2_A24  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x06 << ENCODER_PIN_AF_OFFSET) | (A24),
    TIMG8_ENCODER1_CH2_A27  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x04 << ENCODER_PIN_AF_OFFSET) | (A27),
    TIMG8_ENCODER1_CH2_A30  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x04 << ENCODER_PIN_AF_OFFSET) | (A30),
    TIMG8_ENCODER1_CH2_B7   = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x05 << ENCODER_PIN_AF_OFFSET) | (B7 ),
    TIMG8_ENCODER1_CH2_B11  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x03 << ENCODER_PIN_AF_OFFSET) | (B11),
    TIMG8_ENCODER1_CH2_B16  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x05 << ENCODER_PIN_AF_OFFSET) | (B16),
    TIMG8_ENCODER1_CH2_B19  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x04 << ENCODER_PIN_AF_OFFSET) | (B19),
    TIMG8_ENCODER1_CH2_B22  = (TIM_G8  << ENCODER_INDEX_OFFSET) | (0x05 << ENCODER_PIN_AF_OFFSET) | (B22),
    
    
    TIMG9_ENCODER1_CH2_A2   = (TIM_G9  << ENCODER_INDEX_OFFSET) | (0x0D << ENCODER_PIN_AF_OFFSET) | (A2 ),
    TIMG9_ENCODER1_CH2_B9   = (TIM_G9  << ENCODER_INDEX_OFFSET) | (0x0D << ENCODER_PIN_AF_OFFSET) | (B9 ),
}encoder_channel2_enum;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ENCODER 获取计数值
// 参数说明     index           TIMER 外设模块号
// 返回参数     void
// 使用示例     encoder_get_count(index);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
int16   encoder_get_count       (timer_index_enum index);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ENCODER 清除计数值
// 参数说明     index           TIMER 外设模块号
// 返回参数     void
// 使用示例     encoder_clear_count(index);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void    encoder_clear_count     (timer_index_enum index);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ENCODER 正交解码模式初始化
// 参数说明     index           TIMER 外设模块号
// 参数说明     ch1_pin         通道1引脚
// 参数说明     ch2_pin         通道2引脚
// 返回参数     void
// 使用示例     encoder_quad_init(TIM_G8, TIMG8_ENCODER1_CH1_B10, TIMG8_ENCODER1_CH2_B11);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void    encoder_quad_init       (timer_index_enum index, encoder_channel1_enum ch1_pin, encoder_channel2_enum ch2_pin);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     ENCODER 带方向输出模式初始化
// 参数说明     index           TIMER 外设模块号
// 参数说明     lsb_pin         脉冲输入引脚
// 参数说明     dir_pin         方向输入引脚
// 返回参数     void
// 使用示例     encoder_dir_init(TIM_G8, TIMG8_ENCODER1_CH1_B10, B11);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void    encoder_dir_init        (timer_index_enum index, encoder_channel1_enum lsb_pin, gpio_pin_enum dir_pin);

#endif

