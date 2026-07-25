/*********************************************************************************************************************
* 文件名称          zf_device_hc595_8digit
* 备注信息          两片 74HC595 驱动八位数码管
*********************************************************************************************************************/

#ifndef _ZF_DEVICE_HC595_8DIGIT_H_
#define _ZF_DEVICE_HC595_8DIGIT_H_

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

#define HC595_8DIGIT_DIO_PIN   A24  // 74HC595 DS 数据输入引脚
#define HC595_8DIGIT_SCLK_PIN  A25  // 74HC595 SH_CP 移位时钟引脚
#define HC595_8DIGIT_RCLK_PIN  A26  // 74HC595 ST_CP 锁存时钟引脚

extern uint8 hc595_8digit_buffer[8];  // 八位数码管显示缓存，0-15 表示 0-F，16 表示横杠
extern uint8 hc595_8digit_point_buffer[8];  // 八位数码管小数点缓存，0 为熄灭，1 为点亮

void hc595_8digit_init(void);
void hc595_8digit_display(void);

#endif
