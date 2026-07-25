/*********************************************************************************************************************
* 文件名称          zf_device_hc595_8digit
* 备注信息          两片 74HC595 驱动八位数码管
*********************************************************************************************************************/

#include "zf_device_hc595_8digit.h"

uint8 hc595_8digit_buffer[8] = {0};  // 八位数码管显示缓存，数组下标 0-7 对应第 1-8 位
uint8 hc595_8digit_point_buffer[8] = {0};  // 八位数码管小数点缓存，数组下标 0-7 对应第 1-8 位

static const uint8 hc595_8digit_font[17] =
{
  0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80,
  0x90, 0x8C, 0xBF, 0xC6, 0xA1, 0x86, 0xFF, 0xBF
};  // 共阳数码管段码，低电平点亮，依次为 0-9、A、b、C、d、E、F、-

// 通过 DIO 和 SCLK 将一个字节按最高位在前的顺序移入 74HC595。
static void hc595_8digit_shift_out(uint8 value)
{
  uint8 i = 0;  // 当前移出的位序号，范围 0-7

  for(i = 0; i < 8; i++)
  {
    if((value & 0x80) == 0x80)
      gpio_high(HC595_8DIGIT_DIO_PIN);
    else
      gpio_low(HC595_8DIGIT_DIO_PIN);

    value <<= 1;
    gpio_low(HC595_8DIGIT_SCLK_PIN);
    gpio_high(HC595_8DIGIT_SCLK_PIN);
  }
}

// 初始化数码管使用的三个 GPIO，初始均输出低电平。
void hc595_8digit_init(void)
{
  gpio_init(HC595_8DIGIT_DIO_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
  gpio_init(HC595_8DIGIT_SCLK_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
  gpio_init(HC595_8DIGIT_RCLK_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
	for(uint8 i=0;i<8;i++)				 
	{
		hc595_8digit_buffer[i]=17;
		hc595_8digit_point_buffer[i]=0;
	}
}

// 刷新八位数码管：每次调用依次输出八位显示缓存，并锁存段选和位选数据。
void hc595_8digit_display(void)
{
  uint8 i = 0;            // 当前刷新的数码管位序号，范围 0-7
  uint8 segment_data = 0; // 当前位的段选数据
  uint8 digit_data = 0;   // 当前位的位选数据

  for(i = 0; i < 8; i++)
  {
    if(hc595_8digit_buffer[i] <= 16)
      segment_data = hc595_8digit_font[hc595_8digit_buffer[i]];
    else
      segment_data = 0xFF;

    if(hc595_8digit_point_buffer[i] == 1)
      segment_data &= 0x7F;

    digit_data = (uint8)(1U << i);
    hc595_8digit_shift_out(segment_data);
    hc595_8digit_shift_out(digit_data);

    gpio_low(HC595_8DIGIT_RCLK_PIN);
    gpio_high(HC595_8DIGIT_RCLK_PIN);
  }
}
