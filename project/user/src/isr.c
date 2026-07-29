/*********************************************************************************************************************
* 本文件基于逐飞科技 MSPM0G3519 开源库开发
* 
* 文件名称          isr
* 备注信息          中断服务函数
********************************************************************************************************************/


#include "isr.h"

void TIMA0_IRQHandler (void)
{
    pit_callback_list[0](0, pit_callback_ptr_list[0]);
}

void TIMA1_IRQHandler (void)
{
    pit_callback_list[1](0, pit_callback_ptr_list[1]);
}

void TIMG0_IRQHandler (void)
{
    pit_callback_list[2](0, pit_callback_ptr_list[2]);
}

void TIMG6_IRQHandler (void)
{
    pit_callback_list[3](0, pit_callback_ptr_list[3]);
}

void TIMG7_IRQHandler (void)
{
    pit_callback_list[4](0, pit_callback_ptr_list[4]);
}

void TIMG8_IRQHandler (void)
{
    pit_callback_list[5](0, pit_callback_ptr_list[5]);
}

void TIMG9_IRQHandler (void)
{
    pit_callback_list[6](0, pit_callback_ptr_list[5]);
}

void TIMG12_IRQHandler (void)
{
    pit_callback_list[7](0, pit_callback_ptr_list[6]);
}

void TIMG14_IRQHandler (void)
{
    pit_callback_list[8](0, pit_callback_ptr_list[6]);
}

void UART0_IRQHandler (void)
{
	switch(DL_UART_getPendingInterrupt(UART0))
	{
		case DL_UART_IIDX_TX:
        {
            uart_callback_list[0](UART_INTERRUPT_STATE_TX, uart_callback_ptr_list[0]);
        }break;
		case DL_UART_IIDX_RX:
        {
            uart_callback_list[0](UART_INTERRUPT_STATE_RX, uart_callback_ptr_list[0]);
#if DEBUG_UART_USE_INTERRUPT
                debug_interrupr_handler();
#endif
        }break;

		default:    break;
	}
    DL_UART_clearInterruptStatus(UART0, UART0->CPU_INT.RIS);
}

void UART1_IRQHandler (void)
{
	switch(DL_UART_getPendingInterrupt(UART1))
	{
		case DL_UART_IIDX_TX:
        {
            uart_callback_list[1](UART_INTERRUPT_STATE_TX, uart_callback_ptr_list[1]);
        }break;
		case DL_UART_IIDX_RX:
        case DL_UART_IIDX_OVERRUN_ERROR:
        case DL_UART_IIDX_BREAK_ERROR:
        case DL_UART_IIDX_PARITY_ERROR:
        case DL_UART_IIDX_FRAMING_ERROR:
        case DL_UART_IIDX_RX_TIMEOUT_ERROR:
        case DL_UART_IIDX_NOISE_ERROR:
        {
            // 接收异常时FIFO内可能仍有有效字节；统一交给UART1回调读空，避免后续POS帧停止。
            uart_callback_list[1](UART_INTERRUPT_STATE_RX, uart_callback_ptr_list[1]);
        }break;

		default:    break;
	}
    DL_UART_clearInterruptStatus(UART1, UART1->CPU_INT.RIS);
}

//void UART2_IRQHandler (void)
//{
//	switch(DL_UART_getPendingInterrupt(UART2))
//	{
//		case DL_UART_IIDX_TX:
//        {
//            uart_callback_list[2](UART_INTERRUPT_STATE_TX, uart_callback_ptr_list[2]);
//        }break;
//		case DL_UART_IIDX_RX:
//        {
//            uart_callback_list[2](UART_INTERRUPT_STATE_RX, uart_callback_ptr_list[2]);
//        }break;

//		default:    break;
//	}
//    DL_UART_clearInterruptStatus(UART2, UART2->CPU_INT.RIS);
//}

void UART3_IRQHandler (void)
{
	switch(DL_UART_getPendingInterrupt(UART3))
	{
		case DL_UART_IIDX_TX:
        {
            uart_callback_list[3 - 1](UART_INTERRUPT_STATE_TX, uart_callback_ptr_list[3 - 1]);
        }break;
		case DL_UART_IIDX_RX:
        {
            uart_callback_list[3 - 1](UART_INTERRUPT_STATE_RX, uart_callback_ptr_list[3 - 1]);
        }break;

		default:    break;
	}
    DL_UART_clearInterruptStatus(UART3, UART3->CPU_INT.RIS);
}

void UART6_IRQHandler (void)
{
	switch(DL_UART_getPendingInterrupt(UART6))
	{
		case DL_UART_IIDX_TX:
        {
            uart_callback_list[6 - 1](UART_INTERRUPT_STATE_TX, uart_callback_ptr_list[6 - 1]);
        }break;
		case DL_UART_IIDX_RX:
        {
            uart_callback_list[6 - 1](UART_INTERRUPT_STATE_RX, uart_callback_ptr_list[6 - 1]);
        }break;

		default:    break;
	}
    DL_UART_clearInterruptStatus(UART6, UART6->CPU_INT.RIS);
}

void UART7_IRQHandler (void)
{
	switch(DL_UART_getPendingInterrupt(UART7))
	{
		case DL_UART_IIDX_TX:
        {
            uart_callback_list[7 - 1](UART_INTERRUPT_STATE_TX, uart_callback_ptr_list[7 - 1]);
        }break;
		case DL_UART_IIDX_RX:
        {
            uart_callback_list[7 - 1](UART_INTERRUPT_STATE_RX, uart_callback_ptr_list[7 - 1]);
        }break;

		default:    break;
	}
    DL_UART_clearInterruptStatus(UART7, UART7->CPU_INT.RIS);
}

void GROUP1_IRQHandler (void)
{
    uint8 exti_index = 0;
    uint8 exti_event = 0;

    uint32  register_temp = gpio_group[0]->CPU_INT.IIDX;
    if(register_temp)
    {
        exti_index = register_temp - 1;

        if(15 >= exti_index)
        {
            exti_event  = (gpio_group[0]->POLARITY15_0 >> ((exti_index % 16) * 2)) & 0x03;
        }
        else
        {
            exti_event  = (gpio_group[0]->POLARITY31_16 >> ((exti_index % 16) * 2)) & 0x03;
        }
        exti_callback_list[exti_index](exti_event, exti_callback_ptr_list[exti_index]);
    }
    else
    {
        register_temp = gpio_group[1]->CPU_INT.IIDX;
        if(register_temp)
        {
            exti_index = register_temp - 1;

            if(15 >= exti_index)
            {
                exti_event  = (gpio_group[1]->POLARITY15_0 >> ((exti_index % 16) * 2)) & 0x03;
            }
            else
            {
                exti_event  = (gpio_group[1]->POLARITY31_16 >> ((exti_index % 16) * 2)) & 0x03;
            }
            exti_callback_list[exti_index](exti_event, exti_callback_ptr_list[exti_index]);
        }
    }
}
