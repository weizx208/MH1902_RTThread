/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-30     SummerGift   first version
 * 2020-03-16     SummerGift   add device close feature
 * 2020-03-20     SummerGift   fix bug caused by ORE
 * 2020-05-02     whj4674672   support stm32h7 uart dma
 * 2020-10-14     Dozingfiretruck   Porting for stm32wbxx
 */

#include "board.h"
#include "drv_usart.h"

#ifdef RT_USING_SERIAL

//#define DRV_DEBUG
#define LOG_TAG             "drv.usart"
#include <drv_log.h>

#if !defined(BSP_USING_UART0) && !defined(BSP_USING_UART1) && !defined(BSP_USING_UART2)
#error "Please define at least one BSP_USING_UARTx"
/* this driver can be disabled at menuconfig -> RT-Thread Components -> Device Drivers */
#endif


enum
{
#ifdef BSP_USING_UART0
    UART0_INDEX,
#endif
#ifdef BSP_USING_UART1
    UART1_INDEX,
#endif
#ifdef BSP_USING_UART2
    UART2_INDEX,
#endif
};

static struct mhscpu_uart_config uart_config[] =
{
#ifdef BSP_USING_UART0
    UART0_CONFIG,
#endif
#ifdef BSP_USING_UART1
    UART1_CONFIG,
#endif
#ifdef BSP_USING_UART2
    UART2_CONFIG,
#endif
};

static struct mhscpu_uart uart_obj[sizeof(uart_config) / sizeof(uart_config[0])] = {0};

static rt_err_t mhscpu_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct mhscpu_uart *uart;
    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    uart = rt_container_of(serial, struct mhscpu_uart, serial);
    uart->init.UART_BaudRate    = cfg->baud_rate;
    uart->init.UART_WordLength	= UART_WordLength_8b;
	uart->init.UART_Parity 		= UART_Parity_No;
	uart->init.UART_StopBits	= UART_StopBits_1;

    switch (cfg->data_bits)
    {
    case DATA_BITS_5:
        uart->init.UART_WordLength = UART_WordLength_5b;
        break;
    case DATA_BITS_6:
        uart->init.UART_WordLength = UART_WordLength_6b;
        break;
	case DATA_BITS_7:
        uart->init.UART_WordLength = UART_WordLength_7b;
        break;
	case DATA_BITS_8:
		uart->init.UART_WordLength = UART_WordLength_8b;
        break;
    default:
        uart->init.UART_WordLength = UART_WordLength_8b;
        break;
    }

    switch (cfg->stop_bits)
    {
    case STOP_BITS_1:
        uart->init.UART_StopBits   = UART_StopBits_1;
        break;
    case STOP_BITS_2:
        uart->init.UART_StopBits   = UART_StopBits_2;
        break;
    default:
        uart->init.UART_StopBits   = UART_StopBits_1;
        break;
    }

    switch (cfg->parity)
    {
    case PARITY_NONE:
        uart->init.UART_Parity     = UART_Parity_No;
        break;
    case PARITY_ODD:
        uart->init.UART_Parity     = UART_Parity_Odd;
        break;
    case PARITY_EVEN:
        uart->init.UART_Parity     = UART_Parity_Even;
        break;
    default:
        uart->init.UART_Parity     = UART_Parity_No;
        break;
    }

	GPIO_PinRemapConfig(uart->config->port, uart->config->pin, uart->config->remap);
	
    UART_Init(uart->config->Instance, &uart->init);
	
    return RT_EOK;
}

static rt_err_t mhscpu_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct mhscpu_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = rt_container_of(serial, struct mhscpu_uart, serial);

    switch (cmd)
    {
    /* disable interrupt */
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        NVIC_DisableIRQ(uart->config->irq_type);
        /* disable interrupt */
		UART_ITConfig(uart->config->Instance, UART_IT_RX_RECVD, DISABLE);
        break;

    /* enable interrupt */
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        NVIC_SetPriority(uart->config->irq_type, 1); 
        NVIC_EnableIRQ(uart->config->irq_type);
        /* enable interrupt */
        UART_ITConfig(uart->config->Instance, UART_IT_RX_RECVD, ENABLE);
        break;

    case RT_DEVICE_CTRL_CLOSE:
        UART_DeInit(uart->config->Instance);
        break;

    }
    return RT_EOK;
}

static int mhscpu_putc(struct rt_serial_device *serial, char c)
{
    struct mhscpu_uart *uart;
    RT_ASSERT(serial != RT_NULL);

    uart = rt_container_of(serial, struct mhscpu_uart, serial);
	while(!UART_IsTXEmpty(uart->config->Instance)){}
	UART_SendData(uart->config->Instance, (uint8_t) c);
    return 1;
}

static int mhscpu_getc(struct rt_serial_device *serial)
{
    int ch;
    struct mhscpu_uart *uart;
    RT_ASSERT(serial != RT_NULL);
    uart = rt_container_of(serial, struct mhscpu_uart, serial);

    ch = -1;
	
	if(UART_GetLineStatus(uart->config->Instance) & UART_LINE_STATUS_RX_RECVD)
	{
		ch = UART_ReceiveData(uart->config->Instance);
	}
	
    return ch;
}

/**
 * Uart common interrupt process. This need add to uart ISR.
 *
 * @param serial serial device
 */
static void uart_isr(struct rt_serial_device *serial)
{
	volatile uint32_t iir;
    struct mhscpu_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = rt_container_of(serial, struct mhscpu_uart, serial);

	iir = UART_GetITIdentity(uart->config->Instance);
	
	switch(iir & 0x0F)
	{
		case UART_IT_ID_RX_RECVD:
			rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);
		break;
	}
}

#if defined(BSP_USING_UART0)
void UART0_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&(uart_obj[UART0_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* BSP_USING_UART0 */

#if defined(BSP_USING_UART1)
void UART1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&(uart_obj[UART1_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* BSP_USING_UART1 */

#if defined(BSP_USING_UART2)
void UART2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&(uart_obj[UART2_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* BSP_USING_UART2*/

static const struct rt_uart_ops stm32_uart_ops =
{
    .configure = mhscpu_configure,
    .control = mhscpu_control,
    .putc = mhscpu_putc,
    .getc = mhscpu_getc
};

int rt_hw_usart_init(void)
{
    rt_size_t obj_num = sizeof(uart_obj) / sizeof(struct mhscpu_uart);
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    rt_err_t result = 0;

    for (int i = 0; i < obj_num; i++)
    {
        /* init UART object */
        uart_obj[i].config = &uart_config[i];
        uart_obj[i].serial.ops    = &stm32_uart_ops;
        uart_obj[i].serial.config = config;

        /* register UART device */
        result = rt_hw_serial_register(&uart_obj[i].serial, uart_obj[i].config->name,
                                       RT_DEVICE_FLAG_RDWR
                                       | RT_DEVICE_FLAG_INT_RX
                                       | RT_DEVICE_FLAG_INT_TX
                                       , NULL);
        RT_ASSERT(result == RT_EOK);
    }

    return result;
}

#endif /* RT_USING_SERIAL */
