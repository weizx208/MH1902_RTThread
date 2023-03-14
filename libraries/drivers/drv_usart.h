/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018.10.30     SummerGift   first version
 * 2019.03.05     whj4674672   add stm32h7
 * 2020-10-14     Dozingfiretruck   Porting for stm32wbxx
 */

#ifndef __DRV_USART_H__
#define __DRV_USART_H__

#include <rtthread.h>
#include "rtdevice.h"
#include <rthw.h>
#include <drv_common.h>
#include "mhscpu.h"


#if defined(BSP_USING_UART0)
#ifndef UART0_CONFIG
#define UART0_CONFIG                                                \
    {                                                               \
        .name = "uart0",                                            \
        .Instance = UART0,                                         	\
        .irq_type = UART0_IRQn,                                    	\
		.port = GPIOA,												\
		.pin = GPIO_Pin_0 | GPIO_Pin_1,								\
		.remap = GPIO_Remap_0,										\
    }
#endif /* UART0_CONFIG */
#endif /* BSP_USING_UART0 */

#if defined(BSP_USING_UART1)
#ifndef UART1_CONFIG
#define UART1_CONFIG                                                \
    {                                                               \
        .name = "uart1",                                            \
        .Instance = UART1,                                         	\
        .irq_type = UART1_IRQn,                                    	\
		.port = GPIOD,												\
		.pin = GPIO_Pin_4 | GPIO_Pin_5,								\
		.remap = GPIO_Remap_0,										\
    }
#endif /* UART1_CONFIG */
#endif /* BSP_USING_UART1 */
	
#if defined(BSP_USING_UART2)
#ifndef UART2_CONFIG
#define UART2_CONFIG                                                \
    {                                                               \
        .name = "uart2",                                            \
        .Instance = UART2,                                         	\
        .irq_type = UART2_IRQn,                                    	\
		.port = GPIOC,												\
		.pin = GPIO_Pin_13 | GPIO_Pin_15,							\
		.remap = GPIO_Remap_3,										\
    }
#endif /* UART2_CONFIG */
#endif /* BSP_USING_UART1 */
	
int rt_hw_usart_init(void);

/* mhscpu config class */
struct mhscpu_uart_config
{
    const char 			*name;
    UART_TypeDef 		*Instance;
    IRQn_Type 			irq_type;
	
	GPIO_TypeDef 		*port;
	uint16_t 			pin;
	GPIO_RemapTypeDef 	remap;
};

/* mhscpu uart dirver class */
struct mhscpu_uart
{
    UART_InitTypeDef 			init;
    struct mhscpu_uart_config 	*config;
	
    struct rt_serial_device 	serial;
};

#endif  /* __DRV_USART_H__ */
