/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-5      SummerGift   first version
 */

#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__

#include <rtthread.h>
#include "rtdevice.h"
#include <rthw.h>
#include <drv_common.h>
#include "mhscpu.h"

#ifdef __cplusplus
extern "C" {
#endif

rt_err_t rt_hw_spi_device_attach(const char *bus_name, const char *device_name, rt_base_t cs_pin);

#ifdef __cplusplus
}
#endif

#ifdef BSP_USING_SPI0
#ifndef SPI0_BUS_CONFIG
#define SPI0_BUS_CONFIG                             		\
    {                                               		\
        .Instance = SPIM0,                          		\
        .bus_name = "spi0",                         		\
		.io.port = GPIOB,									\
		.io.pin = GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5,		\
		.io.remap = GPIO_Remap_0,							\
    }
#endif /* SPI0_BUS_CONFIG */
#endif /* BSP_USING_SPI0 */

#ifdef BSP_USING_SPI1
#ifndef SPI1_BUS_CONFIG
#define SPI1_BUS_CONFIG                             		\
    {                                               		\
        .Instance = SPIM1,                          		\
        .bus_name = "spi1",                         		\
		.io.port = GPIOD,									\
		.io.pin = GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_11,	\
		.io.remap = GPIO_Remap_0,							\
    }
#endif /* SPI1_BUS_CONFIG */
#endif /* BSP_USING_SPI1 */
	
#ifdef BSP_USING_SPI2
#ifndef SPI1_BUS_CONFIG
#define SPI1_BUS_CONFIG                             		\
    {                                               		\
        .Instance = SPIM2,                          		\
        .bus_name = "spi2",                         		\
		.io.port = GPIOB,									\
		.io.pin = GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5,		\
		.io.remap = GPIO_Remap_0,							\
    }
#endif /* SPI2_BUS_CONFIG */
#endif /* BSP_USING_SPI2 */

struct mhscpu_hw_spi_cs
{
    rt_base_t pin;
};

struct mhscpu_spi_io
{
	GPIO_TypeDef *port;
	uint16_t pin;
	GPIO_RemapTypeDef remap;
};

struct mhscpu_spi_config
{
    SPI_TypeDef *Instance;
    char *bus_name;
    struct dma_config *dma_rx, *dma_tx;
	
	struct mhscpu_spi_io io;
};

struct mhscpu_spi_device
{
    rt_uint32_t pin;
    char *bus_name;
    char *device_name;
};

#define SPI_USING_RX_DMA_FLAG   (1<<0)
#define SPI_USING_TX_DMA_FLAG   (1<<1)

/* mh1902 spi dirver class */
struct mhscpu_spi
{
    SPI_InitTypeDef init;
    struct mhscpu_spi_config *config;
    struct rt_spi_configuration *cfg;
    
    rt_uint8_t spi_dma_flag;
    struct rt_spi_bus spi_bus;
};

#endif /*__DRV_SPI_H__ */
