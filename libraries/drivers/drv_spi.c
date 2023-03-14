/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-5      SummerGift   first version
 * 2018-12-11     greedyhao    Porting for stm32f7xx
 * 2019-01-03     zylx         modify DMA initialization and spixfer function
 * 2020-01-15     whj4674672   Porting for stm32h7xx
 * 2020-06-18     thread-liu   Porting for stm32mp1xx
 * 2020-10-14     Dozingfiretruck   Porting for stm32wbxx
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"

#ifdef RT_USING_SPI

#if defined(BSP_USING_SPI0) || defined(BSP_USING_SPI1) || defined(BSP_USING_SPI2)

#include "drv_spi.h"
#include <string.h>

//#define DRV_DEBUG
#define LOG_TAG              "drv.spi"
#include <drv_log.h>

enum
{
#ifdef BSP_USING_SPI0
    SPI1_INDEX,
#endif
#ifdef BSP_USING_SPI1
    SPI2_INDEX,
#endif
#ifdef BSP_USING_SPI2
    SPI3_INDEX,
#endif
};

static struct mhscpu_spi_config spi_config[] =
{
#ifdef BSP_USING_SPI0
    SPI0_BUS_CONFIG,
#endif

#ifdef BSP_USING_SPI1
    SPI1_BUS_CONFIG,
#endif

#ifdef BSP_USING_SPI2
    SPI2_BUS_CONFIG,
#endif
};

static struct mhscpu_spi spi_bus_obj[sizeof(spi_config) / sizeof(spi_config[0])] = {0};

static rt_err_t mhscpu_spi_init(struct mhscpu_spi *spi_drv, struct rt_spi_configuration *cfg)
{
    RT_ASSERT(spi_drv != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    SPI_InitTypeDef *spi_init = &spi_drv->init;

    if (cfg->mode & RT_SPI_SLAVE)
    {
        spi_init->SPI_Mode = SPI_Mode_Slave;
    }
    else
    {
        spi_init->SPI_Mode = SPI_Mode_Master;
    }

    if (cfg->mode & RT_SPI_3WIRE)
    {
        spi_init->SPI_Direction = SPI_Direction_1Line_Rx;
    }
    else
    {
        spi_init->SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    }

    if (cfg->data_width == 8)
    {
        spi_init->SPI_DataSize = SPI_DataSize_8b;
    }
    else if (cfg->data_width == 16)
    {
        spi_init->SPI_DataSize = SPI_DataSize_16b;
    }
    else
    {
        return RT_EIO;
    }

    if (cfg->mode & RT_SPI_CPHA)
    {
        spi_init->SPI_CPHA = SPI_CPHA_2Edge;
    }
    else
    {
        spi_init->SPI_CPHA = SPI_CPHA_1Edge;
    }

    if (cfg->mode & RT_SPI_CPOL)
    {
        spi_init->SPI_CPOL = SPI_CPOL_High;
    }
    else
    {
        spi_init->SPI_CPOL = SPI_CPOL_Low;
    }

    if (cfg->mode & RT_SPI_NO_CS)
    {
        spi_init->SPI_NSS = SPI_NSS_0;
    }
    else
    {
        spi_init->SPI_NSS = SPI_NSS_0;
    }

	SYSCTRL_ClocksTypeDef SYSCTRL_Clocks;
    uint32_t SPI_APB_CLOCK;
	
    SYSCTRL_GetClocksFreq(&SYSCTRL_Clocks);
	SPI_APB_CLOCK = SYSCTRL_Clocks.PCLK_Frequency;

    if (cfg->max_hz >= SPI_APB_CLOCK / 2)
    {
        spi_init->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    }
    else if (cfg->max_hz >= SPI_APB_CLOCK / 4)
    {
        spi_init->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    }
    else if (cfg->max_hz >= SPI_APB_CLOCK / 8)
    {
        spi_init->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    }
    else if (cfg->max_hz >= SPI_APB_CLOCK / 16)
    {
        spi_init->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    }
    else if (cfg->max_hz >= SPI_APB_CLOCK / 32)
    {
        spi_init->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    }
    else if (cfg->max_hz >= SPI_APB_CLOCK / 64)
    {
        spi_init->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
    }
    else if (cfg->max_hz >= SPI_APB_CLOCK / 128)
    {
        spi_init->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
    }
    else
    {
        /*  min prescaler 256 */
        spi_init->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    }

    LOG_D("sys freq: %d, pclk2 freq: %d, SPI limiting freq: %d, BaudRatePrescaler: %d",
#if defined(SOC_SERIES_STM32MP1)
          HAL_RCC_GetSystemCoreClockFreq(),
#else
          HAL_RCC_GetSysClockFreq(),
#endif
          SPI_APB_CLOCK,
          cfg->max_hz,
          spi_init->SPI_BaudRatePrescaler);

    if (cfg->mode & RT_SPI_MSB)
    {
        //
    }
    else
    {
        return RT_EIO;
    }

	spi_init->SPI_RXFIFOFullThreshold = SPI_RXFIFOFullThreshold_1;
	spi_init->SPI_TXFIFOEmptyThreshold = SPI_TXFIFOEmptyThreshold_0;

	GPIO_PinRemapConfig(spi_drv->config->io.port, spi_drv->config->io.pin, spi_drv->config->io.remap);
	
    SPI_Init(spi_drv->config->Instance, spi_init);
	SPI_Cmd(spi_drv->config->Instance, ENABLE);
	
    LOG_D("%s init done", spi_drv->config->bus_name);
    return RT_EOK;
}

static rt_uint32_t spixfer(struct rt_spi_device *device, struct rt_spi_message *message)
{
    int state = 0;
    rt_size_t message_length, already_send_length;
    rt_uint16_t send_length;
    rt_uint8_t *recv_buf;
    const rt_uint8_t *send_buf;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);
    RT_ASSERT(device->bus->parent.user_data != RT_NULL);
    RT_ASSERT(message != RT_NULL);

    struct mhscpu_spi *spi_drv =  rt_container_of(device->bus, struct mhscpu_spi, spi_bus);
    struct mhscpu_hw_spi_cs *cs = device->parent.user_data;

    if (message->cs_take && !(device->config.mode & RT_SPI_NO_CS))
    {
		rt_pin_write(cs->pin, PIN_LOW);
    }

    LOG_D("%s transfer prepare and start", spi_drv->config->bus_name);
    LOG_D("%s sendbuf: %X, recvbuf: %X, length: %d",
          spi_drv->config->bus_name,
          (uint32_t)message->send_buf,
          (uint32_t)message->recv_buf, message->length);

    message_length = message->length;
    recv_buf = message->recv_buf;
    send_buf = message->send_buf;
    while (message_length)
    {
        /* the HAL library use uint16 to save the data length */
        if (message_length > 65535)
        {
            send_length = 65535;
            message_length = message_length - 65535;
        }
        else
        {
            send_length = message_length;
            message_length = 0;
        }

        /* calculate the start address */
        already_send_length = message->length - send_length - message_length;
        send_buf = (rt_uint8_t *)message->send_buf + already_send_length;
        recv_buf = (rt_uint8_t *)message->recv_buf + already_send_length;
        
		for(uint32_t i=0;i<send_length;i++)
		{
			uint32_t timeout = 10000;
			if(message->send_buf)
				SPI_SendData(spi_drv->config->Instance, send_buf[i]);
			else
				SPI_SendData(spi_drv->config->Instance, 0xFF);
			
			while(RESET == SPI_GetFlagStatus(spi_drv->config->Instance, SPI_FLAG_RXNE) && timeout)timeout--;
			
			if(timeout)
			{
				uint8_t tmp = SPI_ReceiveData(spi_drv->config->Instance);
				if(message->recv_buf)recv_buf[i] = tmp;
			}
			else
			{
				state = -1;
				break;
			}
		}

        if (state != 0)
        {
            LOG_E("spi transfer error : %d", state);
            message->length = 0;
        }
        else
        {
            LOG_D("%s transfer done", spi_drv->config->bus_name);
        }
    }

    if (message->cs_release && !(device->config.mode & RT_SPI_NO_CS))
    {
		rt_pin_write(cs->pin, PIN_HIGH);
    }

    return message->length;
}

static rt_err_t spi_configure(struct rt_spi_device *device,
                              struct rt_spi_configuration *configuration)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(configuration != RT_NULL);

    struct mhscpu_spi *spi_drv =  rt_container_of(device->bus, struct mhscpu_spi, spi_bus);
    spi_drv->cfg = configuration;

    return mhscpu_spi_init(spi_drv, configuration);
}

static const struct rt_spi_ops stm_spi_ops =
{
    .configure = spi_configure,
    .xfer = spixfer,
};

static int rt_hw_spi_bus_init(void)
{
    rt_err_t result;
    for (int i = 0; i < sizeof(spi_config) / sizeof(spi_config[0]); i++)
    {
        spi_bus_obj[i].config = &spi_config[i];
        spi_bus_obj[i].spi_bus.parent.user_data = &spi_config[i];

        result = rt_spi_bus_register(&spi_bus_obj[i].spi_bus, spi_config[i].bus_name, &stm_spi_ops);
        RT_ASSERT(result == RT_EOK);

        LOG_D("%s bus init done", spi_config[i].bus_name);
    }

    return result;
}

/**
  * Attach the spi device to SPI bus, this function must be used after initialization.
  */
rt_err_t rt_hw_spi_device_attach(const char *bus_name, const char *device_name, rt_base_t cs_pin)
{
    RT_ASSERT(bus_name != RT_NULL);
    RT_ASSERT(device_name != RT_NULL);

    rt_err_t result;
    struct rt_spi_device *spi_device;
    struct mhscpu_hw_spi_cs *pin;

    /* initialize the cs pin && select the slave*/
	rt_pin_mode(cs_pin, PIN_MODE_OUTPUT);

    /* attach the device to spi bus*/
    spi_device = (struct rt_spi_device *)rt_malloc(sizeof(struct rt_spi_device));
    RT_ASSERT(spi_device != RT_NULL);
    pin = (struct mhscpu_hw_spi_cs *)rt_malloc(sizeof(struct mhscpu_hw_spi_cs));
    RT_ASSERT(pin != RT_NULL);
    pin->pin = cs_pin;
    result = rt_spi_bus_attach_device(spi_device, device_name, bus_name, (void *)pin);

    if (result != RT_EOK)
    {
        LOG_E("%s attach to %s faild, %d\n", device_name, bus_name, result);
    }

    RT_ASSERT(result == RT_EOK);

    LOG_D("%s attach to %s done", device_name, bus_name);

    return result;
}

#if defined(BSP_SPI1_TX_USING_DMA) || defined(BSP_SPI1_RX_USING_DMA)
void SPI1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_SPI_IRQHandler(&spi_bus_obj[SPI1_INDEX].handle);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#if defined(BSP_USING_SPI1) && defined(BSP_SPI1_RX_USING_DMA)
/**
  * @brief  This function handles DMA Rx interrupt request.
  * @param  None
  * @retval None
  */
void SPI1_DMA_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&spi_bus_obj[SPI1_INDEX].dma.handle_rx);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#if defined(BSP_USING_SPI1) && defined(BSP_SPI1_TX_USING_DMA)
/**
  * @brief  This function handles DMA Tx interrupt request.
  * @param  None
  * @retval None
  */
void SPI1_DMA_TX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&spi_bus_obj[SPI1_INDEX].dma.handle_tx);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(BSP_USING_SPI1) && defined(BSP_SPI_USING_DMA) */

#if defined(BSP_SPI2_TX_USING_DMA) || defined(BSP_SPI2_RX_USING_DMA)
void SPI2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_SPI_IRQHandler(&spi_bus_obj[SPI2_INDEX].handle);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#if defined(BSP_USING_SPI2) && defined(BSP_SPI2_RX_USING_DMA)
/**
  * @brief  This function handles DMA Rx interrupt request.
  * @param  None
  * @retval None
  */
void SPI2_DMA_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&spi_bus_obj[SPI2_INDEX].dma.handle_rx);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#if defined(BSP_USING_SPI2) && defined(BSP_SPI2_TX_USING_DMA)
/**
  * @brief  This function handles DMA Tx interrupt request.
  * @param  None
  * @retval None
  */
void SPI2_DMA_TX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&spi_bus_obj[SPI2_INDEX].dma.handle_tx);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(BSP_USING_SPI2) && defined(BSP_SPI_USING_DMA) */


int rt_hw_spi_init(void)
{
    return rt_hw_spi_bus_init();
}
INIT_BOARD_EXPORT(rt_hw_spi_init);

#endif /* BSP_USING_SPI1 || BSP_USING_SPI2 || BSP_USING_SPI3 || BSP_USING_SPI4 || BSP_USING_SPI5 */
#endif /* RT_USING_SPI */
