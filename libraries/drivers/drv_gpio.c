/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2018-11-06     balanceTWK        first version
 * 2019-04-23     WillianChan       Fix GPIO serial number disorder
 * 2020-06-16     thread-liu        add STM32MP1
 * 2020-09-01     thread-liu        add GPIOZ
 * 2020-09-18     geniusgogo        optimization design pin-index algorithm
 */

#include <board.h>
#include "drv_gpio.h"

#ifdef RT_USING_PIN

#define PIN_NUM(port, no) (((((port) & 0xFu) << 4) | ((no) & 0xFu)))
#define PIN_PORT(pin) ((uint8_t)(((pin) >> 4) & 0xFu))
#define PIN_NO(pin) ((uint8_t)((pin) & 0xFu))

#if defined(SOC_SERIES_MH1902)
#define PIN_MHSPORT(pin) ((GPIO_TypeDef *)(GPIO_BASE + (0x0010UL * PIN_PORT(pin))))
#endif /* SOC_SERIES_MH1902 */

#define PIN_MHSPIN(pin) ((uint16_t)(1u << PIN_NO(pin)))

#define ITEM_NUM(items) sizeof(items) / sizeof(items[0])


static const struct port_irq_map port_irq_map[] = 
{
	{GPIOA, EXTI_Line0, EXTI0_IRQn},
	{GPIOB, EXTI_Line1, EXTI1_IRQn},
	{GPIOC, EXTI_Line2, EXTI2_IRQn},
	{GPIOD, EXTI_Line3, EXTI3_IRQn}
};

static struct rt_pin_irq_hdr pin_irq_hdr_tab[ITEM_NUM(port_irq_map)][16] =
{
    [0] = 
	{
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
	},
	
	[1] = 
	{
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
	},
	
	[2] = 
	{
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
	},
	
	[3] = 
	{
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
		{-1, 0, RT_NULL, RT_NULL},
	},
};

static rt_base_t mhscpu_pin_get(const char *name)
{
    rt_base_t pin = 0;
    int hw_port_num, hw_pin_num = 0;
    int i, name_len;

    name_len = rt_strlen(name);

    if ((name_len < 4) || (name_len >= 6))
    {
        return -RT_EINVAL;
    }
    if ((name[0] != 'P') || (name[2] != '.'))
    {
        return -RT_EINVAL;
    }

    if ((name[1] >= 'A') && (name[1] <= 'Z'))
    {
        hw_port_num = (int)(name[1] - 'A');
    }
    else
    {
        return -RT_EINVAL;
    }

    for (i = 3; i < name_len; i++)
    {
        hw_pin_num *= 10;
        hw_pin_num += name[i] - '0';
    }

    pin = PIN_NUM(hw_port_num, hw_pin_num);

    return pin;
}

static void mhscpu_pin_write(rt_device_t dev, rt_base_t pin, rt_base_t value)
{
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_pin;

    if (PIN_PORT(pin) < ITEM_NUM(port_irq_map))
    {
        gpio_port = PIN_MHSPORT(pin);
        gpio_pin = PIN_MHSPIN(pin);

		//GPIO_WriteBit(gpio_port, gpio_pin, (BitAction)value);
		if(value) GPIO_SetBits(gpio_port, gpio_pin);
		else GPIO_ResetBits(gpio_port, gpio_pin);
    }
}

static int mhscpu_pin_read(rt_device_t dev, rt_base_t pin)
{
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_pin;
    int value = PIN_LOW;

    if (PIN_PORT(pin) < ITEM_NUM(port_irq_map))
    {
        gpio_port = PIN_MHSPORT(pin);
        gpio_pin = PIN_MHSPIN(pin);
		value = GPIO_ReadInputDataBit(gpio_port, gpio_pin);
    }

    return value;
}

static void mhscpu_pin_mode(rt_device_t dev, rt_base_t pin, rt_base_t mode)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    if (PIN_PORT(pin) >= ITEM_NUM(port_irq_map))
    {
        return;
    }

    /* Configure GPIO_InitStructure */
    GPIO_InitStruct.GPIO_Pin = PIN_MHSPIN(pin);
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Remap = GPIO_Remap_1;

    if (mode == PIN_MODE_OUTPUT)
    {
        /* output setting */
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    }
    else if (mode == PIN_MODE_INPUT)
    {
        /* input setting: not pull. */
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }
    else if (mode == PIN_MODE_INPUT_PULLUP)
    {
        /* input setting: pull up. */
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    }
    else if (mode == PIN_MODE_INPUT_PULLDOWN)
    {
        /* input setting: pull down. */
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }
    else if (mode == PIN_MODE_OUTPUT_OD)
    {
        /* output setting: od. */
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
    }

    GPIO_Init(PIN_MHSPORT(pin), &GPIO_InitStruct);
}

rt_inline rt_int32_t bit2bitno(rt_uint32_t bit)
{
    int i;
    for (i = 0; i < 32; i++)
    {
        if ((0x01 << i) == bit)
        {
            return i;
        }
    }
    return -1;
}

rt_inline const struct port_irq_map *get_port_irq_map(rt_base_t pin)
{
    rt_int32_t mapindex = PIN_PORT(pin);
    if (mapindex < 0 || mapindex >= ITEM_NUM(port_irq_map))
    {
        return RT_NULL;
    }
    return &port_irq_map[mapindex];
};

static rt_err_t mhscpu_pin_attach_irq(struct rt_device *device, rt_int32_t pin,
                                     rt_uint32_t mode, void (*hdr)(void *args), void *args)
{
    rt_base_t level;
    rt_int32_t irqindex = -1;

    if (PIN_PORT(pin) >= ITEM_NUM(port_irq_map))
    {
        return -RT_ENOSYS;
    }

    irqindex = bit2bitno(PIN_MHSPIN(pin));
    if (irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_hdr_tab[PIN_PORT(pin)]))
    {
        return RT_ENOSYS;
    }

    level = rt_hw_interrupt_disable();
    if (pin_irq_hdr_tab[PIN_PORT(pin)][irqindex].pin == pin &&
        pin_irq_hdr_tab[PIN_PORT(pin)][irqindex].hdr == hdr &&
        pin_irq_hdr_tab[PIN_PORT(pin)][irqindex].mode == mode &&
        pin_irq_hdr_tab[PIN_PORT(pin)][irqindex].args == args)
    {
        rt_hw_interrupt_enable(level);
        return RT_EOK;
    }
    if (pin_irq_hdr_tab[PIN_PORT(pin)][irqindex].pin != -1)
    {
        rt_hw_interrupt_enable(level);
        return RT_EBUSY;
    }
    pin_irq_hdr_tab[PIN_PORT(pin)][irqindex].pin = pin;
    pin_irq_hdr_tab[PIN_PORT(pin)][irqindex].hdr = hdr;
    pin_irq_hdr_tab[PIN_PORT(pin)][irqindex].mode = mode;
    pin_irq_hdr_tab[PIN_PORT(pin)][irqindex].args = args;
    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

static rt_err_t mhscpu_pin_dettach_irq(struct rt_device *device, rt_int32_t pin)
{
    rt_base_t level;
    rt_int32_t irqindex = -1;

    if (PIN_PORT(pin) >= ITEM_NUM(port_irq_map))
    {
        return -RT_ENOSYS;
    }

    irqindex = bit2bitno(PIN_MHSPIN(pin));
    if (irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_hdr_tab[PIN_PORT(pin)]))
    {
        return RT_ENOSYS;
    }

    level = rt_hw_interrupt_disable();
    if (pin_irq_hdr_tab[PIN_PORT(pin)][irqindex].pin == -1)
    {
        rt_hw_interrupt_enable(level);
        return RT_EOK;
    }
    pin_irq_hdr_tab[PIN_PORT(pin)][irqindex].pin = -1;
    pin_irq_hdr_tab[PIN_PORT(pin)][irqindex].hdr = RT_NULL;
    pin_irq_hdr_tab[PIN_PORT(pin)][irqindex].mode = 0;
    pin_irq_hdr_tab[PIN_PORT(pin)][irqindex].args = RT_NULL;
    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

static rt_err_t mhscpu_pin_irq_enable(struct rt_device *device, rt_base_t pin,
                                     rt_uint32_t enabled)
{
    const struct port_irq_map *irqmap;
    rt_base_t level;
    rt_int32_t irqindex = -1;
    GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_TriggerTypeDef EXTI_TriggerType;

    if (PIN_PORT(pin) >= ITEM_NUM(port_irq_map))
    {
        return -RT_ENOSYS;
    }

    if (enabled == PIN_IRQ_ENABLE)
    {
        irqindex = bit2bitno(PIN_MHSPIN(pin));
        if (irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_hdr_tab[PIN_PORT(pin)]))
        {
            return RT_ENOSYS;
        }

        level = rt_hw_interrupt_disable();

        if (pin_irq_hdr_tab[PIN_PORT(pin)][irqindex].pin == -1)
        {
            rt_hw_interrupt_enable(level);
            return RT_ENOSYS;
        }

        irqmap = get_port_irq_map(pin);

        /* Configure GPIO_InitStructure */
        GPIO_InitStruct.GPIO_Pin = PIN_MHSPIN(pin);
		GPIO_InitStruct.GPIO_Remap = GPIO_Remap_1;
        switch (pin_irq_hdr_tab[PIN_PORT(pin)][irqindex].mode)
        {
        case PIN_IRQ_MODE_RISING:
            GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			EXTI_TriggerType = EXTI_Trigger_Rising;
            break;
        case PIN_IRQ_MODE_FALLING:
            GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
			EXTI_TriggerType = EXTI_Trigger_Falling;
            break;
        case PIN_IRQ_MODE_RISING_FALLING:
            GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			EXTI_TriggerType = EXTI_Trigger_Rising_Falling;
            break;
        }
        GPIO_Init(PIN_MHSPORT(pin), &GPIO_InitStruct);

		EXTI_LineConfig(irqmap->line, PIN_MHSPIN(pin), EXTI_TriggerType);
		
        NVIC_SetPriority(irqmap->irqno, 0);
        NVIC_EnableIRQ(irqmap->irqno);

        rt_hw_interrupt_enable(level);
    }
    else if (enabled == PIN_IRQ_DISABLE)
    {
        irqmap = get_port_irq_map(pin);
        if (irqmap == RT_NULL)
        {
            return RT_ENOSYS;
        }

        level = rt_hw_interrupt_disable();

		GPIO_InitStruct.GPIO_Pin = PIN_MHSPIN(pin);
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStruct.GPIO_Remap = GPIO_Remap_1;
        GPIO_Init(PIN_MHSPORT(pin), &GPIO_InitStruct);

		EXTI_LineConfig(irqmap->line, PIN_MHSPIN(pin), EXTI_Trigger_Off);
		
		rt_bool_t closeIqr = RT_TRUE;
		for(uint8_t i=0;i<ITEM_NUM(pin_irq_hdr_tab[PIN_PORT(pin)]);i++)
		{
			if(pin_irq_hdr_tab[PIN_PORT(pin)][i].pin != -1)
			{
				closeIqr = RT_FALSE;
				break;
			}
		}
		
		if(closeIqr)
		{
			NVIC_DisableIRQ(irqmap->irqno);
		}
		
        rt_hw_interrupt_enable(level);
    }
    else
    {
        return -RT_ENOSYS;
    }

    return RT_EOK;
}
const static struct rt_pin_ops _stm32_pin_ops =
{
    mhscpu_pin_mode,
    mhscpu_pin_write,
    mhscpu_pin_read,
    mhscpu_pin_attach_irq,
    mhscpu_pin_dettach_irq,
    mhscpu_pin_irq_enable,
    mhscpu_pin_get,
};

rt_inline void pin_irq_hdr(int line, uint32_t status)
{
	for(uint8_t i=0;i<ITEM_NUM(pin_irq_hdr_tab[line]);i++)
	{
		if ((PIN_MHSPIN(pin_irq_hdr_tab[line][i].pin) & status) && pin_irq_hdr_tab[line][i].hdr)
		{
			pin_irq_hdr_tab[line][i].hdr(pin_irq_hdr_tab[line][i].args);
		}
	}
}

void EXTI0_IRQHandler(void)
{
	rt_interrupt_enter();
	
	uint32_t status = EXTI_GetITLineStatus(EXTI_Line0);
	EXTI_ClearITPendingBit(EXTI_Line0);
	NVIC_ClearPendingIRQ(EXTI0_IRQn);
	pin_irq_hdr(EXTI_Line0, status);
	
	rt_interrupt_leave();
}

void EXTI1_IRQHandler(void)
{
	rt_interrupt_enter();
	
	uint32_t status = EXTI_GetITLineStatus(EXTI_Line1);
	EXTI_ClearITPendingBit(EXTI_Line1);
	NVIC_ClearPendingIRQ(EXTI1_IRQn);
	pin_irq_hdr(EXTI_Line1, status);
	
	rt_interrupt_leave();
}

void EXTI2_IRQHandler(void)
{
	rt_interrupt_enter();
	
	uint32_t status = EXTI_GetITLineStatus(EXTI_Line2);
	EXTI_ClearITPendingBit(EXTI_Line2);
	NVIC_ClearPendingIRQ(EXTI2_IRQn);
	pin_irq_hdr(EXTI_Line2, status);
	
	rt_interrupt_leave();
}

void EXTI3_IRQHandler(void)
{
	rt_interrupt_enter();
	
	uint32_t status = EXTI_GetITLineStatus(EXTI_Line3);
	EXTI_ClearITPendingBit(EXTI_Line3);
	NVIC_ClearPendingIRQ(EXTI3_IRQn);
	pin_irq_hdr(EXTI_Line3, status);
	
	rt_interrupt_leave();
}

int rt_hw_pin_init(void)
{
	SYSCTRL_APBPeriphClockCmd(SYSCTRL_APBPeriph_GPIO, ENABLE);

    return rt_device_pin_register("pin", &_stm32_pin_ops, RT_NULL);
}

#endif /* RT_USING_PIN */
