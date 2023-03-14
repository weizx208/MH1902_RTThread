/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     balanceTWK   first version
 */
 
#include "board.h"
#include "drv_spi.h"
#include "spi_flash_sfud.h"

void SystemClock_Config(void)
{
	SYSCTRL_PLLConfig(SYSCTRL_PLL_144MHz);
	SYSCTRL_HCLKConfig(SYSCTRL_HCLK_Div_None);
	SYSCTRL_PCLKConfig(SYSCTRL_PCLK_Div2);
	
	SYSCTRL_APBPeriphClockCmd(SYSCTRL_APBPeriph_GPIO, ENABLE);
	SYSCTRL_AHBPeriphClockCmd(SYSCTRL_AHBPeriph_DMA, ENABLE);
	SYSCTRL_AHBPeriphResetCmd(SYSCTRL_AHBPeriph_DMA, ENABLE);
	SYSCTRL_AHBPeriphClockCmd(SYSCTRL_APBPeriph_TIMM0, ENABLE);
	SYSCTRL_AHBPeriphResetCmd(SYSCTRL_APBPeriph_TIMM0, ENABLE);
	SYSCTRL_APBPeriphClockCmd(SYSCTRL_APBPeriph_UART0 | SYSCTRL_APBPeriph_UART1 | SYSCTRL_APBPeriph_UART2, ENABLE);
	SYSCTRL_APBPeriphResetCmd(SYSCTRL_APBPeriph_UART0 | SYSCTRL_APBPeriph_UART1 | SYSCTRL_APBPeriph_UART2, ENABLE);
	SYSCTRL_APBPeriphClockCmd(SYSCTRL_APBPeriph_SPI0 | SYSCTRL_APBPeriph_SPI1 | SYSCTRL_APBPeriph_SPI2, ENABLE);
	SYSCTRL_APBPeriphResetCmd(SYSCTRL_APBPeriph_SPI0 | SYSCTRL_APBPeriph_SPI1 | SYSCTRL_APBPeriph_SPI2, ENABLE);
	
	SYSCTRL->PHER_CTRL &= ~BIT(20);//smart cardµçÔ´¿ªÆô
	SystemCoreClock = 144000000;
}
