/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-27     balanceTWK   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define DBG_TAG                        "main"
#define DBG_LVL                        DBG_INFO
#include <rtdbg.h>

int main(void)
{
    while (1)
    {
        rt_thread_mdelay(500);
    }
}
