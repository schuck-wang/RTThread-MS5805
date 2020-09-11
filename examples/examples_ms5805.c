/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-09-11     Administrator       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "sensor_meas_ms5805.h"

int ms5805_port(void)
{
    struct rt_sensor_config cfg;
    cfg.intf.dev_name="i2c1";
    rt_hw_ms5805_init("ms5805", &cfg);
    return RT_EOK;
}
INIT_COMPONENT_EXPORT(ms5805_port);

static void read_ms5805_entry(void *args)
{
    rt_device_t sensor_baro = RT_NULL;
    rt_device_t sensor_temp = RT_NULL;
    static struct rt_sensor_data data;
    sensor_baro = rt_device_find("baro_ms5805");
    if (!sensor_baro)
    {
        rt_kprintf("Can't find baro device.\n");
        return;
    }

    if (rt_device_open(sensor_baro, RT_DEVICE_FLAG_RDWR))
    {
        rt_kprintf("Open baro device failed.\n");
        return;
    }
    sensor_temp = rt_device_find("baro_ms5805");
    if (!sensor_temp)
    {
        rt_kprintf("Can't find temp device.\n");
        return;
    }

    if (rt_device_open(sensor_temp, RT_DEVICE_FLAG_RDWR))
    {
        rt_kprintf("Open temp device failed.\n");
        return;
    }
    while (1)
    {
        if(rt_device_read(sensor_baro, 0, &data, 1)==1)
        {
             rt_kprintf("baro: %5d.%d, timestamp:%5d\n", data.data.baro/1000,data.data.baro%1000, data.timestamp);
        }
        if(rt_device_read(sensor_temp, 0, &data, 1)==1)
        {
              rt_kprintf("temp: %5d.%d, timestamp:%5d\n", data.data.temp,data.data.temp/10, data.timestamp);
        }
        rt_thread_mdelay(3000);
    }
    rt_device_close(sensor_baro);
    rt_device_close(sensor_temp);
}
static int ms5805_read_sample(void)
{
    rt_thread_t ms5805_thread;

    ms5805_thread = rt_thread_create("ms5805_th", read_ms5805_entry,
                                   NULL, 1024,
                                    RT_THREAD_PRIORITY_MAX / 2, 20);
    if (ms5805_thread)
        rt_thread_startup(ms5805_thread);
    return 0;
}
INIT_APP_EXPORT(ms5805_read_sample);
