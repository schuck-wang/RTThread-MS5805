/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-19     Administrator       the first version
 */
#ifndef __MEAS_MS5805_SENSOR_V1_H__
#define __MEAS_MS5805_SENSOR_V1_H__
#include <rtdevice.h>

#if defined(RT_VERSION_CHECK)
    #if (RTTHREAD_VERSION >= RT_VERSION_CHECK(5, 0, 2))
        #define RT_SIZE_TYPE   rt_ssize_t
    #else
        #define RT_SIZE_TYPE   rt_size_t
    #endif
#endif
#include "ms5805.h"

int rt_hw_ms5805_init(const char *name, struct rt_sensor_config *cfg);

#endif /*__MEAS_MS5805_SENSOR_V1_H__*/
