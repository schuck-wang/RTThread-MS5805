/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-19     Administrator       the first version
 */
#ifndef __SENSOR_MEAS_MS5805_H__
#define __SENSOR_MEAS_MS5805_H__

#include "sensor.h"
#include "ms5805.h"

#define MS5805_USING_BARO
#define MS5805_USING_TEMP
int rt_hw_ms5805_init(const char *name, struct rt_sensor_config *cfg);

#endif /*__SENSOR_MEAS_MS5805_H__*/
