/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-18     Administrator       the first version
 */
#ifndef __MS5805_H__
#define __MS5805_H__

#include <rtthread.h>

/* the macro below will enable second order temperature compensation */
#define CFG_MS5805_USING_TEMPERATURE_COMPENSATION
/* the macro below will enable ms5611 PROM data crc check */
#define CFG_MS5805_CRC_ENABLE

#define MS5805_I2C_ADDR_CSB      (0xEC>>1)
//#define MS5611_I2C_ADDR_CSB_H   (0xEE >> 1)

/* Oversampling Ratio */
enum ms5805_osr_e
{                             //conversion time (max)
     MS5805_OSR_256 = 0,      //0.54ms
     MS5805_OSR_512 = 2,      //1.06ms
     MS5805_OSR_1024 = 4,     //2.08ms
     MS5805_OSR_2048 = 6,     //4.13ms
     MS5805_OSR_4096 = 8,     //8.22ms
     MS5805_OSR_8192 = 10     //16.44ms

};

#define MS5805_OSR_DEFAULT      MS5805_OSR_4096

struct ms5805_param
{
     uint16_t cx[8];
     enum ms5805_osr_e osr_d1;  //pressure
     enum ms5805_osr_e osr_d2;  //temperature
};

struct ms5805_device
{
    struct rt_device *bus;
    struct ms5805_param ref;
    uint8_t i2c_addr;
    int32_t temp;   //range: -4000 - 8500  (2007=20.07`C)
    int32_t baro;   //range: 1000 - 200000 (100009=1000.09mbar)
    rt_mutex_t  measure_mutex;
};
typedef struct ms5805_device *ms5805_device_t;

ms5805_device_t ms5805_init(const char *dev_name, uint8_t param);
void ms5805_measure(ms5805_device_t dev);
int ms5805_reset_and_load_calibration(ms5805_device_t dev);
void ms5805_deinit(ms5805_device_t dev);

#endif /* __MS5805_H__ */
