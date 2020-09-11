/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-18     Administrator       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <finsh.h>

#define DBG_ENABLE
#define DBG_SECTION_NAME "MS5805"
#define DBG_LEVEL DBG_LOG
#include <rtdbg.h>

#include "ms5805.h"
#ifdef RT_USING_I2C
static int i2c_ms5805_send_cmd(ms5805_device_t dev, uint8_t cmd)
{
    int tmp;
    struct rt_i2c_msg msg;

    msg.addr = dev->i2c_addr;
    msg.flags = RT_I2C_WR;
    msg.buf = &cmd;
    msg.len = 1;
    tmp = rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msg, 1);

    return tmp;
}

static void i2c_ms5805_read(ms5805_device_t dev, uint8_t *pd, uint8_t len)
{
    struct rt_i2c_msg msg;

    msg.addr = dev->i2c_addr;
    msg.flags = RT_I2C_RD;
    msg.buf = pd;
    msg.len = len;
    rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msg, 1);
}

/**
 * This function sends command to ms5805
 *
 * @param dev the pointer of device driver structure
 * @param cmd command
 */
static void ms5805_send_cmd(ms5805_device_t dev, uint8_t cmd)
{
    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
        i2c_ms5805_send_cmd(dev, cmd);
    }

}

/**
 * This function sends command to ms5805
 *
 * @param dev the pointer of device driver structure
 * @param addr ms5611 internal address
 * @param pd the pointer of receive buffer
 * @param len the length of the byte to receive
 */
static void ms5805_recv_data(ms5805_device_t dev, uint8_t addr, uint8_t *pd, uint8_t len)
{
    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
        i2c_ms5805_send_cmd(dev, addr);
        i2c_ms5805_read(dev, pd, len);
    }


}

#ifdef CFG_MS5805_CRC_ENABLE
/**
 * Returns TRUE if crc passes, otherwise returns FALSE
 */
static int ms5805_crc4(uint16_t *n_prom)
{
    int16_t cnt;
    uint16_t n_rem;
    uint16_t crc_read;
    uint8_t n_bit;

    n_rem = 0x00;

    /* save the read crc */
    crc_read = n_prom[0]>>12;

    /* remove CRC byte */
    n_prom[0] = (0x0FFF & (n_prom[0]));

    for (cnt = 0; cnt < 16; cnt++)
    {
        /* uneven bytes */
        if (cnt %2==1)
        {
            n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);
        }
        else
        {
            n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
        }
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & 0x8000)
            {
                n_rem = (n_rem << 1) ^ 0x3000;
            }
            else
            {
                n_rem = (n_rem << 1);
            }
        }
    }

    /* final 4 bit remainder is CRC value */
    n_rem = (0x000F & (n_rem >> 12));
//    n_prom[0] = crc_read<<12| n_prom[0];
    /* return true if CRCs match */
    return (0x000F & crc_read) == (n_rem ^ 0x00);
}
#endif
/**
 * This function resets ms5805 and reads calibration data from the PROM.
 *
 * @param dev the pointer of device driver structure
 *
 * @return Returns 0 if all is ok, otherwise returns -1
 */
int ms5805_reset_and_load_calibration(ms5805_device_t dev)
{
    uint8_t tmp;
    uint8_t rtmp[2];
    int rvl = 0;

    //reset
    tmp = 0x1E;
    ms5805_send_cmd(dev, tmp);
    //wait for PROM contents to be in the device(2.8 ms)
    rt_thread_mdelay(1);
    //read PROM
    for (tmp = 0xA0; tmp < 0xAE;)
    {
        ms5805_recv_data(dev, tmp, rtmp, 2);
        dev->ref.cx[(tmp - 0xA0) / 2] = (rtmp[0] << 8) | rtmp[1];
        tmp += 2;
        rt_kprintf("C[%d]=%d\r\n",(tmp - 0xA0) / 2-1, dev->ref.cx[(tmp - 0xA0) / 2-1] );
    }

#ifdef CFG_MS5805_CRC_ENABLE
    if (ms5805_crc4(dev->ref.cx)==0)
    {
        rvl = -1;
        LOG_E("crc check failed!");
    }
#endif
    return rvl;
}

static void _waiting_convert_finish(int osr)
{
    switch (osr)
    {
        case MS5805_OSR_256:
            osr = 2;
            break;
        case MS5805_OSR_512:
            osr = 2;
            break;
        case MS5805_OSR_1024:
            osr = 4;
            break;
        case MS5805_OSR_2048:
            osr = 5;
            break;
        case MS5805_OSR_4096:
            osr = 10;
            break;
        case MS5805_OSR_8192:
            osr = 20;
            break;
        default:
            osr = 20;
    }
    rt_thread_mdelay(osr);
}
/**
 * This function reads raw data and then calculates temperature and pressure.
 *
 * @param dev the pointer of device driver structure
 *
 * @return the pressure value(unit: Pa)
 */
void ms5805_measure(ms5805_device_t dev)
{
    uint8_t tmp, rtmp[3];
    uint32_t d1, d2;
    int32_t dt;
    int32_t temp;
    int64_t off;
    int64_t sens;
#ifdef CFG_MS5805_USING_TEMPERATURE_COMPENSATION
    int64_t off2;
    int64_t sens2;
    int32_t t2;
#endif

    rt_mutex_take(dev->measure_mutex, RT_WAITING_FOREVER);

    //start D1 conversion
    tmp = 0x40 + dev->ref.osr_d1;
    ms5805_send_cmd(dev, tmp);
    _waiting_convert_finish(dev->ref.osr_d1);
    tmp = 0x00;     //get ADC value
    ms5805_recv_data(dev, tmp, rtmp, 3);
    d1 = (rtmp[0] << 16) | (rtmp[1] << 8) | rtmp[2];
    //start D2 conversion
    tmp = 0x50 + dev->ref.osr_d2;
    ms5805_send_cmd(dev, tmp);
    _waiting_convert_finish(dev->ref.osr_d2);
    tmp = 0x00;     //get ADC value
    ms5805_recv_data(dev, tmp, rtmp, 3);
    d2 = (rtmp[0] << 16) | (rtmp[1] << 8) | rtmp[2];

    /* start calculate pressure */
    dt = d2 - (long long)dev->ref.cx[5] * (2UL << 7);
    temp = 2000 + (dt * (long long)dev->ref.cx[6]) / (2UL << 22);
    off = ((long long)dev->ref.cx[2] * (2UL << 16))+ (((long long)dev->ref.cx[4] * dt) / (2UL << 5));
    sens = (long long)dev->ref.cx[1] * (2UL << 15) + ((long long)dev->ref.cx[3] * dt) / (2UL << 6);

#ifdef CFG_MS5805_USING_TEMPERATURE_COMPENSATION
    if (temp < 2000)
    {
        t2 = ((long long)dt * (long long)dt) / (2ULL << 34)*11;
        off2 = (31 * (temp - 2000) * (temp - 2000)) / (2UL << 2);
        sens2 = (63 * (temp - 2000) * (temp - 2000)) / (2UL << 4);

        temp = temp - t2;
        off = off - off2;
        sens = sens - sens2;
    }
#endif
    dev->temp = temp;
    dev->baro = (int32_t)((((d1 * sens)/(2UL << 20)) - off) / (2UL << 14));
	
    rt_mutex_release(dev->measure_mutex);
}

/**
 * This function initializes the ms5805 device.
 *
 * @param dev_name the name of transfer device
 * @param param the i2c device address for i2c communication
 *
 * @return the pointer of device driver structure, RT_NULL reprensents initialization failed.
 */
ms5805_device_t ms5805_init(const char *dev_name, uint8_t param)
{
    ms5805_device_t dev;

    uint8_t cmd;

    RT_ASSERT(dev_name);

    dev = rt_calloc(1, sizeof(struct ms5805_device));
    if (dev == RT_NULL)
    {
        LOG_E("Can't allocate memory for ms5805 device on '%s'", dev_name);
        goto __exit;
    }

    dev->bus = rt_device_find(dev_name);
    if (dev->bus == RT_NULL)
    {
        LOG_E("Can't find device on '%s'", dev_name);
        goto __exit;
    }

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
        if (param != RT_NULL)
        {
            dev->i2c_addr = param;
        }
        else
        {
            /* try a possible ms5805-i2c address */
            cmd = 0x1E;
            dev->i2c_addr = MS5805_I2C_ADDR_CSB;   //CSB pin is low
            if (i2c_ms5805_send_cmd(dev, cmd) != 1)
            {

                LOG_E("Can't find ms5805 device at '%s' (i2c-slave-address is wrong?)", dev_name);
                goto __exit;

            }
            LOG_I("Device(ms5805) i2c address is: '0x%x'!", dev->i2c_addr);
        }
    }


    dev->measure_mutex = rt_mutex_create("ms5805", RT_IPC_FLAG_FIFO);
    if (dev->measure_mutex == RT_NULL)
    {
        LOG_E("Can't create mutex");
        goto __exit;
    }
    dev->temp = 0;
    dev->ref.osr_d1 = MS5805_OSR_DEFAULT;
    dev->ref.osr_d2 = MS5805_OSR_DEFAULT;
	
	ms5805_reset_and_load_calibration(dev);

    LOG_I("ms5805 init finished");

    return dev;

__exit:
    if (dev != RT_NULL)
    {
        rt_free(dev);
    }

    return RT_NULL;
}

/**
 * This function releases memory.
 *
 * @param dev the pointer of device driver structure
 */
void ms5805_deinit(ms5805_device_t dev)
{
    RT_ASSERT(dev);

    rt_mutex_delete(dev->measure_mutex);
    rt_free(dev);
}
#endif  /* RT_USING_I2C */
