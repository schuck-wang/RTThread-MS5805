# MS5805

## 简介

本软件包是为 MEAS MS5805 气压传感器提供的通用传感器驱动包。通过使用此软件包，开发者可以快速的利用 RT-Thread 将此传感器驱动起来。

本篇文档主要内容如下：

- 传感器介绍
- 支持情况
- 使用说明

## 目录结构 

| 名称     | 说明                         |
| -------- | ---------------------------- |
| docs     | 文档目录                     |
| examples | 例子目录，并有相应的一些说明 |
| inc      | 头文件目录                   |
| src      | 源代码目录                   |

## 传感器介绍

MS5805 是 MEAS（瑞士）推出的一款  IIC 总线接口的新一代高分辨率气压传感器

## 支持情况

| 包含设备         | 气压计 | 温度计 |
| ---------------- | -------- | ------ |
| **通讯接口**     |          |        |
| IIC              | √        | √      |
| **工作模式**     |          |        |
| 轮询             | √        | √      |
| 中断             |          |        |
| FIFO             |          |        |
| **电源模式**     |          |        |
| 掉电             |          |        |
| 低功耗           |          |        |
| 普通             | √        | √      |
| 高功耗           |          |        |
| **数据输出速率** |         |       |
| **测量范围**     | 1000~200000Pa | -40~85℃       |
| **自检**         |          |        |
| **多实例**       | √        | √      |

## 使用说明

### 依赖

- RT-Thread 4.0.0+
- Sensor 组件
- IIC  驱动

### 获取软件包

使用 MS5805 软件包需要在 RT-Thread 的包管理中选中它，具体路径如下：

```
MS5805: MS5805 Digital pressure sensor
    [*]   Enable MS5805 pressure
    [*]   Enable MS5805 temperature
    [*]   Enable MS5805 example
        Version (latest)  --->
```

**Enable MS5805 pressure**： 配置开启大气压强测量功能

**Enable MS5805 temperature**：配置开启温度测量功能

**Version**：软件包版本选择

### 使用软件包

MS5805 软件包初始化函数如下所示：

```
int rt_hw_ms5805_init(const char *name, struct rt_sensor_config *cfg);
```

该函数需要由用户调用，函数主要完成的功能有，

- 设备配置和初始化（根据传入的配置信息，配置接口设备）；
- 注册相应的传感器设备，完成 ms805 设备的注册；

#### 初始化示例

```
#include "sensor_meas_ms5805.h"

int ms5805_port(void)
{
    struct rt_sensor_config cfg;
    cfg.intf.dev_name = "i2c1";
    rt_hw_ms5805_init("ms5805", &cfg);
    return 0;
}
INIT_APP_EXPORT(ms5805_port);
```

## 注意事项

- 使用 IIC 通信接口时，无需设置 IIC 地址（也暂不支持），驱动会自行确认


## 联系人信息

维护人:

- [schuck-wang](https://github.com/schuck-wang)

## 感谢

在此感谢[sogwms](https://github.com/sogwms)提供的MS5611驱动，本驱动是在此基础上修改