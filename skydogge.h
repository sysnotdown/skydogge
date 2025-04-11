
#pragma once
//无效
//#define PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS 1
//#define PICO_FLASH_4M

//当使用4Mflash写数据时，似乎会失败。可能不是4Mb,而是2Mb的板子。
#if defined (PICO_FLASH_4M)
#if defined PICO_FLASH_SIZE_BYTES
#undef PICO_FLASH_SIZE_BYTES
#endif
#define PICO_FLASH_SIZE_BYTES (4*1024*1024)
#endif

#define USE_GPS
//指南针设备二选一
//#define USE_QMC5883 //通常，这个指南针附加在GPS模块上。现在也配置在飞控上。
//#define USE_HMC5883 //在v8飞控上新增hmc5883.

//遥控通信设备可选。
#define USE_REMOTE_CTRL

//如果定义了GPS，选择pio方式接收或uart接收。2选一
#ifdef USE_GPS
//#define USE_GPS_PIO
#define USE_GPS_UART //uart1/0可选。v8板载两个uart口都引出，一个是GPS，一个是通讯遥控
//需要全自动运行时在这里打开，全自动必须带GPS
//全自动运行可以不带遥控模块。即屏蔽掉USE_REMOTE_CTRL, 可以不带测距模块，光流模块，即可屏蔽USE_TOF/USE_OFLOW
//#define FULL_AUTO_RUN
#endif


#ifdef USE_REMOTE_CTRL
//两种通信芯片二选一，代码合并略有差异。
//#define USE_E32_UART_SX1278
//#define USE_E34_UART_NRF24 //uart1
#define USE_DUAL_LORA //uart1/0可选。v8板载两个uart口都引出，一个是GPS，一个是通讯遥控
#endif

//imu/baro是必须要选的
#define USE_ICM426XX //含42605和42688支持
#define USE_BMP390  //气压计

//tof在板子上只能用扩展口，使用ext1扩展口，pio-uart方式读取数据。因i2c没有中断信号，不方便中断处理。
//v8板子有一个四针扩展口其实是三针接口，连接pin15，就是给tof预留的端口。
#define USE_TOF //pio1-sm0
//MTF01有光流，MT01只有测距没有光流，如果不定义这个，则即使是使用MTF01也不读取光流数据。
//#define USE_OFLOW


//电机负担的上下边界
//single motor thrust limit
#define TOPDUTY (0.999) 
#define BOTDUTY (0.080) //2306 0.15 2308 0.1, 2306如果最低发力0.15，在整机700克时降落速度就在6米左右，主要是推力太大。

//空中最低推力，根据机身重量不同而定。绝不能高于飞机保持平衡的推力，否则会一直向上飞。
//保持这个最低推力是防止推力意外降低到一个很小的值导致坠机。只有在降落时，推力才能低于这个值。
//空中平衡推力是1.6时，可设置1.2，平衡推力是1.3时，可设置为1.0.
//kv1400电机平衡推力可以低至1.1，更大的kv值更低。
//tmotor 2207v3 kv1750 7寸三叶，估计有更低的起飞油门，暂时设置一个更低的数值。之前0.8，现设置0.65
//换用2叶8寸桨对比3叶8寸差别不大，但换用9寸2叶明显油门降低，电流减小，悬停油门低至0.8，这么低的悬停油门，控高表现反而不好。

//single thrust in range 0-1.0, total thrust in range 0-4.0
//total thrust low limit
//最低0.8的推力在2806 kv900上空机降落速度只能达到7米左右，必须要降低才可以增大速度。暂时改为0.7，必要时还可以改低。
//由于新电机2306.5，1500kv属于较大kv值，其起飞平衡推力是0.85左右，适度降低这个最低推力可以使得下降速度更快。0.65只能使速度达到大约7米/秒
#define INAIR_BOTTHRUST (0.55) //要保证这个推力远低于平衡推力。2306 ->0.7 2806 ->0.7

//3.6的上限推力似乎大了，当它大了以后，平衡方面很难抵抗稍微大一点的风，因为电机差力出不来。
//比如前电机1.0，1.0，后电机只能是0.8，0.8，这种差异很难抵抗稍大的风力，保持姿态控制。
//感觉最大推力设置在3.0比较合适。当一侧为1.0时，另一侧可以低至0.6，有一定的姿态余量。
//total thrust up limit
#define INAIR_TOPTHRUST (3.0) //垂直推力上限，给平衡调节留有一定余地。


//水平速度极限，不宜太大，耗电太多，如果有电流限制，可以适度增大这个值，顺风时可以跑的更快更经济。
//如果没有电流限制，则以这个速度为目标速度，尽可能匹配这个速度。逆风时可能耗电极大。
#define HORI_SPD_LMT (20.0) 

//roll方向的转动惯量相对于pitch方向的比例值。如果电池纵向布置，这个值小于1.
#define ROLL_ADJUST_RATIO (0.95) //要考虑带摄像头的情况，本来是1.0，带摄像头要考虑降低为0.92
//#define ROLL_ADJUST_RATIO (0.8)  //新的长向布局。

//这个未来考虑放进配置表里，BALM
//#define BALANCE_RATIO (100)  //75适合2306 碳纤架， 100适合 2806碳纤架
//#define BALANCE_RATIO (1.0) //2306.5 碳纤架  20250323 缩小百倍

//这两个也可以考虑放进配置表里, BARH, TOFH
#define BARO_HEIGHT_RATIO (0.12) //气压控高主参数
#define TOF_HEIGHT_RATIO (0.10) //测距控高主参数

#define LED_PIN (25)
#define CURRENT_PIN (26) //接电调的电流表, 老板子是29, 新板子26
#define VOLTAGE_PIN (28) //板上电压接线

#if defined FULL_AUTO_RUN && !defined USE_GPS
#error auto run need gps
#endif

#include <stdio.h>
#include <vector>
#include <fstream>
#include <math.h>

#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "hardware/flash.h"
//#include "pico/stdio_usb.h" //20241211+
#include "hardware/structs/usb.h" //20241211+
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "pico/platform.h"
#include "pico/multicore.h"


//#include "gps.h"
#include "gps_ublox.h"
//#include "tof400.h"
//#include "CMTF01.h" //替换老的tof模块
#include "CTofSenseF.h" //新的测距模块
#include "randqueue.h"
#include "micro_task.h"
#include "motor.h"
#include "micro_task_exec.h"
#include "macro_task_exec.h"
#include "power.h"
#include "press.h"
#include "CTimeDif.h"
#include "remote.h"
#include "CDual_Lora.h"  //自组通信模块，全双工lora
#include "CQMC5883.h"
#include "CHMC5883.h"

#include "Randqueue_origin.h" //日志的缓存需要原始的版本

#include "icm426xx.h"
#include "bmp390_spi.h"

//pio小程序自动生成的头文件
#if defined USE_TOF
#include "uart_rx.pio.h"
#endif

#include "tools.h"


//2,3在电路板上被占用，不能用了
//#define PIO_UART_RX_PIN_A 3
//#define PIO_RX_SERIAL_BAUD_A  115200
//#define PIO_UART_RX_PIN_B 2
//#define PIO_RX_SERIAL_BAUD_B  115200

//使用pio_i2c速度能更快
//这个定义在makefile里
#if defined USE_PIO_I2C
#include "i2c.pio.h"
#include "pio_i2c.h"
#endif


//led频率设置为1000，则参数如下
#define LED_TOP_V  62499
#define LED_FEQ_DIV 32



void led_fade_in();
void led_fade_out();
void led_blink(int times, int gap_ms);
float check_system_power_supply();
uint64_t get_time_us_mark();
uint32_t get_time_mark();
void logmessage(std::string str);
void write_log_to_flash();
void read_log_from_flash();
void Do_magnet_calibration();
void Do_user_calibration();
void Do_imu_calibration();


//初次检查方向的方法，
typedef enum {
    head_compass0=0, //使用板载指南针
    head_compass1=1, //使用GPS指南针
    head_north=2,  //机头指向北方
    head_south=3,
    head_east=4,
    head_west=5,
    head_east_south=6,
    head_east_north=7,
    head_west_north=8,
    head_west_south=9,
    head_none=10, //无方向盲目起飞
} initial_heading_method;