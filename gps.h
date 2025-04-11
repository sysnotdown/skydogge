#pragma once

#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "gps_info.h"

//时间：HHMMSS[毫秒，3位数], uint
//定位可用/不可用，bool
//纬度latitude：float 度数，经过换算
//经度longitude：float 度度，经过换算
//海拔高度：float, 单位米
//相对速度：float, 单位是米/秒，经过换算
//航向角度：float, 0-360度
//水平定位精度：float, 单位米


#define _gps_speed_valid (0x01) //速度有效
#define _gps_height_valid (0x01<<1)  //高度有效
#define _gps_direction_valid (0x01<<2) //方向有效
#define _gps_accurate_valid (0x01<<3) //精度有效
#define _gps_time_valid (0x01<<4)   //时间有效
#define _gps_position_valid (0x01<<5) //经纬度有效

//uart gps reader
//上电路板，限制使用uart1端口，接收口gp9，发送口gp8
//现在由于8口做bmp390的中断口，所以只使用9号口接收数据。pio0

//长方板，pin15为pio接收。
#define GPS_RX_PIN 15
#define GPS_TX_PIN 7
#define GPS_BUADRATE (115200)
#define GPS_UART_PORT (1)

// //小方板，外挂ebyte的gps.
// #define GPS_RX_PIN 1
// #define GPS_TX_PIN 0
// #define GPS_BUADRATE (115200)
// #define GPS_UART_PORT (0)

class CGPS {
public:
    bool Initialize(int uart_port=GPS_UART_PORT, int pin_rx=GPS_RX_PIN, int pin_tx=GPS_TX_PIN);
    bool Read(gps_info& info);//返回false无有效信息读取
    bool Pio_PutChar(char c, gps_info& info); //pio调用，输入一个字符数据，如果有完整的信息则返回

    //仅限于同属东经，北纬，不适合跨区计算
    //返回角度在0-359.99度之间。距离单位米
    //输入输出角度参数全部为度数。
    // static void dist_angle_by_gps(double lati_from, double logi_from, double lati_to, double logi_to, float& dist, float& angle)
    // {
    // #define ER 6380000
    // #define cratio  57.29578
    //     double dNorth = ER * ( lati_to - lati_from)/ cratio;
    //     double dEast = ER *(logi_to - logi_from) * cos(lati_from/ cratio)/ cratio;

    //     angle = atan2(dEast, dNorth); //弧度
    //     angle *= cratio; //转360度

    //     if (angle < 0) angle += 360;

    //     dist = sqrt(dNorth*dNorth + dEast * dEast);

    // #undef ER
    // #undef cratio
    // }

    //给出基础坐标，目标的航向角和距离，计算目标的GPS坐标点。
    //angle是航向角，指向北方为0
    //输入输出角度参数全部为度数。
    // static void gps_by_dist_angle(double lati_base, double logi_base, float dist, float angle, double& lati, double& logi)
    // {
    // #define ER 6380000
    // #define cratio  57.29578
    //     float dNorth = cos(angle/ cratio)*dist;
    //     float dEast = sin(angle / cratio)*dist;
    //     float an = asin(dNorth / ER);
    //     float ae = asin(dEast / (cos(lati_base/ cratio)*ER));
    //     lati = lati_base + an * cratio;
    //     logi = logi_base + ae * cratio;
    // #undef ER
    // #undef cratio
    // }

    //国科威的GPS芯片，即从ebyte买的E108，不能保存配置，只能每次启动去配置，
    //这是配置命令接口。因为需要配置，所以用这个GPS需要占用uart端口，不能使用pio
    void SendPGKC(std::string cmd) {
        //uint8_t cmd[]="$PGKC462*3F<CR><LF>\r\n";
        //uart_write_blocking(uart0, cmd, 21);

        //char cmd[]= "$PGKC101,1000*02<CR><LF>";
        //char cmd[]= "$PGKC101,200*31<CR><LF>"; //200毫秒一个数据。
        uart_puts(uart0, cmd.c_str());
    };
private:
    int InfoLineParse(std::string& line, gps_info& info, uint8_t& infobits);
    std::string _rbuf;
    gps_info _inclass_buf;
    uint8_t _validbit;
    int _uart_port;
};


