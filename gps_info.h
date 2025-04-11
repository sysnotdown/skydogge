#pragma once

#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

class gps_info{
public:
    gps_info() {
        gpstime=0;
        latitude=0;
        longitude=0;
        height=0;
        speed=0;
        speed_east=0;
        speed_north=0;
        speed_down=0;
        direction=0;
        hacc=0;
        vacc=0;
        ns=0;
        tmark=0;
    }


    uint gpstime;
    double latitude;//纬度
    double longitude;//经度
    float height;
    float speed;
    float speed_north; //北向速度
    float speed_east; //东向速度
    float speed_down; //下降速度  nmea协议暂时没找到这个数据
    float direction;
    float hacc; //水平定位精度
    float vacc; //垂直定位精度，nmea协议暂时不填写这个数据。
    uint8_t ns; //卫星数量
    uint32_t tmark; //系统时间戳，从启动以来的毫秒数。


    gps_info& operator=(const gps_info& o)
    {
        gpstime=o.gpstime;
        latitude=o.latitude;
        longitude=o.longitude;
        height=o.height;
        speed=o.speed;
        speed_north=o.speed_north;
        speed_east=o.speed_east;
        speed_down=o.speed_down;
        direction=o.direction;
        hacc=o.hacc;
        vacc=o.vacc;
        ns=o.ns;
        tmark=o.tmark;
        return *this;
    }
};