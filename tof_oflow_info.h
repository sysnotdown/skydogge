
//tof测距，光流集合信息。
#pragma once

#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include "pico/stdlib.h"


class tof_oflow_info
{
public:
    uint32_t tmark;
    uint8_t tof_strengh; //信号强度。越大接收强度越高 20231225 放开以检测雪地测距的奇怪问题。
    float tof_distance; //原始测距数据
    float fixed_tof_distance; //倾角修正后的距离
    float yaw; //新增，记录数据来时的yaw角度。
    
    bool tof_valid;
    
    #if defined USE_OFLOW
    bool oflow_valid;
    uint8_t oflow_quality; //光流数据质量
    float fixed_oflow_spdx; //倾角修正后的速度
    float fixed_oflow_spdy; //倾角修正后的速度
    float sumx;
    float sumy;
    #endif

    tof_oflow_info() {
        tmark=0;
        tof_valid=false;
        fixed_tof_distance=0;
        tof_distance=0;
        tof_strengh=0;
        yaw=0;
        #if defined USE_OFLOW
        oflow_valid=false;
        fixed_oflow_spdx=0;
        fixed_oflow_spdy=0;
        oflow_quality=0;
        sumx=0;
        sumy=0;
        #endif
    }

    tof_oflow_info& operator=(const tof_oflow_info& o)
    {
        tmark=o.tmark;
        tof_valid=o.tof_valid;
        fixed_tof_distance=o.fixed_tof_distance;
        tof_distance=o.tof_distance;
        yaw=o.yaw;
        tof_strengh=o.tof_strengh;
        #if defined USE_OFLOW
        oflow_quality= o.oflow_quality;
        oflow_valid=o.oflow_valid;
        fixed_oflow_spdx= o.fixed_oflow_spdx;
        fixed_oflow_spdy= o.fixed_oflow_spdy;
        sumx=o.sumx;
        sumy=o.sumy;
        #endif
        return *this;
    } 
};
