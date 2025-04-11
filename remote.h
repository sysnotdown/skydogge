#pragma once

#include <math.h>
#include "pico/stdlib.h"

class remote_info
{
    public:
    uint32_t tmark; //系统时间戳，代表收到信息的时间
    int8_t pos1, pos2, pos3, pos4; //四个摇杆位置，已带方向有符号。
    remote_info() {
        tmark=0;
        pos1=0;
        pos2=0;
        pos3=0;
        pos4=0;
    }

    remote_info& operator=(const remote_info& o)
    {
        tmark=o.tmark;
        pos1=o.pos1;
        pos2=o.pos2;
        pos3=o.pos3;
        pos4=o.pos4;
        return *this;
    }

    bool operator==(const remote_info& o)
    {
        if(pos1!=o.pos1) return false;
        if(pos2!=o.pos2) return false;
        if(pos3!=o.pos3) return false;
        if(pos4!=o.pos4) return false;  
        return true;            
    }

    bool operator!=(const remote_info& o)
    {
        if(*this==o) return false;
        else return true;
    }
};

