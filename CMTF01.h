#pragma once
#include "tof_oflow_info.h"

//v8版本飞控使用Pin15, 之前是使用Pin0
#define TOF_OFLOW_RX_PIN (15)

#define TOF_OFLOW_BUADRATE (115200)

//5v下的近距离表现更好，所以用5v电压。
//目前光流的安装方向是，机器向前y为负数，向后y为正数。
//向左x为负值，向右x为正值。即x轴和系统相同，y轴和系统相反。

//由于改用MT01,代码也改为支持MT01,差别是没有光流数据。
class CMTF01
{
    public:
    CMTF01() {
        _rbuf_tail=0;
    }

    bool Pio_PutChar(char c, tof_oflow_info& info);

    protected:
    char _rbuf[32];
    size_t _rbuf_tail;
};