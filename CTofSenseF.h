
//mt-01升级后价格太高，新选了tofsense做测距。
//数据包不同。需要另写。20240828
#pragma once
#include "tof_oflow_info.h"

//v8版本飞控使用Pin15, 之前是使用Pin0
#define TOF_OFLOW_RX_PIN (15)
//#define TOF_OFLOW_BUADRATE (921600) //tofsense默认baud比较大
#define TOF_OFLOW_BUADRATE (115200) //经过配置后
class CTofSenseF
{
    public:
    CTofSenseF() {
        _rbuf_tail=0;
    }

    bool Pio_PutChar(char c, tof_oflow_info& info);

    protected:
    uint8_t _rbuf[32];
    size_t _rbuf_tail;
};