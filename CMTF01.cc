#include "CMTF01.h"
#include <memory.h>
#include "skydogge.h"

extern uint32_t get_time_mark();

bool CMTF01::Pio_PutChar(char c, tof_oflow_info& info)
{
    //数据包帧头是0xEF
    if(_rbuf_tail==0 && c!=0xEF) return false;

    _rbuf[_rbuf_tail++]=c;

    if(_rbuf_tail >=6) {
        uint8_t devid= _rbuf[1];
        uint8_t paylen= _rbuf[5];
        if(devid!=0x0F) //设备id都是0x0f
        {
            _rbuf_tail=0;
            //printf("devid!=0x0f\n");
            return false;
        }

        if(paylen>=32)
        {
            _rbuf_tail=0;
            //printf("paylen>=32\n");
            return false;
        }

        if(_rbuf_tail >= size_t(7+paylen))
        {
            uint8_t chksum= _rbuf[7+paylen-1];
            uint8_t check=0;
            for(size_t i=0;i< size_t(6+paylen);i++)
            {
                check+= _rbuf[i];
            }

            if(check!=chksum) {
                //偶有发生。
                _rbuf_tail=0;
                //printf("check sum fail, paylen=%d\n", paylen);
                return false;
            }

            //check ok.
            uint32_t dist;
            memcpy(&dist, _rbuf+10, 4);
            uint8_t tof_sig_strenth= _rbuf[14]; //信号强度
            uint8_t tofv=_rbuf[16]; //1有效0无效

#if defined USE_OFLOW
            int16_t fx,fy;
            memcpy(&fx, _rbuf+18, 2);
            memcpy(&fy, _rbuf+20, 2);
            uint8_t fq=_rbuf[22];  //光流质量
            uint8_t fv=_rbuf[23];  //1有效0无效

            //经过转换后，这个速度可以看成是一个角速度，单位是弧度。
            //如果速度是1m/s可以看成是1弧度/秒的角速度。
            info.fixed_oflow_spdx= float(fx)/100.0; //原始数据cm/s @ 1m 换算为 m/s @1m
            info.fixed_oflow_spdy= float(fy)/100.0;

            info.oflow_quality = fq;
            info.oflow_valid = fv;
#endif
            info.tof_distance= float(dist)/1000.0f; //原始数据毫米，换算为米。
            info.fixed_tof_distance= info.tof_distance;
            //在避免tofsense测距固件问题时，新引入了info.tof_distance，但这边没有修改
            //导致测距有效但始终是0，导致降落时直接停机。 现修正。20241212.
            //info.fixed_tof_distance= float(dist)/1000.0f; //原始数据毫米，换算为米。
            info.tof_strengh=tof_sig_strenth; //为节约空间，取消了tof信号强度
            info.tof_valid=tofv;
            info.tmark= get_time_mark();

            if(_rbuf_tail > 7+ size_t(paylen))
            {
                memmove(_rbuf, _rbuf+7+paylen, _rbuf_tail-7-paylen);
                _rbuf_tail-= 7+paylen;
            }
            else
            {
                _rbuf_tail=0;
            }

            return true;
        }
    }
    else
    {
        return false;
    }

    
    return false;
}