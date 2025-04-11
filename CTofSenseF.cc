#include "CTofSenseF.h"

#include <memory.h>
#include "skydogge.h"

extern uint32_t get_time_mark();
extern float _baro_height;
extern bool _inair;

bool CTofSenseF::Pio_PutChar(char c, tof_oflow_info& info)
{
    //数据包帧头是0x57
    if(_rbuf_tail==0 && c!=0x57) return false; //0x57 frame header

    _rbuf[_rbuf_tail++]=c;

    if(_rbuf_tail >=4) {
        uint8_t funcmark= _rbuf[1];
        uint8_t devid= _rbuf[3];
        if(devid!=0x00 ||funcmark!=0x00) 
        {
            _rbuf_tail=0;
            printf("devid fail\n");
            return false;
        }

        if(_rbuf_tail >= 16) //完整包可解析
        {

            uint8_t chksum= _rbuf[15];
            uint8_t check=0;
            for(size_t i=0;i< 15;i++)
            {
                check+= _rbuf[i];
            }

            if(check!=chksum) {
                //偶有发生。
                _rbuf_tail=0;
                printf("tofsense check sum fail\n");
                return false;
            }

            //check ok.
            uint32_t dist= uint32_t(_rbuf[8])+(uint32_t(_rbuf[9])<<8)+ (uint32_t(_rbuf[10])<<16);
            uint8_t tofv=_rbuf[11]; //1有效0无效

            uint16_t tof_sig_strengh= uint16_t(_rbuf[12]) + (uint16_t(_rbuf[13])<<8); //信号强度

            //info.fixed_tof_distance= float(dist)/1000.0f; //原始数据毫米，换算为米。
            info.tof_distance= float(dist)/1000.0f; //原始数据毫米，换算为米。
            info.fixed_tof_distance= info.tof_distance;
            info.tof_strengh=tof_sig_strengh; 
            info.tof_valid=tofv;
            info.tmark= get_time_mark();

            //防范错误！这个模块有时会出错，测距超出范围时会标记测距有效，距离为0.
            //也因此而炸过几次。mtf01模块就没问题。
            //20241130 还在900多米高处出现过测距有效，原因不明。因此，把有效测距的信号强度再提高到20，之前设置为10，避免误报。
            //20250224 为减少空中误报，信号强度<45不算有效测距，在光亮的地表，很远的距离都常有误报。在雪地里很可能误报非常严重。
            //气压高度越高，对测距要求越严格，以防止测距出现幻觉。
            //uint16_t thres=30; //正常20，高于50米测距全关。
 
            if(info.tof_valid && tof_sig_strengh < 15)
            {
                //测距标记有效但信号强度差的，也标记为无效。
                //printf("valid tof but strengh=%d, mark invalid\n", tof_sig_strengh);
                info.tof_valid=false;
            }
            else if(info.tof_valid && info.tof_distance>8.0) 
            {
                //printf("dist>8.0, %02X,%02X,%02X, %u\n", _rbuf[10], _rbuf[9], _rbuf[8], dist);
                info.tof_valid=false;
            }
            else if(info.tof_valid && info.tof_distance<0.0)
            {
                //printf("dist<0, %02X,%02X,%02X, %u\n", _rbuf[10], _rbuf[9], _rbuf[8], dist);
                info.tof_valid=false;
            }

            // //0距离信号的强度要求更严格。
            // if(info.tof_distance==0.0 && tofv && tof_sig_strengh < 30)
            // {
            //     info.tof_valid=false;
            // }

            //printf("got tof pack, v=%d, d=%f, s=%d\n", tofv, info.fixed_tof_distance, info.tof_strengh);

            if(_rbuf_tail > 16)
            {
                memmove(_rbuf, _rbuf+16, _rbuf_tail-16);
                _rbuf_tail-= 16;
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