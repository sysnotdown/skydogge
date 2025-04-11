#include "CDual_Lora.h"

#include "remote.h"
#include <random>
std::default_random_engine rgen;
std::uniform_int_distribution<int> enc_code_gen(0, 255);

//目前选用的是硅传sx1278-tc006的uart芯片。有几个管脚：
//对应系统的uart0端口，不能动。默认速率9600.

extern void led_blink(int times, int gap_ms);
extern uint32_t get_time_mark();
extern void Send_remote_message(std::string msg);
//定义使用E32 lora芯片来通讯，采用兼容插槽，共用E34代码。

//包类型定义，由飞控发往通信板的包类型，确定的类型对应了内容的长度。
enum fc_comm_exchange_packet_type
{
    packet_power=1,
    packet_gps=2,
    packet_baro=3,
    packet_predifined_message=4,
    packet_message=5, //这个不定长
    packet_imu=6,
    packet_motor=7, 
    packet_status=8, //inair等
};


//20240318 引入，准备替换老的校验方法。
uint8_t crc8(const uint8_t *data, size_t size) {
	uint8_t crc = 0x00;
	for (size_t i = 0; i < size; i++) {
		crc ^= data[i];
		for (uint8_t bit = 8; bit > 0; --bit) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0x07; // 多项式x^8 + x^2 + x^1 + 1 (0x107)
			}
			else {
				crc = crc << 1;
			}
		}
	}
	return crc;
}

//一次完整的飞控数据推送耗时7.4毫秒，包括GPS等，所以感觉115200的速度偏低了。
void CDual_Lora::Initialize(int8_t uport, int pin_rx, int pin_tx)
{

    _uport= uport;
    if(_uport==0)
    {
        uart_init(uart0, DUAL_LORA_BAUDRATE);
        uart_set_format(uart0, 8,1, UART_PARITY_NONE);//lora sx1280初始设置

        uart_set_fifo_enabled(uart0, true);//使用fifo
        uart_set_hw_flow(uart0, true, true);//使用硬件流控制

    }
    else
    {
        uart_init(uart1, DUAL_LORA_BAUDRATE);
        uart_set_format(uart1, 8,1, UART_PARITY_NONE);//lora sx1280初始设置

        uart_set_fifo_enabled(uart1, true);//使用fifo
        uart_set_hw_flow(uart1, true, true);//使用硬件流控制
    }

    //_rdbuf.reserve(128); //保留128字节，避免分配释放内存太频繁。

    gpio_set_function(pin_tx, GPIO_FUNC_UART);
    gpio_set_function(pin_rx, GPIO_FUNC_UART);

    critical_section_init(&_send_critical_section);
}

//返回值最低位是摇杆标记，次低位是预定义命令标记
uint8_t CDual_Lora::OnReceive(remote_info& info, uint8_t& predefined_task, std::string& taskcont)
{

    while(uart_is_readable(_uport?uart1:uart0))
    {
        char c=uart_getc(_uport?uart1:uart0);
        //_rdbuf.push_back(c);
        _rbuf[_rbuf_tail++]=c;
    }


    //命令解析。
    bool got_stick=false;
    bool got_predefined_task=false;
    bool got_task=false;

    //while(_rdbuf.size()>=4) {
    while(_rbuf_tail>=4) {

        uint8_t hd0= _rbuf[0];
        uint8_t hd1= _rbuf[1];
        //现在修改了代码，通信板传递上来的总是完整的包，包头是0xa1,0x62两个字节。
        if(hd0 == 0xA1 && hd1 == 0x62)
        {

            if(_rbuf[2]==0b11110000)
            {
                //这是摇杆包。现在没有开关信息
                //if(_rdbuf.size()<8) break;
                if(_rbuf_tail < 8) break;

                uint8_t crc= _rbuf[7];
                //uint8_t chk=crc8((uint8_t*)_rdbuf.data(), 7);//20240318 修改
                uint8_t chk=crc8((uint8_t*)_rbuf, 7);//20240318 修改
                //20231117+ 新增校验码，通信板上传到飞控的数据也有可能被干扰，会影响到姿态。
                if(chk!=crc) {
                    //bad check.
                    //_rdbuf.erase(0,8);
                    //删除头部8个字节。
                    if(_rbuf_tail<=8) _rbuf_tail=0;
                    else {
                        memmove(_rbuf, _rbuf+8, _rbuf_tail-8);
                        _rbuf_tail-=8;
                    }
                    printf("crc fail\n");
                }
                else{
                    got_stick=true;
                    info.pos1 = _rbuf[3];
                    info.pos2 = _rbuf[4];
                    info.pos3 = _rbuf[5];
                    info.pos4 = _rbuf[6];
                    info.tmark=get_time_mark();
                    //_rdbuf.erase(0,8);

                    if(_rbuf_tail<=8) _rbuf_tail=0;
                    else {
                        memmove(_rbuf, _rbuf+8, _rbuf_tail-8);
                        _rbuf_tail-=8;
                    }
                    //printf("got %d %d %d %d\n", info.pos1, info.pos2, info.pos3, info.pos4);
                }
            }
            else if(_rbuf[2]==0b10010000)
            {
                //这是预定义任务包
                //if(_rdbuf.size()<5) break;
                if(_rbuf_tail < 5) break;

                uint8_t crc= _rbuf[4];
                //uint8_t chk=crc8((uint8_t*)_rdbuf.data(),4); //20240318 修改
                uint8_t chk=crc8((uint8_t*)_rbuf,4); //20240318 修改
                if(chk==crc) {
                    got_predefined_task=true;
                    predefined_task = _rbuf[3];
                    printf("got pt=%d\n", predefined_task);
                    if(predefined_task==0) {
                        got_predefined_task=false;
                        printf("pt=0 invalid task.\n");
                    }
                }else{
                    printf("crc check fail for predefined task\n");
                }

                //_rdbuf.erase(0,5);
                if(_rbuf_tail<=5) _rbuf_tail=0;
                else {
                    memmove(_rbuf, _rbuf+5, _rbuf_tail-5);
                    _rbuf_tail-=5;
                }
            }
            else if(_rbuf[2]==0b01100000) //task.
            {
                //任务包，2字节包头，1字节包类型，1字节包长度，至少1字节包内容，1字节校验码。最少6字节。
                //容忍0字节内容则至少是5字节。
                //if(_rdbuf.size()<5) break; 
                if(_rbuf_tail<5) break;

                uint8_t contlen=_rbuf[3];
                //if(_rdbuf.size()<5+size_t(contlen)) break;
                if(_rbuf_tail < 5+size_t(contlen)) break;

                if(contlen>0 && contlen<200) //0字节包没意义。
                {
                    //uint8_t crc=crc8((uint8_t*)_rdbuf.data(), 4+contlen); //20240318 修改
                    uint8_t crc=crc8((uint8_t*)_rbuf, 4+contlen); //20240318 修改
                    if(crc==_rbuf[4+contlen])
                    {
                        got_task=true;
                        
                        //taskcont=_rdbuf.substr(4, contlen);
                        //这里还是返回std::string,暂时不改。
                        taskcont.clear();
                        taskcont.append(_rbuf+4, contlen);
                    }
                }

                //_rdbuf.erase(0, 5+size_t(contlen));

                if(_rbuf_tail < 5+size_t(contlen)) _rbuf_tail=0;
                else {
                    memmove(_rbuf, _rbuf+5+size_t(contlen), _rbuf_tail-5-size_t(contlen));
                    _rbuf_tail-=5+size_t(contlen);
                }
            }
            else
            {
                //不能识别的类型。
                //_rdbuf.erase(0,1);
                if(_rbuf_tail<=1) _rbuf_tail=0;
                else {
                    memmove(_rbuf, _rbuf+1, _rbuf_tail-1);
                    _rbuf_tail-=1;
                }
                continue;
            }
            
        }
        else
        {
            //其他不可识别的命令头部。也许是没对齐。也许同频干扰。删除一个字节。
            //_rdbuf.erase(0,1);
            if(_rbuf_tail<=1) _rbuf_tail=0;
            else {
                memmove(_rbuf, _rbuf+1, _rbuf_tail-1);
                _rbuf_tail-=1;
            }
            continue;
        }

    }

    uint8_t nret=0;
    if(got_stick) {
        nret|=1;
    }
    if(got_predefined_task) {
        nret|=2;
    }
    if(got_task) {
        nret|=4;
    }
    return nret;
}


void CDual_Lora::SendPredefinedMessage(uint8_t id) //发送预定类型的消息。
{
    uint8_t buf[32];
    buf[0]=0xA1; //边界码2个字节，区分包边界。
    buf[1]=0x62;
    buf[2]=0; //校验码防止通信干扰
    buf[3]=fc_comm_exchange_packet_type::packet_predifined_message; //包类型。
    buf[4]=id;

    //计算校验码。连同包类型有13字节。
    // for(size_t i=3; i<5;i++)
    // {
    //     buf[2]^=buf[i];
    // }

    buf[2]= crc8(buf+3, 2); //20240802+

    //全包5字节。
    uart_write_blocking(_uport?uart1:uart0, buf, 5);
}


//扩展了一个数据
void CDual_Lora::SendPowerInfo(float voltage, float current, float consume) //发送电压信息。
{
    uint8_t buf[32];
    buf[0]=0xA1; //边界码2个字节，区分包边界。
    buf[1]=0x62;
    buf[2]=0; //校验码防止通信收到干扰
    buf[3]=fc_comm_exchange_packet_type::packet_power; //包类型。
    memcpy(buf+4, &voltage, sizeof(float));
    memcpy(buf+8, &current, sizeof(float));
    memcpy(buf+12, &consume, sizeof(float));

    //计算校验码。连同包类型有13字节。
    // for(size_t i=3; i<16;i++)
    // {
    //     buf[2]^=buf[i];
    // }

    buf[2]= crc8(buf+3, 13); //20240802+

    //全包16字节。
    uart_write_blocking(_uport?uart1:uart0, buf, 16);
}


void CDual_Lora::SendMessage(std::string msg) //发送消息。
{
    if(msg.size()>160) return; //超过长度直接不发。
    if(msg.size()==0) return;

    uint8_t len=uint8_t(msg.size());

    uint8_t buf[80];
    buf[0]=0xA1; //边界码2个字节，区分包边界。
    buf[1]=0x62;
    buf[2]=0; //校验码防止通信收到干扰
    buf[3]=fc_comm_exchange_packet_type::packet_message; //包类型。
    buf[4]=len; //包长度 uint8_t
    memcpy(buf+5, msg.data(), len);

    // for(uint8_t i=3;i<5+len;i++)
    // {
    //     buf[2]^=buf[i];
    // }

    buf[2]=crc8(buf+3, 2+size_t(len));//20240802+

    uart_write_blocking(_uport?uart1:uart0, buf, 5+size_t(len));
}


//新法只发裸数据，不考虑加密。
void CDual_Lora::SendGpsInfo(gps_info& gps)
{
// sizeof(double)=8, sizeof(float)=4
// sizeof(int8_t)=1, sizeof(int16_t)=2
// sizeof(int32_t)=4, sizeof(int64_t)=8

    uint8_t buf[64];
    buf[0]=0xA1; //边界码2个字节，区分包边界。
    buf[1]=0x62;
    buf[2]=0; //校验码防止通信收到干扰
    buf[3]=fc_comm_exchange_packet_type::packet_gps; //包类型。
    memcpy(buf+4, &gps.latitude, sizeof(double)); //8
    memcpy(buf+12, &gps.longitude, sizeof(double)); //8
    memcpy(buf+20, &gps.height, sizeof(float)); //4
    memcpy(buf+24, &gps.speed, sizeof(float)); //4
    memcpy(buf+28, &gps.direction, sizeof(float)); //4
    memcpy(buf+32, &gps.hacc, sizeof(float)); //4;
    memcpy(buf+36, &gps.speed_down, sizeof(float)); //4
    memcpy(buf+40, &gps.ns, sizeof(uint8_t)); //1

    //计算校验码。连同包类型有38字节。
    // for(size_t i=3; i<41;i++)
    // {
    //     buf[2]^=buf[i];
    // }

    buf[2]= crc8(buf+3, 38); //20240802+
    //全包41字节。
    uart_write_blocking(_uport?uart1:uart0, buf, 41);
}



//新方法只发送裸数据，通信板负责最后的发送。
void CDual_Lora::SendBaroInfo(float press, float temp)
{
    uint8_t buf[32];
    buf[0]=0xA1; //边界码2个字节，区分包边界。
    buf[1]=0x62;
    buf[2]=0; //校验码防止通信收到干扰
    buf[3]=fc_comm_exchange_packet_type::packet_baro; //包类型。
    memcpy(buf+4, &press, sizeof(float));
    memcpy(buf+8, &temp, sizeof(float));

    //计算校验码。连同包类型有9字节。
    // for(size_t i=3; i<12;i++)
    // {
    //     buf[2]^=buf[i];
    // }
    buf[2]=crc8(buf+3, 9);

    //全包12字节。
    uart_write_blocking(_uport?uart1:uart0, buf, 12);
}


void CDual_Lora::SendMotorInfo(float m0, float m1, float m2, float m3)
{
    uint8_t buf[32];
    buf[0]=0xA1; //边界码2个字节，区分包边界。
    buf[1]=0x62;
    buf[2]=0; //校验码防止通信收到干扰
    buf[3]=fc_comm_exchange_packet_type::packet_motor; //包类型。
    memcpy(buf+4, &m0, sizeof(float));
    memcpy(buf+8, &m1, sizeof(float));
    memcpy(buf+12, &m2, sizeof(float));
    memcpy(buf+16, &m3, sizeof(float));

    //计算校验码。连同包类型有17字节。
    // for(size_t i=3; i<20;i++)
    // {
    //     buf[2]^=buf[i];
    // }

    buf[2]=crc8(buf+3, 17);

    //全包20字节。
    uart_write_blocking(_uport?uart1:uart0, buf, 20);
}


//扩展了两个数据。
void CDual_Lora::SendImuInfo(float pitch, float roll, float yaw, float heading)
{
    uint8_t buf[32];
    buf[0]=0xA1; //边界码2个字节，区分包边界。
    buf[1]=0x62;
    buf[2]=0; //校验码防止通信收到干扰
    buf[3]=fc_comm_exchange_packet_type::packet_imu; //包类型。
    memcpy(buf+4, &pitch, sizeof(float));
    memcpy(buf+8, &roll, sizeof(float));
    memcpy(buf+12, &yaw, sizeof(float));
    memcpy(buf+16, &heading, sizeof(float));

    //计算校验码。连同包类型有17字节。
    // for(size_t i=3; i<20;i++)
    // {
    //     buf[2]^=buf[i];
    // }
    buf[2]=crc8(buf+3, 17); //20240802+

    //全包20字节。
    uart_write_blocking(_uport?uart1:uart0, buf, 20);
}



//取消了本地信号感知，因为通信板可以处理。
//新增on_remote标记。
//last_sig显示间隔是0.1秒，20240121改
void CDual_Lora::SendStatus(bool inair, bool gps_bad, bool on_remote, uint16_t last_sig)
{
    uint8_t buf[32];
    buf[0]=0xA1; //边界码2个字节，区分包边界。
    buf[1]=0x62;
    buf[2]=0; //校验码防止通信收到干扰
    buf[3]=fc_comm_exchange_packet_type::packet_status; //包类型。
    
    if(inair) buf[4]=1;
    else buf[4]=0;

    if(gps_bad) buf[5]=1;
    else buf[5]=0;

    if(on_remote) buf[6]=1;
    else buf[6]=0;

    memcpy(buf+7, &last_sig, sizeof(uint16_t)); //2字节。

    // for(size_t i=3;i<9;i++)
    // {
    //     buf[2]^= buf[i];
    // }

    buf[2]=crc8(buf+3, 6); //20240802+

    //全包8字节。
    uart_write_blocking(_uport?uart1:uart0, buf, 9);
}