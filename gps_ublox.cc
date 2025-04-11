#include "gps_ublox.h"
#include "memory.h"

extern uint32_t get_time_mark();
extern void Send_remote_message(std::string msg);
// extern uint32_t _total_gps_distance; //总飞行距离
// extern uint32_t _gps_distance_since_last_reset; //本次飞行距离。
//如需如下数据，需要对GPS模块进行设置。CFG-BATCH-EXTRAPVT
// Include extra PVT data -- 
// Include additional PVT information in UBX-LOG-BATCH messages. 
// If not selected only basic information is included. 
// The fields iTOW, tAcc, numSV, hMSL, vAcc, velN, velE, 
// velD, sAcc, headAcc and pDOP in 'UBX-LOG-BATCH' are 
// only valid if this flag is set.
// 默认没有设置，但似乎有numSV, vacc数据。

bool CGPS_Ublox::Initialize(int uart_port,int pin_rx, int pin_tx)
{

// EBYTE E108 GPS模块不支持消息频率保存，每次启动都要设置。
//$PGKC101,200*8B<CR><LF> 发送这个命令是请求200毫秒间隔一个数据。

    if(uart_port)
    {
        uart_init(uart1, GPS_UBLOX_BUADRATE);
        uart_set_format(uart1, 8,1, UART_PARITY_NONE);
        //uart_set_format(uart0, 8, 1, UART_PARITY_EVEN); //lora sx1280初始设置

        uart_set_fifo_enabled(uart1, true);//使用fifo
        uart_set_hw_flow(uart1, true, true);//使用硬件流控制
        gpio_set_function(pin_tx, GPIO_FUNC_UART);
        gpio_set_function(pin_rx, GPIO_FUNC_UART);

        _rbuf_tail=0;
        _uart_port=1;

        return true;
    }
    else 
    {
        uart_init(uart0, GPS_UBLOX_BUADRATE);
        uart_set_format(uart0, 8,1, UART_PARITY_NONE);
        //uart_set_format(uart0, 8, 1, UART_PARITY_EVEN); //lora sx1280初始设置

        uart_set_fifo_enabled(uart0, true);//使用fifo
        uart_set_hw_flow(uart0, true, true);//使用硬件流控制
        gpio_set_function(pin_tx, GPIO_FUNC_UART);
        gpio_set_function(pin_rx, GPIO_FUNC_UART);

        _rbuf_tail=0;
        _uart_port=0;

        return true;
    }
    
}

bool CGPS_Ublox::Read(gps_info& info)
{

    while(uart_is_readable(_uart_port?uart1:uart0))
    {
        char c=uart_getc(_uart_port?uart1:uart0);
        _rbuf[_rbuf_tail++]=c;
        if(_rbuf_tail>=255) break;
    }

    return ParseBuffer(info);
}

bool CGPS_Ublox::Pio_PutChar(char c, gps_info& info)
{
    _rbuf[_rbuf_tail++]=c;
    return ParseBuffer(info);
}


bool CGPS_Ublox::ParseBuffer(gps_info& info)
{

    if(_rbuf_tail <2) {
        return false;
    }

    //check first 2bytes. make sure content is start with good head.
    if(_rbuf[0]!= char(0xB5)||_rbuf[1]!= char(0x62))
    {
        //bad align. remove some data.
        size_t pos= std::string::npos;


        for(size_t i=1;i<_rbuf_tail-1;i++) //_rbuf_tail>=2
        {
            if(_rbuf[i]== char(0xB5) && _rbuf[i+1]== char(0x62))
            {
                pos= i;
                break;
            }
        }


        if(pos==std::string::npos) {
            if(_rbuf[_rbuf_tail-1]!= char(0xB5))  {
                 _rbuf_tail=0; //clear all buf data
            }
            else {
                //left last buf byte 0xB5
                _rbuf[0]=0xB5;
                _rbuf_tail=1;
            }
            return false;
        }else{
            _rbuf_tail=0;
        }
    }

    //至此头部2字节确认对齐并存在。

    //完整头部至少6个字节。
    if(_rbuf_tail < 6) {
        return false;
    }

    uint8_t msg_class= _rbuf[2];
    uint8_t msg_id = _rbuf[3];

    uint16_t msg_len0= _rbuf[4];
    uint16_t msg_len1= _rbuf[5];
    uint16_t msg_len = (msg_len1<<8) + msg_len0; //payload 长度。

    //payload 长度应该加以限制，如果这个数据出现错误，比如说接收到50000的长度，那么就需要等50000个字节后的校验码
    //再去判断整个包是否有效，这样会把内存占用，也会丢失很多有效定位包。一个不合理的payload长度包应该丢弃。
    //但是payload长度多长是合理的包？ 因为上面有msg_class, msg_id，可以作为辅助判断。

    uint16_t pack_len = 8 + msg_len; //总包长度。

    if(msg_class==1 && msg_id==7)
    {
        //nav class, pvt msg, 定长92字节。
        if(msg_len!=92) {

            if(_rbuf_tail>pack_len)
            {
                memmove(_rbuf, _rbuf+pack_len, _rbuf_tail-pack_len);
                _rbuf_tail-=pack_len;
            } 
            else
            {
                _rbuf_tail=0;
            }
            return false;
        }
    }
    else
    {
        //其他类型的包。大于256长度即丢弃。
        //包很长，而且缓冲区没接收全，不等待这个包。如果是接收全了，就解析。
        if(msg_len >= 256 && _rbuf_tail < pack_len)
        {
            _rbuf_tail=0;
            return false;
        }
    }

    //等待接收全包
    if(_rbuf_tail < pack_len) {
        return false;
    }

    //达到全包长度了，解析包。
    
    //先进行校验码检查。
    uint8_t ch_a=0; uint8_t ch_b=0;
    for(uint16_t i=2; i< 6 + msg_len; i++) {
        ch_a+= _rbuf[i];
        ch_b+= ch_a;
    }

    if(char(ch_a)!=_rbuf[pack_len-2])
    {
        //删除全包。
        if(_rbuf_tail>pack_len)
        {
            memmove(_rbuf, _rbuf+pack_len, _rbuf_tail-pack_len);
            _rbuf_tail-=pack_len;
        } 
        else
        {
            _rbuf_tail=0;
        }
        return false;
    }
    if(char(ch_b)!=_rbuf[pack_len-1])
    {
        //删除全包。
        if(_rbuf_tail>pack_len)
        {
            memmove(_rbuf, _rbuf+pack_len, _rbuf_tail-pack_len);
            _rbuf_tail-=pack_len;
        } 
        else
        {
            _rbuf_tail=0;
        }
        return false;
    }

    //校验通过提取payload
    char payload[256];
    memcpy(payload, _rbuf+6, msg_len);

    //缓存区删除全包。
    if(_rbuf_tail>pack_len)
    {
        memmove(_rbuf, _rbuf+pack_len, _rbuf_tail-pack_len);
        _rbuf_tail-=pack_len;
    } 
    else
    {
        _rbuf_tail=0;
    }

    //消息类型，
    switch(msg_class)
    {
        case 0x01: //Navigation Results: Position, Speed, Time, Acc, Heading, DOP, SVs used
            return NavPacket(msg_id, payload, msg_len, info);
            break;
        // case 0x02: //Receiver Manager Messages: Satellite Status, RTC Status
        //     break;
        // case 0x04: //Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
        //     break;
        // case 0x05: //Ack/Nack Messages: as replies to CFG Input Messages
        //     break;
        // case 0x06: //Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc.
        //     break;
        // case 0x0A: //Monitoring Messages: Comunication Status, CPU Load, Stack Usage, Task Status
        //     break;
        default: //其他的不管了。
            break;
    }

    return false;
}

bool CGPS_Ublox::Nav_pvt_Packet(uint8_t msgid, const char* payload, size_t payload_len, gps_info& info)
{
    if(payload_len!=92) {
        return false; 
    }

    uint32_t utow;
    memcpy(&utow, payload, 4);
    uint16_t uyear;
    memcpy(&uyear, payload+4, 2);
    uint8_t umonth= payload[6];
    uint8_t uday=payload[7];
    uint8_t uhour=payload[8];
    uint8_t uminite=payload[9];
    uint8_t usecond=payload[10];
    uint8_t xvalid=payload[11]; //有效性指示。0位日期有效，1位时间有效，2位时间日期完全有效，3位磁偏角有效
    uint32_t utacc;
    memcpy(&utacc, payload+12, 4);
    int32_t inano;
    memcpy(&inano, payload+16, 4);
    uint8_t ufixtype=payload[20]; //定位类型，0，无定位，1.推测定位，2，2d定位，3，3d定位，4，综合推测定位，5仅时间有效
    //if(!ufixtype) {
    if(ufixtype!=3) { //20240618改，严格限制3d定位。
        return false;
    }

    uint8_t xflags=payload[21];
    uint8_t xflags2=payload[22];
    uint8_t unumsv=payload[23];

    int32_t ilon; //精度
    memcpy(&ilon, payload+24, 4);
    int32_t ilati; //纬度
    memcpy(&ilati, payload+28, 4);


    int32_t iheight; //椭球面高度。
    memcpy(&iheight, payload+32, 4);

    int32_t ihmsl; //水平面高度，即海拔高度
    memcpy(&ihmsl, payload+36, 4);
    uint32_t uhacc; //水平精度
    memcpy(&uhacc, payload+40, 4);
    uint32_t uvacc; //垂直精度
    memcpy(&uvacc, payload+44, 4);
    int32_t iveln; //北向速度
    memcpy(&iveln, payload+48, 4);
    int32_t ivele; //东向速度
    memcpy(&ivele, payload+52, 4);
    int32_t iveld; //下降速度
    memcpy(&iveld, payload+56, 4);
    int32_t igspd; //? 对地速度，感觉应该是个uint
    memcpy(&igspd, payload+60, 4);
    int32_t iheadmot;  //Heading of motion 运动方向
    memcpy(&iheadmot, payload+64, 4);
    uint32_t usacc;  //速度精度
    memcpy(&usacc, payload+68, 4);
    uint32_t uheadacc; //Heading accuracy estimate 运动方向的精度
    memcpy(&uheadacc, payload+72, 4);
    //uint16_t updop;
    //uint8_t reverse;

    uint16_t xflags3; //m10文档中新增的字段，m8里面没有。
    memcpy(&xflags3, payload+78, 2);
    if(xflags3 & 0x01) {
        //bit0=1时，定位无效
        return false;
    }

    //经过2次有效性过滤，不该再出现无效定位数据了，再有就报警。
    if(ilon==0||ilati==0) {
        //Send_remote_message("bad logi/lati data");
        return false;
    }

    //在这附近，m8,m10的定义不同，之前一直按照m8的文档来处理，但使用m10的芯片

    int32_t  iheadveh; //机头指向角度。数据无效。
    memcpy(&iheadveh, payload+84, 4);
    int16_t  imagdec; //磁偏角，2字节整数，带符号，比例100.
    memcpy(&imagdec, payload+88, 2);
    int16_t  imagacc; //磁偏角精度
    memcpy(&imagacc, payload+90, 2);


//垂直方向速度没有取。
    info.longitude = double(ilon) * 1E-7;
    info.latitude = double(ilati) * 1E-7;
    info.ns= unumsv; //卫星数量,有数据
    info.height = float(ihmsl)/1000.0;
    info.hacc = float(uhacc)/1000.0;
    info.vacc = float(uvacc)/1000.0; //可接近1米，精度可以。
    info.speed = float(igspd)/1000.0;
    info.speed_east = float(ivele)/1000.0;
    info.speed_north = float(iveln)/1000.0;
    info.speed_down = float(iveld)/1000.0; //下降速度, 20230922新增
    info.direction= float(iheadmot) * 1E-5;
    //20231101修改这里，时间现在是整数类似 135624，代表13点，56分，24秒，抛弃纳秒精度数据。
    uint8_t local_hour= (uhour+8)%24; //转中国时区。
    info.gpstime = uint(local_hour)*10000 + uint(uminite)*100 + uint(usecond);
    info.tmark = get_time_mark();

    return true;
}

//数据返回给全局记录, 但掉电即掉记录，只能显示本次飞行里程，没有太大意思。
// void CGPS_Ublox::Nav_odo_Packet(uint8_t msgid, const char* payload, size_t payload_len)
// {
//     if(payload_len!=20) return;

//     memcpy(&_total_gps_distance, payload+8, 4);
//     memcpy(&_gps_distance_since_last_reset, payload+12, 4);
// }

bool CGPS_Ublox::NavPacket(uint8_t msgid, const char* payload, size_t payload_len, gps_info& info)
{
    if(msgid==7) return Nav_pvt_Packet(msgid, payload, payload_len, info);
    //else if(msgid==9) Nav_odo_Packet(msgid, payload, payload_len); //UBX-NAV-ODO 执行后返回false
    return false;
}
