#include "gps.h"
#include "tools.h"

//目前GPS默认9600，不知道如何提高速率
//重新在pc端配置了波特率115200

//这个初始化应该不会失败
//si=true, use system interrupt to read data.
bool CGPS::Initialize(int uart_port,int pin_rx, int pin_tx)
{

// EBYTE E108 GPS模块不支持消息频率保存，每次启动都要设置。
//$PGKC101,200*8B<CR><LF> 发送这个命令是请求200毫秒间隔一个数据。

    if(uart_port)
    {
        uart_init(uart1, GPS_BUADRATE);
        uart_set_format(uart1, 8,1, UART_PARITY_NONE);
        //uart_set_format(uart0, 8, 1, UART_PARITY_EVEN); //lora sx1280初始设置

        uart_set_fifo_enabled(uart1, true);//使用fifo
        uart_set_hw_flow(uart1, true, true);//使用硬件流控制
        gpio_set_function(pin_tx, GPIO_FUNC_UART);
        gpio_set_function(pin_rx, GPIO_FUNC_UART);

        _rbuf.clear();
        _uart_port=1;

        const uint8_t cmd[]= "$PGKC462*3F\r\n";
        uart_write_blocking(uart1, cmd, 13);
        //uart_puts(uart1, "$PGKC101,200*8B<CR><LF>"); //ebyte e108 专门配置，200毫秒一个数据。

        return true;
    }
    else 
    {
        uart_init(uart0, GPS_BUADRATE);
        uart_set_format(uart0, 8,1, UART_PARITY_NONE);
        //uart_set_format(uart0, 8, 1, UART_PARITY_EVEN); //lora sx1280初始设置

        uart_set_fifo_enabled(uart0, true);//使用fifo
        uart_set_hw_flow(uart0, true, true);//使用硬件流控制
        gpio_set_function(pin_tx, GPIO_FUNC_UART);
        gpio_set_function(pin_rx, GPIO_FUNC_UART);

        _rbuf.clear();
        _uart_port=0;

        const uint8_t cmd[]= "$PGKC462*3F\r\n";
        uart_write_blocking(uart0, cmd, 13);
        return true;
    }
    
}

//pio调用输入一个字符
bool CGPS::Pio_PutChar(char c, gps_info& info)
{

        // if(c>127) {
        //     printf("put %02X to gps, possible wrong\n", c);
        // }

        _rbuf.push_back(c);
   
        if(c=='\n') 
        {
            gps_info one; uint8_t infobits=0;

            int nr= InfoLineParse(_rbuf, one, infobits); //返回0就是已定位，数据有效，时间也一定有效。
            if(nr==0) {
                //有效读取了一行数据，将信息的有效数据填入返回值。
                if(one.gpstime!=_inclass_buf.gpstime) {
                    //不是同组数据，而且新时间一定超过老时间。
                    _inclass_buf=one;
                    _validbit=infobits;

                }else{

                    if(infobits & _gps_speed_valid) {
                        _inclass_buf.speed = one.speed;
                        _validbit |= _gps_speed_valid;
                    }
                    if(infobits & _gps_height_valid) {
                        _inclass_buf.height = one.height;
                        _validbit |= _gps_height_valid;
                    }
                    if(infobits & _gps_direction_valid) {
                        _inclass_buf.direction = one.direction;
                        _validbit |= _gps_direction_valid;
                    }
                    if(infobits & _gps_accurate_valid) {
                        _inclass_buf.hacc= one.hacc;
                        _validbit |= _gps_accurate_valid;
                    }

                }
            }
            _rbuf.clear();

            //数据完整就不继续读了，返回。
            if(_validbit == 0x3f) {
                info= _inclass_buf;
                _validbit=0;
                _inclass_buf.gpstime=0;
                return true;
            }
        }
        else if(_rbuf.size()>2048)
        {
            _rbuf.clear();
        }

        // //避免数据错误导致缓存无限增长，以头部标记$来重新开始
        // else if(c=='$'&& _rbuf.size()>2048)
        // {
        //     printf("buf oversized\n");
        //     _rbuf.clear();
        //     _rbuf.push_back(c);
        // }

        return false;
}

//如果读到了位置信息，时间信息等，以最后的数据为准返回。
//时间：HHMMSS[毫秒，3位数], uint
//定位可用/不可用，bool
//纬度：float 度数，经过换算
//经度：float 度度，经过换算
//海拔高度：float, 单位米
//相对速度：float, 单位是米/秒，经过换算
//航向角度：float, 0-360度
//水平定位精度：float, 单位米
bool CGPS::Read(gps_info& info)
{
    while(uart_is_readable(_uart_port?uart1:uart0))
    {
        char c=uart_getc(_uart_port?uart1:uart0);

        if(c>127) printf("possible bad char %02X\n", c);
        
        _rbuf.push_back(c);
   
        if(c=='\n') {
            gps_info one; uint8_t infobits=0;

            int nr= InfoLineParse(_rbuf, one, infobits); //返回0就是已定位，数据有效，时间也一定有效。
            if(nr==0) {
                //有效读取了一行数据，将信息的有效数据填入返回值。
                if(one.gpstime!=_inclass_buf.gpstime) {
                    //不是同组数据，而且新时间一定超过老时间。
                    _inclass_buf=one;
                    _validbit=infobits;

                }else{

                    if(infobits & _gps_speed_valid) {
                        _inclass_buf.speed = one.speed;
                        _validbit |= _gps_speed_valid;
                    }
                    if(infobits & _gps_height_valid) {
                        _inclass_buf.height = one.height;
                        _validbit |= _gps_height_valid;
                    }
                    if(infobits & _gps_direction_valid) {
                        _inclass_buf.direction = one.direction;
                        _validbit |= _gps_direction_valid;
                    }
                    if(infobits & _gps_accurate_valid) {
                        _inclass_buf.hacc= one.hacc;
                        _validbit |= _gps_accurate_valid;
                    }

                }
            }
            _rbuf.clear();

            //数据完整就不继续读了，返回。
            if(_validbit == 0x3F) {
                info= _inclass_buf;
                _validbit=0;
                _inclass_buf.gpstime=0;
                 return true;
            }
        }
        else if(_rbuf.size()>2048)
        {
            _rbuf.clear();
        }

        // //避免数据错误导致缓存无限增长，以头部标记$来重新开始
        // if(c=='$'&& _rbuf.size()>4096)
        // {
        //     printf("buf oversized\n");
        //     _rbuf.clear();
        //     _rbuf.push_back(c);
        // }
    }

    return false;
}

//缺失的信息不填入info
//返回0就是已经定位了，否则不返回0.
//有对应的信息设置相应的信息位。
int CGPS::InfoLineParse(std::string& line, gps_info& info, uint8_t& infobits)
{

    TrimString(line);
    std::vector<std::string> segs;
    ParseLine(line, segs);
    if(segs.size()<2) return -1;

//**RMC信息：解状态、经纬度、地面速度、地面航向角、UTC时间、
// $GPRMC,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,<11>,<12><CR><LF> 

// 1) 标准定位时间（UTC time）格式：时时分分秒秒.秒秒秒（hhmmss.sss）。 
// 2) 定位状态，A = 数据可用，V = 数据不可用。 
// 3) 纬度，格式：度度分分.分分分分（ddmm.mmmm）。 
// 4) 纬度区分，北半球（N）或南半球（S）。 
// 5) 经度，格式：度度分分.分分分分。 //度可以是3位数，小数点前两个是分
// 6) 经度区分，东（E）半球或西（W）半球。 
// 7) 相对位移速度， 0.0 至 1851.8 knots 
// 8) 相对位移方向，000.0 至 359.9度。实际值。 
// 9) 日期，格式：日日月月年年（ddmmyy）。 
// 10) 磁偏角，000.0 至180.0。总是正数，似乎也是没有数据，空字段。
// 11) E/W,代表磁偏角方向向东还是向西 ，似乎也是没有数据，空字段。
// 12) Checksum.(检查位) 

//有时我们的机器给出定位可用标记，但只有纬度可用或只有经度可用，数字后跟字母N,S,E,W等，
//如果缺少数据，可能会缺两个数据段，比如缺经度数据和后面的经度区分字E，这样段落数会少两个。
//这种部分可用的定位这里处理成定位不可用。

    if(segs[0].compare("$GPRMC")==0||segs[0].compare("$GNRMC")==0||
        segs[0].compare("$BDRMC")==0||segs[0].compare("$GLRMC")==0)
    {
        //printf("got **RMC\n");
        //printf(line.c_str());
        //printf("\n");

        if(segs.size()<11) {
            printf("bad **RMC segsize\n");
            return -2;
        }

        TrimString(segs[1]);
        size_t pos= segs[1].find_first_of('.');
        if(pos==std::string::npos) {
            //printf("wrong time format:%s\n", segs[1].c_str());
            return -3; //wrong format;
        }
        std::string p1= segs[1].substr(0,pos);
        std::string p2= segs[1].substr(pos+1);
        info.gpstime= atoi(p1.c_str())*1000 + atoi(p2.c_str());

        TrimString(segs[2]);

        if(segs[2].compare("A")==0)
        {
            TrimString(segs[3]);
            if(segs[3].length()<9) {
                printf("bad lati format\n");
                return -4; //wrong format
            }

            size_t pdot= segs[3].find_first_of('.');
            if(pdot==std::string::npos||pdot<4) {
              printf("bad lati format\n");
              return -5; //wrong format;
            }

            p1= segs[3].substr(0,pdot-2);
            p2= segs[3].substr(pdot-2);
            double d=atof(p1.c_str());
            double md=atof(p2.c_str());
            info.latitude= d+md/60.0f;

            TrimString(segs[4]);
            if(segs[4].compare("N")!=0) {
                printf("bad lati mark\n");
                return -6; //北纬标记
            }

            TrimString(segs[5]);
            if(segs[5].length()<9) {
                printf("bad longi format\n");
                return -7; //wrong format
            }

            pdot= segs[5].find_first_of('.');
            if(pdot==std::string::npos||pdot<4) {
              printf("bad longi format\n");
              return -8; //wrong format;
            }

            p1= segs[5].substr(0,pdot-2);
            p2= segs[5].substr(pdot-2);
            d=atof(p1.c_str());
            md=atof(p2.c_str());
            info.longitude= d+md/60.0f;  

            TrimString(segs[6]);
            if(segs[6].compare("E")!=0) {
                printf("bad lati mark\n");
                return -9; //东经标记
            }

            TrimString(segs[7]);
            d=atof(segs[7].c_str());
            info.speed=d*0.514; //换算米/秒

            TrimString(segs[8]);
            info.direction= atof(segs[8].c_str());
            
            //自行计算东向和北向速度。
            info.speed_east = info.speed*sin(info.direction/57.2958);
            info.speed_north= info.speed*cos(info.direction/57.2958);

            infobits |= _gps_time_valid;
            infobits |= _gps_position_valid;
            infobits |= _gps_speed_valid;
            infobits |= _gps_direction_valid;

            //info.accurate=-1;//此格式无精度
            //info.height=-1;//此格式无高度
            //printf("**RMC located\n");
            return 0;
        }
        else
        {
            //info.located=false;
            //printf("**RMC not located\n");
            return -10;
        }
    }


//**GGA信息：UTC时分秒、经纬度、GPS状态、卫星数量、高程、差分延迟、基站号，四种同格式
//GPGGA:单GPS,BDGGA:单北斗,GLGGA:单格罗纳斯,GNGGA:联合定位
// 格式：GPGGA,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,<11>,<12>,<13>,<14>,<15>,<16>,<17>,<18>
// 实例：$GPGGA,140145.000,3150.863861,N,11711.928739,E,1,11,0.79,175.165,M,0.009,M,53
// <0>信息ID
// <1>UTC时间，格式：“时时分分秒秒.秒秒秒”。
// <2>纬度，格式为"度度分分.分分分分”"
// <3>N：南半球，S：北半球
// <4>经度，格式为"度度度分分.分分分分”"
// <5>E：东半球，W：西半球
// <6>定位状态，
// “0”=定位不可用或无效
// “1”=GNSS 定位
// “2”=DGPS 定位
// <7>使用的卫星数（00~12），有可能不等于可见卫星数量
// <8>水平精度因子
// <9>大地高，单位：米
// <10>米（固定字段）
// <11>大地水准面差距，单位：米
// <12>米（固定字段）
// <13>差分卫星导航系统数据龄期，单位：秒。若不用 DGPS，可以省略
// <14>差分基准站标识号（0000-1023）。若不用 DGPS，可以省略
// <15>,数据字段结束符
// <16>校验和，格式：十六进制
// <17>每条 NMEA 语句以回车换行符结束
    else if(segs[0].compare("$GPGGA")==0||
        segs[0].compare("$BDGGA")==0||
        segs[0].compare("$GLGGA")==0||
        segs[0].compare("$GNGGA")==0)
    {

        //printf("got **GGA\n");
        //printf(line.c_str());
        //printf("\n");

        if(segs.size()<14) {
            printf("bad GGA segsize\n");
            printf(line.c_str());
            printf("segsize=%d\n", segs.size());
            return -11;
        }
        TrimString(segs[1]);
        size_t pos= segs[1].find_first_of('.');
        if(pos==std::string::npos) {
            //printf("wrong time format:%s\n", segs[1].c_str());
            return -12; //wrong format;
        }
        std::string p1= segs[1].substr(0,pos);
        std::string p2= segs[1].substr(pos+1);
        info.gpstime= atoi(p1.c_str())*1000 + atoi(p2.c_str());


        TrimString(segs[6]);
        if(segs[6].compare("0")==0) {

            //printf("**GGA not located\n");
            return -13;

        }else{
 
            TrimString(segs[2]);
            if(segs[2].length()<9) return -14; //wrong format

            size_t pdot= segs[2].find_first_of('.');
            if(pdot==std::string::npos||pdot<4) {
              printf("bad lati format\n");
              return -15; //wrong format;
            }

            p1= segs[2].substr(0,pdot-2);
            p2= segs[2].substr(pdot-2);
            double d=atof(p1.c_str());
            double md=atof(p2.c_str());
            info.latitude= d+md/60.0f;

            TrimString(segs[3]);
            if(segs[3].compare("N")!=0) return -16;

            TrimString(segs[4]);
            if(segs[4].length()<9) return -17; //wrong format

            pdot= segs[4].find_first_of('.');
            if(pdot==std::string::npos||pdot<4) {
              printf("bad longi format\n");
              return -18; //wrong format;
            }

            p1= segs[4].substr(0,pdot-2);
            p2= segs[4].substr(pdot-2);

            d=atof(p1.c_str());
            md=atof(p2.c_str());
            info.longitude= d+md/60.0f;  

            TrimString(segs[5]);
            if(segs[5].compare("E")!=0) return -19;

            TrimString(segs[7]); //卫星数量
            int n=atoi(segs[7].c_str());
            info.ns=n;

            TrimString(segs[8]);
            d=atof(segs[8].c_str());
            info.hacc=d; //水平精度

            TrimString(segs[9]);
            info.height= atof(segs[9].c_str());

            infobits |= _gps_time_valid;
            infobits |= _gps_position_valid;
            infobits |= _gps_accurate_valid;
            infobits |= _gps_height_valid;

            //info.speed=-1; //此格式没有速度
            //info.direction=-1; //此格式没有方向
            //printf("**GGA located\n");
            return 0;

        }

        //printf("%s,%s,%s,%s,%s\n", segs[2].c_str(), segs[3].c_str(), segs[5].c_str(), segs[10].c_str(), segs[12].c_str());
    }
    //**GSA信息有用的只有定位精度，海拔精度，其他地方没有。
    else if(segs[0].compare("$GPGSA")==0||
            segs[0].compare("$BDGSA")==0||
            segs[0].compare("$GLGSA")==0||
            segs[0].compare("$GNGSA")==0 )
    {
        //$GPGSA,A,3,05,12,17,19,20,199,,,,,,,3.2,2.2,2.3*09
        //解析时要注意最后一段含有校验码
        //暂时忽略这个，以后增加。主要提取海拔精度。
        // 字段0：$GPGSA，语句ID，表明该语句为GPS DOP and Active Satellites（GSA）当前卫星信息
        // 字段1：定位模式，A=自动手动2D/3D，M=手动2D/3D
        // 字段2：定位类型，1=未定位，2=2D定位，3=3D定位
        // 字段3：PRN码（伪随机噪声码），第1信道正在使用的卫星PRN码编号（00）（前导位数不足则补0）
        // 字段4：PRN码（伪随机噪声码），第2信道正在使用的卫星PRN码编号（00）（前导位数不足则补0）
        // 字段5：PRN码（伪随机噪声码），第3信道正在使用的卫星PRN码编号（00）（前导位数不足则补0）
        // 字段6：PRN码（伪随机噪声码），第4信道正在使用的卫星PRN码编号（00）（前导位数不足则补0）
        // 字段7：PRN码（伪随机噪声码），第5信道正在使用的卫星PRN码编号（00）（前导位数不足则补0）
        // 字段8：PRN码（伪随机噪声码），第6信道正在使用的卫星PRN码编号（00）（前导位数不足则补0）
        // 字段9：PRN码（伪随机噪声码），第7信道正在使用的卫星PRN码编号（00）（前导位数不足则补0）
        // 字段10：PRN码（伪随机噪声码），第8信道正在使用的卫星PRN码编号（00）（前导位数不足则补0）
        // 字段11：PRN码（伪随机噪声码），第9信道正在使用的卫星PRN码编号（00）（前导位数不足则补0）
        // 字段12：PRN码（伪随机噪声码），第10信道正在使用的卫星PRN码编号（00）（前导位数不足则补0）
        // 字段13：PRN码（伪随机噪声码），第11信道正在使用的卫星PRN码编号（00）（前导位数不足则补0）
        // 字段14：PRN码（伪随机噪声码），第12信道正在使用的卫星PRN码编号（00）（前导位数不足则补0）
        // 字段15：PDOP综合位置精度因子（0.5 - 99.9）
        // 字段16：HDOP水平精度因子（0.5 - 99.9）
        // 字段17：VDOP垂直精度因子（0.5 - 99.9）
        // 字段18：校验值

    }
    //**GSV主要是关于卫星的信息，和定位关系不大，忽略。
    else if(segs[0].compare(0,5,"$PGKC")==0 ||segs[0].compare("$PGKC001")==0)
    {
        //国科微GPS命令字头。
        printf("got $PGKC\n");
        printf(line.c_str());
        printf("\n");
    }


    return -20; //其他不关心的命令
}