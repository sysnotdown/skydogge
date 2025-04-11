
#include "icm426xx.h"
#include <stdio.h>
#include <iostream>
#include "pico/stdlib.h"

//42688和42605在AAF配置上不同，如采用同设置可能造成42688加速度噪音过大。
//42688 AAF的带宽在40-3979，42605 带宽在10-995
// The slave address of the ICM-42605 is b110100X, which is 7 bits long. The LSB bit of the 7-bit address is determined by the logic
// level on pin AP_AD0. This allows two ICM-42605s to be connected to the same I2C bus. When used in this configuration, the
// address of one of the devices should be b1101000 (pin AP_AD0 is logic low) and the address of the other should be b1101001 (pin
// AP_AD0 is logic high)

//陀螺仪滤波有三级：NF,AAF,UIF，输出率500hz

//WHO_AM_I 地址 117（0x75）,内容0x42，0x47是42688
//REG_BANK_SEL地址 118 (76h) 用来选择bank

#define GYRO_FS_SEL (2)  //从0-7分别代表范围2000,1000,500,250,125,62.5,31.25,15.625 
#define ACCEL_FS_SEL (2)   //0-3分别代表 16g, 8g, 4g, 2g

//目前aaf参数是18-324-7，带宽是200左右
//表格里最大的AAF带宽是995，对应参数63-3968-3
//804带宽对应参数 54-2944-3
//594带宽对应参数 43-1856-4
//507带宽对应参数 38-1440-4
//364带宽对应参数 29-848-5
//当陀螺仪使用了NOTCH FILTER时，最大带宽只有362，后期再使用AAF时，选择更高的带宽似乎没有意义了。364带宽是最优选择。
//似乎aaf带宽大点，yaw角漂移小点。
#define GYRO_AAF_DIS (0)  //Disables the Gyroscope Anti Alias Filter (AAF) 1=disable, 0=enable
//在42605的配置上，28-784-5的带宽是349，用于陀螺仪，9-81-9带宽99，用于加速度。
//在42688上，28-784-5的带宽是1395，9-81-9的带宽是394
//为了使42688的配置接近原始的42605，需要陀螺仪配置参数为8-64-9（带宽348），加速度配置2-4-13（带宽84）
//42605的加速度配置可以改为8-64-9（带宽87）这样噪音过滤比99带宽更好。

//20240228, 之前一直使用349hz带宽，准备降低到200左右，减小噪音，太大的带宽对更高的控制应该无用。
//42688似乎陀螺仪的噪音较大，温度漂移比较多。yaw角温度漂移大。需要考虑更大的AAF滤波。
//目前42605与滤波带宽之间的关系尚不清楚，在加大42688的滤波带宽之前，先放松42605的滤波带宽，在测试机上应用。
//balance里已经对角速度做了3均值处理，以适应更大噪音的陀螺仪数据。之前这里使用22-488-6 263hz
//先修改为44-1952-4  612hz 测试无明显问题，修改为63-3968-3 995hz, 测试无明显问题, yaw温飘很小
//9-81-9 99hz 10-100-8 110hz 低带宽噪音小，yaw角积分不易漂移。
//14-196-7 对应158hz 17-288-7 196hz
#define GYRO_AAF_DELT_42605 (17) //28-784-5 349hz  18-324-7  209hz  22-488-6 263hz  21-440-6 249hz
#define GYRO_AAF_DELTSQR_42605 (288)
#define GYRO_AAF_BITSHIFT_42605  (7)

//3-9-12 对应 126hz, 4-16-11 170hz 5-25-10 213hz, 6-36-10 258hz 8-64-9 384hz  21-440-6 997hz
//12-144-8 536hz 17-288-7 785hz 
//考虑实验126hz 似乎摆动较大，也许和后面的ui滤波延迟有关，也没有明显改善yaw角飞行中漂移。
//漂移可能和室内外温差有关，室内校准的到了室外漂移就较大，另外也和震动有关，震动导致z轴的角速度波动。
//滤波力度大，带宽低时，温飘极大。如果要考虑这一点，还是要进行大带宽滤波。
//21-440-6 997hz, 试试这个温飘稳定性。当带宽达到这个程度时，数据几乎感觉不到温飘。可见滤波带宽越小温度影响越大。
//但第二次重复测试还是有温飘，这次先加热了。而且同温度下的数据也有漂移。感觉是温度变化速度对温飘影响很大。
//还是有温飘，很明显。所以还是不用这么高的带宽了。7-49-9 303hz  6-36-10 258hz
//42605也可以上到995hz带宽，到头了。
//带宽太大，Yaw角积分易漂移，可考虑选3-9-12
#define GYRO_AAF_DELT_42688 (4)
#define GYRO_AAF_DELTSQR_42688 (16)
#define GYRO_AAF_BITSHIFT_42688  (11)


#define ACCEL_AAF_DIS (0)  //1=disable, 0=enable


#define ACCEL_AAF_DELT_42605 (5)  //带宽53
#define ACCEL_AAF_DELTSQR_42605 (25)
#define ACCEL_AAF_BITSHIFT_42605 (10)

#define ACCEL_AAF_DELT_42688 (2)  //带宽84
#define ACCEL_AAF_DELTSQR_42688 (4) 
#define ACCEL_AAF_BITSHIFT_42688 (13)

//关闭陷波滤波看看
#define GYRO_NF_DIS (1)  //Disables bit of the gyro Notch Filter, 1=disable, 0=enable
//fdesired is the desired frequency of the Notch Filter in kHz, let fdesired=2khz, COSWZ=cos(2*pi*2/8)=0 COSWZ_SEL=0
#define GYRO_X_NF_COSWZ (0)
#define GYRO_Y_NF_COSWZ (0)
#define GYRO_Z_NF_COSWZ (0)
#define GYRO_X_NF_COSWZ_SEL (0)
#define GYRO_Y_NF_COSWZ_SEL (0)
#define GYRO_Z_NF_COSWZ_SEL (0)

#define GYRO_NF_BW_SEL_42605 (0)  //0-7 对应于不同带宽 362, 170, 83, 41, 21 ...
#define GYRO_NF_BW_SEL_42688 (3) //这个42688也不同于42605，2=329,3=162

//经过UI_FILT滤波，带宽大体是odr的一半
//考虑到陀螺仪的数据延迟涉及到平衡稳定，如果延迟偏大则机身倾角响应不及时，摇摆大。
//所以对于陀螺仪的滤波选择延迟小的。2阶，0带宽大约246hz，延迟1.3毫秒

#define GYRO_UI_FILT_ORD (1) //UI_FILT 阶数， 0=1阶，1=2阶，2=3阶，其他无定义。
#define ACCEL_UI_FILT_ORD (2) //高阶滤波稍慢一点

//陀螺仪这里考虑选1， odr/4的带宽。115hz左右。和上面的同步改。时间上会耽误2-3毫秒。噪音应该较小。
#define GYRO_UI_FILT_BW (2)  // 0= ODR/2, 1=max(400Hz, ODR)/4
#define ACCEL_UI_FILT_BW (2) //0=ODR/2, 1=max(400Hz, ODR)/4, 2=max(400Hz, ODR)/5, 带宽98hz, 5.4毫秒延迟。

//输出率，可选范围12.5，25，50，100，200，500，1000，2000，4000，8000
//以上的参数值都是选项值，这里直接给出输出率，而不是对于的选项值，代码里需要转换一下。
// #define GYRO_ODR  (500)
// #define ACCEL_ODR (500)

#define GYRO_ODR  (0b00001111) //500
#define ACCEL_ODR (0b00001111) //500

#define TEMP_FILT_BW (5)  //温度低通滤波带宽选择，这选10hz 


bool CICM426XX::Reset()
{
    //DEVICE_CONFIG do device reset.
    uint8_t buf[] = {0x11, 0x01};

    read_registers(0x11, buf, 1);
    write_onebyte(0x11, buf[0] | 0x01); 

    //reset后需要等待至少1毫秒
    sleep_ms(2);

    //WHO_AM_I
    //20230622 出现故障，这个who am i 会返回0x40, 不稳定，有时又正常有时不正常。
    //可能是前一天侧翻重摔导致这个主芯片虚焊了。压了下主芯片，似乎稳定了。
    read_registers(0x75, buf, 1);
    if(buf[0]== 0x42)
    {
        printf("confirm icm-42605\n");
        _chipType=0;
    }
    else if(buf[0]== 0x47)
    {
        printf("confirm icm-42688\n");
        _chipType=1;
    }
    else
    {
        printf("bad whoami %d\n", buf[0]);
        _chipType=0xFF; //不支持的芯片。
        return false;
    }

    //配置 SPI_SLEW_RATE, 4ns-12ns 
    //pico只有两档slew rate, 快和慢，并不知道时间。
    //42605默认是<2ns, 整个不动。


    //中断配置INT_CONFIG，采用默认的脉冲模式，高电平激发，push-pull模式。
    //这个配置如果采用默认设置可能不激发中断。
    write_onebyte(0x14, 0b11011);

    //FIFO_CONFIG采用默认模式。

    //PWR_MGMT0
    read_registers(0x4E, buf, 1);
    write_onebyte(0x4E, buf[0] | 0x0F); //全选低噪音模式(LN)

    //sleep_ms(30); //Gyroscope needs to be kept ON for a minimum of 45ms

    //GYRO_CONFIG0 ODR设置，范围设置
    //buf[0]= 0x4F;  
    read_registers(0x4F, buf, 1);
    buf[0] = buf[0] & 0b00010000; //第5位是保留位，不动。
    buf[0] = buf[0] | (GYRO_FS_SEL<<5); //高3位是范围选择
    buf[0] = buf[0] | GYRO_ODR; //输出率
    write_onebyte(0x4F, buf[0]);

    //ACCEL_CONFIG0 ODR设置，范围设置
    //4G, 500Hz ODR
    read_registers(0x50, buf, 1);
    buf[0] = buf[0] & 0b00010000; //第5位是保留位，不动。
    buf[0] = buf[0] | (ACCEL_FS_SEL<<5); //范围选择
    buf[0] = buf[0] | ACCEL_ODR;  //输出率
    
    write_onebyte(0x50, buf[0]);  

    //GYRO_CONFIG1
    //buf[0]= 0x51;
    //buf[1]= 0b10000110; //TEMP_FILT_BW DLPF BW = 20Hz, GYRO_UI_FILT_ORD 2rd Order,3rd Order
    //GYRO_UI_FILT_ORD 选择三阶滤波
    read_registers(0x51, buf, 1);
    buf[0] = buf[0] & 0b00010000; //第5位是保留位，不动。高3位是温度滤波
    buf[0] = buf[0] | (TEMP_FILT_BW<<5);
    buf[0] = buf[0] | (GYRO_UI_FILT_ORD <<2);
    buf[0] = buf[0] | 0b00000010; //GYRO_DEC2_M2_ORD, 无详细解释
    write_onebyte(0x51, buf[0]);

    //GYRO_ACCEL_CONFIG0
    //ACCEL_UI_FILT_BW, GYRO_UI_FILT_BW
    //UI_FILT低通滤波带宽设置，影响数据延迟
    read_registers(0x52, buf, 1);
    buf[0]= (ACCEL_UI_FILT_BW<<4) & GYRO_UI_FILT_BW;
    write_onebyte(0x52, buf[0]); 

    //ACCEL_CONFIG1   保持默认，2nd order filter.
    read_registers(0x53, buf, 1);
    buf[0] = buf[0] & 0b11100001; //清理低1-4位
    buf[0] = buf[0] | (ACCEL_UI_FILT_ORD<<3); 
    buf[0] = buf[0] | (2<<1); //ACCEL_DEC2_M2_ORD 无详细资料。
    write_onebyte(0x53, buf[0]);

    //APEX_CONFIG0
    //buf[0]= 0x56;
    //buf[1] = 2; //DMP_ODR=50hz
    //write_onebyte(0x56, 2);   

    //SMD_CONFIG default closed.
    //FIFO_CONFIG1 default closed;
    //INT_CONFIG0
    //buf[0]= 0x63;
    //buf[1] = 1<<5; //UI_DRDY_INT_CLEAR: Clear on Sensor Register Read
    write_onebyte(0x63, 1<<5);   

    //INT_CONFIG1 似乎操作没有影响。
    // user should change setting to 0 from default setting of 1,
    //for proper INT1 and INT2 pin operation
    // buf[0]= 0x64;
    // buf[1] = 0; 
    read_registers(0x64, buf, 1);
    write_onebyte(0x64, buf[0] & ~(0x10));


    //INT_SOURCE0
    //buf[0]= 0x65;
    //buf[1] = 1<<3;  // UI data ready interrupt routed to INT1
    write_onebyte(0x65, 1<<3);



    //INT_SOURCE1 default ok
   // INT_SOURCE3 default ok
    //buf[0]= 0x68;
    //buf[1] = 1<<3;  // UI data ready interrupt routed to INT2
    //因为一块imu损坏了INT管脚，所以软件打开了这里，即FSYNC做中断，然后板子上短接了INT/FSYNC
    //INT和FSYNC同时打开没问题，因为正常板子只接收INT管脚的中断，FSYNC悬空的。
    //v6的板子有缺陷要打开，v7用了新的芯片，没有问题，可关闭，应该也可打开，反正那个脚是悬空的。
    write_onebyte(0x68, 1<<3); 

   //INT_SOURCE4 default ok.

    //切换到bank1
   //REG_BANK_SEL
    write_onebyte(0x76, 1);

    //SENSOR_CONFIG0 default ok

    //GYRO_CONFIG_STATIC2 0x0b, 可在此关闭NF过滤和AAF过滤。
    read_registers(0x0b, buf, 1);
    buf[0] = buf[0] & 0b11111100; //清理后两位，默认打开。
    //0位是NF过滤开关，1位是AAF开关。选1是关闭，0是打开。
    if(GYRO_AAF_DIS)
    {
        buf[0] = buf[0] | 0x02;
    }

    if(GYRO_NF_DIS)
    {
        buf[0] = buf[0] | 0x01;
    }

    write_onebyte(0x0b, 1);


    if(_chipType==0)
    {
        //42605
    //GYRO_CONFIG_STATIC3
        //GYRO_AAF_DELT config
        read_registers(0x0C, buf, 1);
        buf[0] = buf[0] & 0b11000000; //清理后6位
        buf[0] = buf[0] | GYRO_AAF_DELT_42605; //DELT
        write_onebyte(0x0C, buf[0]);

        uint16_t deltasqt= GYRO_AAF_DELTSQR_42605;
        uint8_t hsqt= deltasqt>>8;
        uint8_t lsqt= deltasqt & 0x00FF;
        //GYRO_CONFIG_STATIC4
        //GYRO_AAF_DELTSQR config
        write_onebyte(0x0D, lsqt); //GYRO_AAF_DELTSQR低位，高位要写到GYRO_CONFIG_STATIC5里面去。

        //GYRO_CONFIG_STATIC5
        //GYRO_AAF_BITSHIFT  config
        write_onebyte(0x0E, (GYRO_AAF_BITSHIFT_42605<<4)|hsqt); //44混合，高4位是GYRO_AAF_BITSHIFT，低4位是hsqt
    }
    else if(_chipType==1)
    {
        //42688
            //GYRO_CONFIG_STATIC3
        //GYRO_AAF_DELT config
        read_registers(0x0C, buf, 1);
        buf[0] = buf[0] & 0b11000000; //清理后6位
        buf[0] = buf[0] | GYRO_AAF_DELT_42688; //DELT
        write_onebyte(0x0C, buf[0]);

        uint16_t deltasqt= GYRO_AAF_DELTSQR_42688;
        uint8_t hsqt= deltasqt>>8;
        uint8_t lsqt= deltasqt & 0x00FF;
        //GYRO_CONFIG_STATIC4
        //GYRO_AAF_DELTSQR config
        write_onebyte(0x0D, lsqt); //GYRO_AAF_DELTSQR低位，高位要写到GYRO_CONFIG_STATIC5里面去。

        //GYRO_CONFIG_STATIC5
        //GYRO_AAF_BITSHIFT  config
        write_onebyte(0x0E, (GYRO_AAF_BITSHIFT_42688<<4)|hsqt); //44混合，高4位是GYRO_AAF_BITSHIFT，低4位是hsqt
    }
    


    //下面几个参数都是NF设置。
    //NF_COSWZ=0, NF_COSWZ_SEL=0; 全部选0，因为fdesired选了2，所以算出来两个都是0
    //GYRO_X_NF_COSWZ
    write_onebyte(0x0F, GYRO_X_NF_COSWZ);   

    //GYRO_Y_NF_COSWZ
    write_onebyte(0x10, GYRO_Y_NF_COSWZ); 

    //GYRO_Z_NF_COSWZ
    write_onebyte(0x11, GYRO_Z_NF_COSWZ);  

    //GYRO_CONFIG_STATIC9
    read_registers(0x12, buf, 1);
    buf[0] = buf[0] & 0x11000000; //清理低6位。
    buf[0] = buf[0] | GYRO_X_NF_COSWZ;
    buf[0] = buf[0] | (GYRO_Y_NF_COSWZ<<1);
    buf[0] = buf[0] | (GYRO_Z_NF_COSWZ<<2);
    buf[0] = buf[0] | (GYRO_X_NF_COSWZ_SEL<<3);
    buf[0] = buf[0] | (GYRO_Y_NF_COSWZ_SEL<<4);
    buf[0] = buf[0] | (GYRO_Z_NF_COSWZ_SEL<<5);
    write_onebyte(0x12, buf[0]);

    if(_chipType==0)
    {
        //GYRO_CONFIG_STATIC10 Selects bandwidth for gyroscope notch filter.
        read_registers(0x13, buf, 1);
        buf[0]= buf[0] & 0b10001111;  //清理掉4，5，6三位。
        buf[0]= buf[0] | (GYRO_NF_BW_SEL_42605<<4);  //GYRO_NF_BW_SEL=0, 362 hz
        write_onebyte(0x13, buf[0]);
    }
    else if(_chipType==1)
    {
        //GYRO_CONFIG_STATIC10 Selects bandwidth for gyroscope notch filter.
        read_registers(0x13, buf, 1);
        buf[0]= buf[0] & 0b10001111;  //清理掉4，5，6三位。
        buf[0]= buf[0] | (GYRO_NF_BW_SEL_42688<<4);  //GYRO_NF_BW_SEL=0, 362 hz
        write_onebyte(0x13, buf[0]);  
    }

    //INTF_CONFIG5, pin9 function, int2/fync
    // buf[0]= 0x7B;
    // buf[1] = 0; //设置为int2
    // write_onebyte(0x7B, 0);   


    //切换到bank2
   //REG_BANK_SEL
    write_onebyte(0x76, 2);

    //加速度抗锯齿过滤器
    if(_chipType==0)
    {
        //42605
        
        //ACCEL_AAF_DELT, bank2
        read_registers(0x03, buf, 1);
        buf[0] = buf[0] & 0b10000000; //清理后7位
        buf[0] = buf[0] | (ACCEL_AAF_DELT_42605 <<1); 
        buf[0] = buf[0] | ACCEL_AAF_DIS;
        //buf[0] = buf[0] | 1; //最后一位是开关。1=disable, 0=enable
        write_onebyte(0x03, buf[0]); 

        uint16_t deltasqt= ACCEL_AAF_DELTSQR_42605;
        uint8_t hsqt= deltasqt>>8;
        uint8_t lsqt= deltasqt & 0x00FF;
        //ACCEL_AAF_DELTSQT, bank2
        write_onebyte(0x04, lsqt); //写低8位，高位的写到下一个寄存器的低位。
        
        //ACCEL_AAF_BITSHIFT, bank2
        write_onebyte(0x05, (ACCEL_AAF_BITSHIFT_42605<<4)|hsqt); //bitshift=7, 低位写入deltasqt的高位字节0x01
    }
    else if(_chipType==1)
    {
        //42688
        //ACCEL_AAF_DELT, bank2
        read_registers(0x03, buf, 1);
        buf[0] = buf[0] & 0b10000000; //清理后7位
        buf[0] = buf[0] | (ACCEL_AAF_DELT_42688 <<1); 
        buf[0] = buf[0] | ACCEL_AAF_DIS;
        //buf[0] = buf[0] | 1; //最后一位是开关。1=disable, 0=enable
        write_onebyte(0x03, buf[0]); 

        uint16_t deltasqt= ACCEL_AAF_DELTSQR_42688;
        uint8_t hsqt= deltasqt>>8;
        uint8_t lsqt= deltasqt & 0x00FF;
        //ACCEL_AAF_DELTSQT, bank2
        write_onebyte(0x04, lsqt); //写低8位，高位的写到下一个寄存器的低位。
        
        //ACCEL_AAF_BITSHIFT, bank2
        write_onebyte(0x05, (ACCEL_AAF_BITSHIFT_42688<<4)|hsqt); //bitshift=7, 低位写入deltasqt的高位字节0x01
    }



    //切换到bank0，方便以后读取测量数据。
   //REG_BANK_SEL
    write_onebyte(0x76, 0);

    return true;
}

void CICM426XX::Read(float accl[3], float gyro[3], float& temp)
{

    int16_t iacc[3], igyro[3], itemp;
    read_raw(iacc, igyro, itemp);
    for(size_t i=0;i<3;i++) {
        accl[i]= iacc[i]/8192.0f; //+-4G换算系数
        //accl[i]= iacc[i]/16384.0f; //+-2G换算系数
        gyro[i]= igyro[i]/65.5f; //500dps换算系数
    }
    temp= itemp/132.48 + 25;
}


void CICM426XX::read_raw(int16_t accel[3], int16_t gyro[3], int16_t& temp)
{
    //记得切换到bank0
//寄存器温度转换 (TEMP_DATA / 132.48) + 25
//FIFIO温度转换  (FIFO_TEMP_DATA / 2.07) + 25
    uint8_t buffer[14];
    read_registers(0x1D, buffer, 14);

    temp = (buffer[0]<<8) + buffer[1];

    for (size_t i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2 + 2] << 8 | buffer[(i * 2) + 1 + 2]);
    }

    for (size_t i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2 + 8] << 8 | buffer[(i * 2) + 1 + 8]);
    }
}