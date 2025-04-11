
#include "bmp390_spi.h"
#include <stdio.h>

//spi 10 MHz, I2c 3.4Mhz
#define BMP390_ADDR (0x77)  //i2c有效
//#define BMP390_ADDR2 (0x77)

#define NVM_PAR_P11_Addr  0x45
#define NVM_PAR_P10_Addr  0x44
#define NVM_PAR_P9_H_Addr  0x43
#define NVM_PAR_P9_L_Addr  0x42
#define NVM_PAR_P8_Addr  0x41
#define NVM_PAR_P7_Addr  0x40
#define NVM_PAR_P6_H_Addr  0x3F
#define NVM_PAR_P6_L_Addr  0x3E
#define NVM_PAR_P5_H_Addr  0x3D
#define NVM_PAR_P5_L_Addr  0x3C
#define NVM_PAR_P4_Addr  0x3B
#define NVM_PAR_P3_Addr  0x3A
#define NVM_PAR_P2_H_Addr  0x39
#define NVM_PAR_P2_L_Addr  0x38
#define NVM_PAR_P1_H_Addr  0x37
#define NVM_PAR_P1_L_Addr  0x36
 
#define NVM_PAR_T3_Addr  0x35
#define NVM_PAR_T2_H_Addr  0x34
#define NVM_PAR_T2_L_Addr  0x33
#define NVM_PAR_T1_H_Addr  0x32
#define NVM_PAR_T1_L_Addr  0x31

#define Total_Number_32 4294967296.0
#define Total_Number_30 1073741824.0
#define Total_Number_29 536870912.0
#define Total_Number_24 16777216.0
#define Total_Number_20 1048576.0
#define Total_Number_16 65536.0
#define Total_Number_15 32768.0
#define Total_Number_14 16384.0
#define Total_Number_12 4096.0
#define Total_Number_8 256.0
#define Total_Number_6 64.0
#define Total_Number_5 32.0
#define Total_Number_1 2.0
 
#define Total_Number_Neg_8 0.00390625
#define Total_Number_Neg_3 0.125

//1+8+50+7 是50hz参数
//1+2+100+15 是100hz参数，噪音都是0.3pa
#define OSR_TEMP (1) //1,2
#define OSR_PRESS (8)  //1,2,4,8,16,32
#define ODR (50) //3.9.1 measurement time
//可能不宜太大，因为不能及时反应跌落，小点，允许部分噪音存在可以及时感知跌落。
//15似乎比7还好，高度波动小，气压波动小，发力波动小，31太大了，会冲顶
//100hz下，iir=7/15都还行
//50hz下，iir=15会有冲顶现象，估计是时序不对。这和100hz下iir=31一样的现象，iir的延迟是明显的。
//从效果看，100hz/iir=7或15，50hz/iir=7 这两者差不多。  100/15 或 50/7都有冲顶的可能。
//要避免冲顶可能，100hz搭配iir=7, 50hz搭配iir=3，这样数据噪音大点，平滑性上稍差了。
#define IIR (7)  //iir filter cof, 0,1,3,7,15,31,63,127， 7比15好，感知跌落更快。3不如7好，可能是噪音大

//根据测量时间推算8+1最大52.7hz, 2+1最大146hz,1+1最大207hz,4+1最大91.8hz
//4+1的配置最不实用，odr无法配置到90附近，只能选50，2+1只能选100hz，1+1利用率较好，刚好可选200hz.
//table 23:measurement time指明了测量时间和速率。

//bmp390在采用spi接口读取数据时，头部有一个dummy byte，这点似乎和icm42605不同。
void CBMP390_Spi::Read(float& press, float& temp)
{
    uint32_t ipress, itemp;
    read_raw(ipress, itemp);
    

    compensate_temperature(temp, itemp, ipress);
    compensate_pressure(press, temp, ipress);
}

void CBMP390_Spi::read_raw(uint32_t& ipress, uint32_t& itemp)
{
    uint8_t buf[6]; 
    read_registers(0x04, buf, 6);
    //三字节压力，三字节温度。低位在前。
    //按照设置，我们是8倍压力采样，是19位有效数据，一个整数是0.33pa
    //温度是一倍采样的，16位有效，一个整数是0.005度

    //printf("buf: %u,%u,%u,%u,%u,%u\n", buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
 
    /* Temporary variables to store the sensor data */
    uint32_t data_xlsb;
    uint32_t data_lsb;
    uint32_t data_msb;

    /* Store the parsed register values for pressure data */
    data_xlsb = (uint32_t)buf[0];
    data_lsb = (uint32_t)buf[1] << 8;
    data_msb = (uint32_t)buf[2] << 16;
    ipress = data_msb | data_lsb | data_xlsb;

    /* Store the parsed register values for temperature data */
    data_xlsb = (uint32_t)buf[3];
    data_lsb = (uint32_t)buf[4] << 8;
    data_msb = (uint32_t)buf[5] << 16;
    itemp = data_msb | data_lsb | data_xlsb;

    //printf("ipress=%u, itemp=%u\n", ipress, itemp);
}


void CBMP390_Spi::data_ready_check(bool& pr, bool& tr)
{
    uint8_t buf[2];
    read_registers(0x03, buf, 1); //read status;
    pr= buf[0] & 0b00100000;
    tr =buf[0] & 0b01000000;

    // if(!pr && !tr) {
    //     read_registers(BMP390_ADDR, 0x02, buf, 1);
    //     printf("error id=%u\n", buf[0]);
    // }
}

bool CBMP390_Spi::Reset()
{

    uint8_t buf[2] ={0};

    read_registers(0, buf, 1); //read chip id =0x60

    printf("chip id: %d\n", buf[0]);
    if(buf[0]!=0x60) {
        //printf("wrong chipid, %d\n", buf[0]);
        return false;
    }else{
        //printf("good chipid=%d\n", buf[0]);
        printf("confirm bmp390\n");
    }

    read_registers(0x01, buf, 1); //read rev id;

    //printf("rev id: %d\n", buf[0]);    


    //之前需要检查状态，是否接受命令？
    //Register 0x03 “STATUS”
    read_registers(0x03, buf, 1); //read status;

    if(buf[0] & 0b00010000) {
        printf("command ready!\n");
    }else{
        printf("command not ready\n");
    }

    //Register 0x7E “CMD”
    write_onebyte(0x7E, 0xB6); //soft reset.
    sleep_ms(100);

    //这是一个命令，检查命令是否执行成功。
    //Register 0x02 “ERR_REG”
    read_registers(0x02, buf, 1); 
    if(buf[0] & 0x01) {
        printf("fatal error\n");
    }else if(buf[0] & 0x02) {
        printf("comand fail\n");
    }else if(buf[0] & 0x04) {
        printf("sensor configuration error detected\n");
    }

    //读取补偿参数
    read_calib_data();


    //所有的设置读进来，修改后一起存储或许能解决问题。
    //从地址0x15开始读取，直到0x1F, 这个范围内是可修改的配置地址。
    uint8_t conf[11]={0}; //11个字节存在可配置数据。
    read_registers(0x15, conf, 11);

    // printf("conf first read:\n");
    // for(int i=0;i<11;i++) {
    //     printf("%02x,", conf[i]);
    // }
    // printf("\n");

   //0x15,0x16完全忽略。   conf[0], conf[1]
    //0x17 FIFO_CONFIG_1   conf[2]
    //保持不动，和bmp3设置相同。
    //printf("conf2 origin=%02x\n", conf[2]);
    //conf[2] = conf[2] & 0b11100000; //低5位置0，高三位不动。禁用fifo.
    //printf("conf2 after=%02x\n", conf[2]);

    //0x18 FIFO_CONFIG_2   conf[3]
    conf[3] = conf[3] & 0b11100000;

    //0x19 INT_CTRL         conf[4]
    conf[4] = conf[4] & 0b10000000;
    conf[4] = conf[4] | 0b01100010; //最高位不动，修改低7位。+int_ds=high

    //0x1A IF_CONF        conf[5]
    conf[5] = conf[5] & 0b11111000; //高位不动，低3位置零， spi 4线模式。

    //0x1B PWR_CTRL        conf[6]
    conf[6] = conf[6] | 0b00110011; //0145位置1，其他不动。气压温度全开，正常模式。
    
    //0x1C OSR            conf[7]
    conf[7]= conf[7] & 0b11000000; //清理低6位。
    //conf[7]= conf[7] | 0b00001100;  //2倍，16倍，可以搭配25hz输出。
    //conf[7]= conf[7] | 0b00001011;  //2倍，8倍，无法搭配50hz输出，有配置错误。
    //conf[7]= conf[7] | 0b00000011; //1倍+8倍，可以搭配50hz输出。无人机推荐配置。
    //conf[7]= conf[7] | 0b00000001;  //1倍+2倍，可以搭配100hz输出

    uint8_t osr_p=0;
    switch (OSR_PRESS)
    {
    case 1:
        osr_p=0;
        break;
    case 2:
        osr_p=1;
        break;
    case 4:
        osr_p=2;
        break;
    case 8:
        osr_p=3;
        break;
    case 16:
        osr_p=4;
        break;
    case 32:
        osr_p=5;
        break;
    default:
        break;
    }

    uint8_t osr_t=0;
    switch (OSR_TEMP)
    {
    case 1:
        osr_t =0;
        break;
    case 2:
        osr_t =1;
        break;
    default:
        break;
    }

    conf[7]= conf[7] | ((osr_t<<3)|osr_p);

    //0x1D ODR            conf[8]
    conf[8]= conf[8] & 0b11100000; //清理低5位。
    //附上最后2位10=50hz, 01=100hz, 00=200hz, 无法设置200hz采样率+2倍超采样。
    //估计是整个芯片的内部采样率最高只有200hz. 
    //conf[8]= conf[8] | 0b00000010; //50hz
    //conf[8]= conf[8] | 0b00000011; //25hz.
    //conf[8] = conf[8] | 0b00000001;  //100hz.

    uint8_t odr_sel;
    switch(ODR)
    {
        case 200:
        odr_sel=0;
        break;
        case 100:
        odr_sel=1;
        break;
        case 50:
        odr_sel=2;
        break;
        case 25:
        odr_sel=3;
        break;
        default:
        odr_sel=2;
    }

    conf[8] = conf[8] | odr_sel;

    //0x1E 保留           conf[9]
    //0x1F CONFIG         conf[10]
    conf[10]= conf[10] & 0b11110001; //清理123位。
    //conf[10]= conf[10] | 0b00000010; //附上123位011=filter coefficient is 7， 001=filter coefficient is 1

    uint8_t iir_coef;
    switch (IIR)
    {
    case 0:
        iir_coef=0;
        break;
    case 1:
        iir_coef=1;
        break;
    case 3:
        iir_coef=2;
        break;
    case 7:
        iir_coef=3;
        break;
    case 15:
        iir_coef=4;
        break;
    case 31:
        iir_coef=5;
        break;          
    case 63:
        iir_coef=6;
        break;       
    case 127:
        iir_coef=7;
        break;             
    default:
        break;
    }

    conf[10]= conf[10] | (iir_coef<<1);
    
    // printf("conf before write:\n");
    // for(int i=0;i<11;i++) {
    //     printf("%02x,", conf[i]);
    // }
    // printf("\n");



    //bmp390的i2c不支持批量写，只能一个一个写
    write_onebyte(0x15, conf[0]);
    write_onebyte(0x16, conf[1]);
    write_onebyte(0x17, conf[2]);
    write_onebyte(0x18, conf[3]);
    write_onebyte(0x19, conf[4]);
    write_onebyte(0x1A, conf[5]);
    write_onebyte(0x1B, conf[6]);
    write_onebyte(0x1C, conf[7]);
    write_onebyte(0x1D, conf[8]);
    write_onebyte(0x1E, conf[9]);
    write_onebyte(0x1F, conf[10]);
    sleep_ms(10);

//read again and check.
    for(int repeat=0; repeat<3; repeat++)
    {
        //保证数据写到位
        uint8_t conf_check[11]={0};
        read_registers(0x15, conf_check, 11);
        printf("conf read again:\n");
        for(int i=0;i<11;i++) {
            printf("%02x,", conf_check[i]);
        }
        printf("\n");

        //检查有错误的地方再写一次。
        bool error=false;
        for(uint8_t i=0;i<11;i++)
        {
            if(conf_check[i]!=conf[i])
            {
                printf("write again conf[%d]=%02x\n", i, conf[i]);
                error=true;
                write_onebyte(0x15+i, conf[i]);
                sleep_ms(10);
            }
        }

        if(!error) {
            break;
        }
    }
    
    read_registers(0x02, buf, 1); 
    if(buf[0] & 0x01) {
        printf("fatal error\n");
    }else if(buf[0] & 0x02) {
        printf("comand fail\n");
    }else if(buf[0] & 0x04) {
        printf("sensor configuration error detected\n");
    }
    
    return true;

}

void CBMP390_Spi::Read_Interrupt_Status(uint8_t& status)
{
    read_registers(0x11, &status, 1);
}

//read all calib data, 21bytes.
void CBMP390_Spi::read_calib_data()
{
    uint8_t buf[21];
    read_registers(0x31, buf, 21);
    parse_calib_data(buf);

    // if(nr==21) {
    //     printf("read calib data return good\n");
    // }else{
    //     printf("read request 21, return %d bytes\n", nr);
    // }

    // printf("raw calib data:\n");
    // for(int i=0;i<21;i++) {
    //     printf("%02x,", buf[i]);
    // }
    // printf("\n");

    //printf("pt1=%f,pt2=%f,pt3=%f\n", quantized_par_t1, quantized_par_t2, quantized_par_t3);
    //printf("pp1=%f,pp2=%f,pp3=%f,pp4=%f,pp5=%f,pp6=%f\n", quantized_par_p1, quantized_par_p2, quantized_par_p3, quantized_par_p4, quantized_par_p5, quantized_par_p6);
    //printf("pp7=%f,pp8=%f,pp9=%f,pp10=%f,pp11=%f\n", quantized_par_p7, quantized_par_p8, quantized_par_p9, quantized_par_p10, quantized_par_p11);
}

void CBMP390_Spi::parse_calib_data(uint8_t reg_data[])
{
    #define CONCAT_BYTES(msb, lsb)             (((uint16_t)msb << 8) | (uint16_t)lsb)
    ///* Temporary variable to store the aligned trim data */
    //struct bmp3_reg_calib_data *reg_calib_data = &dev->calib_data.reg_calib_data;
    //struct bmp3_quantized_calib_data *quantized_calib_data = &dev->calib_data.quantized_calib_data;

    /* Temporary variable */
    double temp_var;

    /* 1 / 2^8 */
    temp_var = 0.00390625f;
    uint16_t par_t1 = CONCAT_BYTES(reg_data[1], reg_data[0]);
    quantized_par_t1 = ((float)par_t1 / temp_var);

    uint16_t par_t2 = CONCAT_BYTES(reg_data[3], reg_data[2]);
    temp_var = 1073741824.0f;
    quantized_par_t2 = ((float)par_t2 / temp_var);

    int8_t par_t3 = (int8_t)reg_data[4];
    temp_var = 281474976710656.0f;
    quantized_par_t3 = ((float)par_t3 / temp_var);

    int16_t par_p1 = (int16_t)CONCAT_BYTES(reg_data[6], reg_data[5]);
    temp_var = 1048576.0f;
    quantized_par_p1 = ((float)(par_p1 - (16384)) / temp_var);
    int16_t par_p2 = (int16_t)CONCAT_BYTES(reg_data[8], reg_data[7]);
    temp_var = 536870912.0f;
    quantized_par_p2 = ((float)(par_p2 - (16384)) / temp_var);
    int8_t par_p3 = (int8_t)reg_data[9];
    temp_var = 4294967296.0f;
    quantized_par_p3 = ((float)par_p3 / temp_var);
    int8_t par_p4 = (int8_t)reg_data[10];
    temp_var = 137438953472.0f;
    quantized_par_p4 = ((float)par_p4 / temp_var);

    uint16_t par_p5 = CONCAT_BYTES(reg_data[12], reg_data[11]);

    /* 1 / 2^3 */
    temp_var = 0.125f;
    quantized_par_p5 = ((float)par_p5 / temp_var);
    uint16_t par_p6 = CONCAT_BYTES(reg_data[14], reg_data[13]);
    temp_var = 64.0f;
    quantized_par_p6 = ((float)par_p6 / temp_var);
    int8_t par_p7 = (int8_t)reg_data[15];
    temp_var = 256.0f;
    quantized_par_p7 = ((float)par_p7 / temp_var);
    int8_t par_p8 = (int8_t)reg_data[16];
    temp_var = 32768.0f;
    quantized_par_p8 = ((float)par_p8 / temp_var);
    int16_t par_p9 = (int16_t)CONCAT_BYTES(reg_data[18], reg_data[17]);
    temp_var = 281474976710656.0f;
    quantized_par_p9 = ((float)par_p9 / temp_var);
    int8_t par_p10 = (int8_t)reg_data[19];
    temp_var = 281474976710656.0f;
    quantized_par_p10 = ((float)par_p10 / temp_var);
    int8_t par_p11 = (int8_t)reg_data[20];
    temp_var = 36893488147419103232.0f;
    quantized_par_p11 = ((float)par_p11 / temp_var);

}


void CBMP390_Spi::compensate_temperature(float& temperature,
                                     uint32_t itemp, //raw data
                                     uint32_t ipress //raw data
                                     )
{

    int64_t uncomp_temp = itemp;
    float partial_data1;
    float partial_data2;

    partial_data1 = (float)(uncomp_temp - quantized_par_t1);
    partial_data2 = (float)(partial_data1 * quantized_par_t2);

    /* Update the compensated temperature in calib structure since this is
     * needed for pressure calculation */
    temperature = partial_data2 + (partial_data1 * partial_data1) * quantized_par_t3;

#define BMP3_MIN_TEMP_DOUBLE     (-40.0f)
#define BMP3_MAX_TEMP_DOUBLE     (85.0f)
    /* Returns compensated temperature */
    if (temperature < BMP3_MIN_TEMP_DOUBLE)
    {
        temperature = BMP3_MIN_TEMP_DOUBLE;
    }

    if (temperature > BMP3_MAX_TEMP_DOUBLE)
    {
        temperature = BMP3_MAX_TEMP_DOUBLE;
    }

}


void CBMP390_Spi::compensate_pressure(float &pressure,
                            float temprature, //补偿后的温度
                            uint32_t ipress //raw data
)
{

    /* Variable to store the compensated pressure */
    //double comp_press;

    /* Temporary variables used for compensation */
    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_data4;
    float partial_out1;
    float partial_out2;

    partial_data1 = quantized_par_p6 * temprature;
    partial_data2 = quantized_par_p7 * pow_bmp3(temprature, 2);
    partial_data3 = quantized_par_p8 * pow_bmp3(temprature, 3);
    partial_out1 = quantized_par_p5 + partial_data1 + partial_data2 + partial_data3;
    partial_data1 = quantized_par_p2 * temprature;
    partial_data2 = quantized_par_p3 * pow_bmp3(temprature, 2);
    partial_data3 = quantized_par_p4 * pow_bmp3(temprature, 3);
    partial_out2 = ipress * (quantized_par_p1 + partial_data1 + partial_data2 + partial_data3);
    partial_data1 = pow_bmp3((float)ipress, 2);
    partial_data2 = quantized_par_p9 + quantized_par_p10 * temprature;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + pow_bmp3((float)ipress, 3) * quantized_par_p11;
    pressure = partial_out1 + partial_out2 + partial_data4;

#define BMP3_MIN_PRES_DOUBLE   30000.0f
#define BMP3_MAX_PRES_DOUBLE   125000.0f

    if (pressure < BMP3_MIN_PRES_DOUBLE)
    {
        pressure = BMP3_MIN_PRES_DOUBLE;
    }

    if (pressure > BMP3_MAX_PRES_DOUBLE)
    {
        pressure = BMP3_MAX_PRES_DOUBLE;
    }

}