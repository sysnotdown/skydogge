#pragma once

//port i2c0

//v8版本在飞控板上增加了hmc5883L, 使用0,1端口，i2c0端口。
#define HMC5883_SDA (0) 
#define HMC5883_SCL (1) 


#define HMC5883L_ADDR 0x1E//和QMC5883L不同

//Registers Control //0x09

// #define Mode_Standby    0b00000000
// #define Mode_Continuous 0b00000001

// #define ODR_10Hz        0b00000000
// #define ODR_50Hz        0b00000100
// #define ODR_100Hz       0b00001000
// #define ODR_200Hz       0b00001100

#define ODR_0_75Hz          0b00000000      //0.75hz
#define ODR_1_5Hz           0b00000100   //1.5hz
#define ODR_3Hz             0b00001000   //3hz
#define ODR_7_5Hz           0b00001100   //7.5hz
#define ODR_15Hz            0b00010000   //15hz
#define ODR_30Hz            0b00010100   //30hz
#define ODR_75Hz            0b00011000   //75hz

#define AVG_SAM_0           0b00000000  //输出采样均值数=0
#define AVG_SAM_2           0b00100000  //输出采样均值数=2
#define AVG_SAM_4           0b01000000  //输出采样均值数=4
#define AVG_SAM_8           0b01100000  //输出采样均值数=8

//以上两个重叠，设置在0x00地址上。

//测量范围，单独写入寄存器0x01地址
#define RANGE_0_88      0b00000000
#define RANGE_1_3       0b00100000
#define RANGE_1_9       0b01000000
#define RANGE_2_5       0b01100000
#define RANGE_4_0       0b10000000
#define RANGE_4_7       0b10100000
#define RANGE_5_6       0b11000000
#define RANGE_8_1       0b11100000

//第三个配置地址0x02直接全部写0，这样就进入连续测量模式。

// #define RNG_2G          0b00000000
// #define RNG_8G          0b00010000

// #define OSR_512         0b00000000
// #define OSR_256         0b01000000
// #define OSR_128         0b10000000
// #define OSR_64          0b11000000

#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "CI2cBase.h"


class CHMC5883 : public CI2cBase
{
    public:

    bool init(){

        // gpio_init(QMC5883_SDA);
        // gpio_init(QMC5883_SCL);
        // i2c_init(i2c1, 400*1000); //传感器最大400khz
        // gpio_set_function(QMC5883_SDA, GPIO_FUNC_I2C);
        // gpio_set_function(QMC5883_SCL, GPIO_FUNC_I2C);
        // gpio_pull_up(QMC5883_SDA);
        // gpio_pull_up(QMC5883_SCL);

        uint8_t rxdata;

        int ret=raw_read(HMC5883L_ADDR, &rxdata, 1);
        if(ret<0) {
            printf("i2c device not exists, ret=%d\n", ret);
            return false;
        }
        else{
            printf("confirm hmc5883\n");
        }

        uint8_t wxdata;
        wxdata= ODR_75Hz|AVG_SAM_4;

        write_register(HMC5883L_ADDR, 0x00, &wxdata, 1);
        wxdata= RANGE_0_88;
        write_register(HMC5883L_ADDR, 0x01, &wxdata, 1);
        wxdata=0;
        write_register(HMC5883L_ADDR, 0x02, &wxdata, 1); //连续采样
        return true;
    }


    bool read(int16_t& x,int16_t& y,int16_t& z)
    {

        uint8_t buf[6];

        //0x03-0x08 是x,z,y三轴数据
        int n=read_registers(HMC5883L_ADDR, 0x03, buf, 6);
        if(n!=6) return false;
        //对比qmc5883,顺序不同,高位低位也不同
        x= (buf[0]<<8)+ buf[1];
        z= (buf[2]<<8)+ buf[3];
        y= (buf[4]<<8)+ buf[5];

        //调节安装方向，保持和GPS上的指南针一致。
        x=-x;

        return true;
    }
};