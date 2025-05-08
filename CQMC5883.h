#pragma once

//port i2c1

//v6版本使用26，27
// #define QMC5883_SDA (26) //26
// #define QMC5883_SCL (27) //27

//v7版本使用18，19
// #define QMC5883_SDA (18)
// #define QMC5883_SCL (19) 

//主板自带qmc5883
#define QMC5883_SDA (0) 
#define QMC5883_SCL (1) 

#define QMC5883L_ADDR 0x0D//The default I2C address is 0D: 0001101


//Registers Control //0x09

#define Mode_Standby    0b00000000
#define Mode_Continuous 0b00000001

#define ODR_10Hz        0b00000000
#define ODR_50Hz        0b00000100
#define ODR_100Hz       0b00001000
#define ODR_200Hz       0b00001100

#define RNG_2G          0b00000000
#define RNG_8G          0b00010000

#define OSR_512         0b00000000
#define OSR_256         0b01000000
#define OSR_128         0b10000000
#define OSR_64          0b11000000

#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "CI2cBase.h"

class CQMC5883 : public CI2cBase
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
        // int ret = i2c_read_blocking(i2c1, QMC5883L_ADDR, &rxdata, 1, false);
        // if(ret<0) {
        //     //printf("i2c device not exists\n");
        //     return false;
        // }

        int ret=raw_read(QMC5883L_ADDR, &rxdata, 1);
        if(ret<0) {
            printf("i2c device not exists, ret=%d\n", ret);
            return false;
        }
        else{
            printf("confirm qmc5883\n");
        }

        softReset(); //20230708+
        sleep_ms(50);

        //pio
        // int ret=raw_read(QMC5883L_ADDR, NULL, 0);
        // if(ret<0) {
        //     printf("i2c device not exists, ret=%d\n", ret);
        //     return false;      
        // }

        // uint8_t buf[2];
        // buf[0] = 0x0B;
        // buf[1] = 0x01;
        // i2c_write_blocking(i2c1, QMC5883L_ADDR, buf, 2, false);

        uint8_t buf[2];
        buf[0]=0x01;
        write_register(QMC5883L_ADDR, 0x0B, buf, 1);

        setMode(Mode_Continuous,ODR_50Hz,RNG_2G,OSR_64);
        return true;
    }

    void setMode(uint8_t mode,uint8_t odr,uint8_t rng,uint8_t osr){

        // uint8_t buf[2];
        // buf[0] = 0x09;
        // buf[1] = mode|odr|rng|osr;
        // i2c_write_blocking(i2c1, QMC5883L_ADDR, buf, 2, false);

        uint8_t buf[2];
        buf[0]= mode|odr|rng|osr;
        write_register(QMC5883L_ADDR, 0x09, buf, 1);
    }

    void softReset(){

        // uint8_t buf[2];
        // buf[0] = 0x0A;
        // buf[1] = 0x80;
        // i2c_write_blocking(i2c1, QMC5883L_ADDR, buf, 2, false);

        uint8_t buf[2];
        buf[0]=0x80;
        write_register(QMC5883L_ADDR, 0x0A, buf, 1);
    }

    bool read(int16_t& x,int16_t& y,int16_t& z)
    {

        uint8_t buf[6];

        //buf[0]=0x00;
        //i2c_write_blocking(i2c1, QMC5883L_ADDR, buf, 1, true);  // true to keep master control of bus
        //i2c_read_blocking(i2c1, QMC5883L_ADDR, buf, 6, false);  // false - finished with bus

        int n=read_registers(QMC5883L_ADDR, 0, buf, 6);
        if(n!=6) return false;

        x= buf[0]+ (buf[1]<<8);
        y= buf[2]+ (buf[3]<<8);
        z= buf[4]+ (buf[5]<<8);
        return true;
    }
};