#pragma once

#include "CSpiBase.h"

//老的方版设计
// #define BMP390_PIN_INT  6
// #define BMP390_PIN_SCK  2
// #define BMP390_PIN_MOSI 3
// #define BMP390_PIN_MISO 4
// #define BMP390_PIN_CS   5
// #define BMP390_SPI_PORT 0

//20240515 两个传感器做了交换位置。
#define BMP390_PIN_SCK  10
#define BMP390_PIN_MOSI 11
#define BMP390_PIN_MISO 12
#define BMP390_PIN_CS   13
#define BMP390_PIN_INT  14  //14 22 is test port
#define BMP390_SPI_PORT 1

class CBMP390_Spi : public CSpiBase
{
    public:
    bool Reset();
    void Read(float& press, float& temp);
    void data_ready_check(bool& pr, bool& tr);
    void Read_Interrupt_Status(uint8_t& status);
    protected:
    void read_raw(uint32_t& ipress, uint32_t& itemp);
    void Parameter_Reading(int Pressure_Para[],int Temperature_Para[]);

    private:
    void read_calib_data();
    void parse_calib_data(uint8_t data[]);
    void compensate_temperature(float &temperature,
                                     uint32_t itemp, //raw data
                                     uint32_t ipress //raw data
                                     );
    void compensate_pressure(float &pressure,
                            float temprature, //补偿后的温度
                            uint32_t ipress //raw data
    );

    float pow_bmp3(float base, uint8_t power)
    {
        float pow_output = 1;

        while (power != 0)
        {
           pow_output = base * pow_output;
           power--;
        }

        return pow_output;
    }

    uint8_t device_addr;

    float quantized_par_t1;
    float quantized_par_t2;
    float quantized_par_t3;
    float quantized_par_p0;
    float quantized_par_p1;
    float quantized_par_p2;
    float quantized_par_p3;
    float quantized_par_p4;
    float quantized_par_p5;
    float quantized_par_p6;
    float quantized_par_p7;
    float quantized_par_p8;
    float quantized_par_p9;
    float quantized_par_p10;
    float quantized_par_p11;    
};