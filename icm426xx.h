
#pragma once
#include "hardware/spi.h"
#include "pico/stdlib.h"

#include "CSpiBase.h"
#include "imu.h"



//电路板上的端口，主要是用了10，11，12，14一组spi,外加14中断。
// #define ICM426XX_PIN_SCK  10
// #define ICM426XX_PIN_MOSI 11
// #define ICM426XX_PIN_MISO 12
// #define ICM426XX_PIN_CS   13
// #define ICM426XX_PIN_INT  14
// #define ICM426XX_PIN_FSYNC  15
// #define ICM426XX_SPI_PORT 1

//20240515 两个传感器做了交换，Imu在前，baro在后
#define ICM426XX_PIN_SCK  2
#define ICM426XX_PIN_MOSI 3
#define ICM426XX_PIN_MISO 4
#define ICM426XX_PIN_CS   5
#define ICM426XX_PIN_INT  6
#define ICM426XX_SPI_PORT 0

class CICM426XX : public CSpiBase, public ImuBase
{
    public:
    bool Reset();
    void Read(float accl[3], float gyro[3], float& temp);
    private:
    void read_raw(int16_t accel[3], int16_t gyro[3], int16_t& temp);
    protected:
        uint8_t _chipType; //0:42605,1:42688, 其他无定义。
};

