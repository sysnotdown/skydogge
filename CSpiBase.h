#pragma once
#include "hardware/spi.h"
#include "pico/stdlib.h"

class CSpiBase
{

public:
//bmp390读取数据时，先有一个字节的dummy，这点似乎Icm42605没有。
    void Initialize(uint8_t spi_port, uint8_t pin_sck, uint8_t pin_miso, uint8_t pin_mosi, uint8_t pin_cs, uint32_t speed, uint8_t dummy)
    {
        if(spi_port)
            spi_init(spi1, speed);
        else
            spi_init(spi0, speed);

        gpio_set_function(pin_miso, GPIO_FUNC_SPI);
        gpio_set_function(pin_sck, GPIO_FUNC_SPI);
        gpio_set_function(pin_mosi, GPIO_FUNC_SPI);
        //cs每设备一根，不共用。spi0配备了三个cs线路，gp1,gp5,gp17, spi1配备了2个cs接口gp9,gp13
        //是否cs可以任选一个gpio不确定。
        gpio_set_function(pin_cs, GPIO_FUNC_SPI); //是否需要设置? 目前不设置也可以工作

        //bmp390如果拉高cs, 则进入i2c模式，所以不能拉高。尝试过后还是不行，放弃
        // Chip select is active-low, so we'll initialise it to a driven-high state
        
        gpio_init(pin_cs);
        gpio_set_dir(pin_cs, GPIO_OUT);
        gpio_put(pin_cs, 1);

        _spi_port=spi_port;
        _pin_cs=pin_cs;
        _pin_miso=pin_miso;
        _pin_mosi=pin_mosi;
        _pin_sck=pin_sck;
        _speed=speed;
        _dummy=dummy;
    }

    //有dummy存在时，限制一次最多读取127字节。
    int read_registers(uint8_t reg, uint8_t *buf, uint16_t len) 
    {

        //The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation.
    // The following 7 bits contain the Register Address. In cases of multiple-byte Read/Writes, data is
    // two or more bytes:
        if(_dummy)
        {
            uint8_t lbuf[128];
            reg |= 0x80;
            cs_select();
            int nret;
            if(_spi_port)
            {
                spi_write_blocking(spi1, &reg, 1);
                nret=spi_read_blocking(spi1, 0, lbuf, len+_dummy);
                for(size_t i=0;i<len;i++) buf[i]=lbuf[i+_dummy];
            }
            else
            {
                spi_write_blocking(spi0, &reg, 1);
                nret=spi_read_blocking(spi0, 0, lbuf, len+_dummy); 
                for(size_t i=0;i<len;i++) buf[i]=lbuf[i+_dummy];
            }
            cs_deselect();
            return nret-_dummy;
        }
        else
        {
            reg |= 0x80;
            cs_select();
            int nret;
            if(_spi_port)
            {
                spi_write_blocking(spi1, &reg, 1);
                nret=spi_read_blocking(spi1, 0, buf, len);
            }
            else
            {
                spi_write_blocking(spi0, &reg, 1);
                nret=spi_read_blocking(spi0, 0, buf, len); 
            }
            cs_deselect();
            return nret;
        }
    }

    void write_onebyte(uint8_t reg, uint8_t val)
    {
        uint8_t buf[2];
        buf[0]=reg;
        buf[1]=val;

        cs_select();
        if(_spi_port)
            spi_write_blocking(spi1, buf, 2);
        else
            spi_write_blocking(spi0, buf, 2);
        cs_deselect();  
    }

protected:
    inline void cs_select() {
        asm volatile("nop \n nop \n nop");
        gpio_put(_pin_cs, 0);  // Active low
        asm volatile("nop \n nop \n nop");
    }

    inline void cs_deselect() {
        asm volatile("nop \n nop \n nop");
        gpio_put(_pin_cs, 1);
        asm volatile("nop \n nop \n nop");
    }

protected:
    uint8_t _spi_port;
    uint8_t _pin_cs;
    uint8_t _pin_mosi;
    uint8_t _pin_miso;
    uint8_t _pin_sck;
    uint32_t _speed;
    uint8_t _dummy;
};