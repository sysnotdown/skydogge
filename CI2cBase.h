#pragma once
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <stdio.h>

#if defined USE_PIO_I2C
#include "i2c.pio.h"
#include "pio_i2c.h"

// extern PIO  pio_i2c ;
// extern uint sm_i2c ;
// extern uint offset_i2c;

#endif

class CI2cBase
{
    public:

    #if defined USE_PIO_I2C    
        void Pio_initialize(PIO pio, uint sm, uint8_t pin_sda, uint8_t pin_scl)
        {
            _pio=pio;
            _sm=sm;



            //不能设置这个，否则读数据卡死。
            // if(pio==pio0)
            // {
            //     gpio_set_function(pin_sda, GPIO_FUNC_PIO0);
            //     gpio_set_function(pin_scl, GPIO_FUNC_PIO0);
            // }
            // else
            // {
            //     gpio_set_function(pin_sda, GPIO_FUNC_PIO1);
            //     gpio_set_function(pin_scl, GPIO_FUNC_PIO1);
            // }
        }
        
    #else
    void Initialize(uint8_t iport, uint8_t pin_scl, uint8_t pin_sda, uint32_t speed)
        {
            if(iport)
                i2c_init(i2c1, speed);
            else
                i2c_init(i2c0, speed);

            //i2c_init(&i2c1_inst, 100000);//400000 as default. 4000 fail
            gpio_set_function(pin_sda, GPIO_FUNC_I2C);
            gpio_set_function(pin_scl, GPIO_FUNC_I2C);
            gpio_pull_up(pin_sda);
            gpio_pull_up(pin_scl);

            _i2c_port=iport;
            _pin_scl=pin_scl;
            _pin_sda=pin_sda;
            _speed=speed;
        }
    #endif

    protected:
    #if defined USE_PIO_I2C
        int raw_read(uint8_t device_addr, uint8_t* buf, int16_t len)
        {
            return pio_i2c_read_blocking(_pio, _sm, device_addr, buf, len);
        }
    #else
        int raw_read(uint8_t device_addr, uint8_t* buf, int16_t len)
        {
            if(_i2c_port)
            {
                return i2c_read_blocking(i2c1, device_addr, buf, len, false);
            }
            else
            {
                return i2c_read_blocking(i2c0, device_addr, buf, len, false);
            }
        }
    #endif

    #if defined USE_PIO_I2C
        int read_registers(uint8_t device_addr, uint8_t reg_addr, uint8_t *buf, uint16_t len) 
        {
            int nret=pio_i2c_write_blocking(_pio, _sm, device_addr, &reg_addr, 1);
            nret= pio_i2c_read_blocking(_pio, _sm, device_addr, buf, len); 
            return nret;
        }
    #else
        int read_registers(uint8_t device_addr, uint8_t reg_addr, uint8_t *buf, uint16_t len) 
        {
            if(_i2c_port)
            {
                i2c_write_blocking(i2c1, device_addr, &reg_addr, 1, true);
                int n=i2c_read_blocking(i2c1, device_addr, buf, len, false);
                return n;
            }
            else
            {
                i2c_write_blocking(i2c0, device_addr, &reg_addr, 1, true);
                int n=i2c_read_blocking(i2c0, device_addr, buf, len, false);
                return n;
            }
        }
    #endif

    #if defined USE_PIO_I2C
        int write_onebyte(uint8_t device_addr, uint8_t reg_addr, uint8_t val)
        {
            uint8_t buf[2];
            buf[0]=reg_addr;
            buf[1]=val;
            return pio_i2c_write_blocking(_pio, _sm, device_addr, buf, 2);
        }

    #else
        int write_onebyte(uint8_t device_addr, uint8_t reg_addr, uint8_t val)
        {
            uint8_t buf[2];
            buf[0]=reg_addr;
            buf[1]=val;
            if(_i2c_port)
            {
                return i2c_write_blocking(i2c1, device_addr, buf, 2, true);
            }
            else 
            {
                return i2c_write_blocking(i2c0, device_addr, buf, 2, true);
            }
            
        }
    #endif

    #if defined USE_PIO_I2C
        int write_register(uint8_t device_addr, uint8_t reg_addr, uint8_t* buf, uint8_t len)
        {
            uint8_t nbuf[256];
            nbuf[0]=reg_addr;
            for(size_t i=0;i<len;i++) nbuf[1+i]=buf[i];
            return pio_i2c_write_blocking(_pio, _sm, device_addr, nbuf, len+1);
        }
    #else
        int write_register(uint8_t device_addr, uint8_t reg_addr, uint8_t* buf, uint8_t len)
        {

            if(_i2c_port)
            {
                uint8_t nbuf[256];
                nbuf[0]=reg_addr;
                for(size_t i=0;i<len;i++) nbuf[1+i]=buf[i];

                return i2c_write_blocking(i2c1, device_addr, nbuf, len+1, true);
            }
            else 
            {
                uint8_t nbuf[256];
                nbuf[0]=reg_addr;
                for(size_t i=0;i<len;i++) nbuf[1+i]=buf[i];

                return i2c_write_blocking(i2c0, device_addr, nbuf, len+1, true);
            }
        }

    #endif    

        #if defined USE_PIO_I2C
        PIO _pio;
        uint _sm;
        #else
        uint8_t _i2c_port;
        uint8_t _pin_scl;
        uint8_t _pin_sda;
        uint32_t _speed;
        #endif
};