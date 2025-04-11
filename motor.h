
#ifndef _MOTOR_H
#define _MOTOR_H
#include "CMotorFilter.h"
//新的dshot支持，20230929
#define PROTOCOL_DSHOT
//#define PROTOCOL_PWM

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "randqueue.h"

#if defined PROTOCOL_DSHOT
#include "dshot_encoder.h"
#endif
//提供操作电机接口，不提供平衡计算方法。

struct _motor{
    uint8_t pin_num; //gpio num
    uint8_t slice_num;   //16 slices
    uint8_t channel_num;  //0/1
    float burden; //0.0-1.0
};

// //v6版本使用18，19，20，21四个口输出
// #define MOTOR_PINLF (18)
// #define MOTOR_PINRF (19)
// #define MOTOR_PINLR (20)
// #define MOTOR_PINRR (21)


#define MOTOR_PINLF (19)
#define MOTOR_PINRF (18)
#define MOTOR_PINLR (21)
#define MOTOR_PINRR (20)

//20240130 新增_thrust记录，代表垂直方向的总推力。
class mortor_burden
{
public:
    mortor_burden() {
        tmark=0;
        for(size_t i=0;i<4;i++) {
            _burden[i]=0;
        }
        _sum=0;
        _thrust=0;
    }
    mortor_burden(const float v) {
        tmark=0;
        _sum=v;
        for(size_t i=0;i<4;i++) {
            _burden[i]=v/4.0;
        }
    }

    mortor_burden& 
    operator+=(const mortor_burden &o)
    {
        _sum+=o._sum;
        for(size_t i=0;i<4;i++) {
            _burden[i]+=o._burden[i];
        }
        _thrust+= o._thrust;
        return *this;
    }  
      
    mortor_burden
    operator+(const mortor_burden &rhs)
    {
        mortor_burden o;
        o = *this;
        o += rhs;
        return o;
    }

    mortor_burden& operator=(const mortor_burden& o)
    {
        tmark=o.tmark;
        _sum=o._sum;
        for(size_t i=0;i<4;i++) {
            _burden[i]=o._burden[i];
        }
        _thrust=o._thrust;
        return *this;
    } 

    mortor_burden
    operator/(const int n)
    {
        mortor_burden ins;
        ins= *this;
        ins._sum/=n;
        for(size_t i=0;i<4;i++) {
            ins._burden[i]/=n;
        }
        ins._thrust/=n;
        return ins;
    }

    uint32_t tmark;
    float _sum;
    float _burden[4];
    float _thrust; //总体垂直方向的推力，20240130+
};


//#define USE_MOTOR_FILTER

class CMotor {
    public:
    //左前，右前，左后，右后分别对应的输出针脚
    bool Initialize(int pinLF=MOTOR_PINLF, int pinRF=MOTOR_PINRF, int pinLR=MOTOR_PINLR, int pinRR=MOTOR_PINRR);
    void GetBurdens(float burden[4]); //获得电机负担
    void SetBurdens(float burden[4], float thrust); //调整电机负担
    void SetBurdens(float b0, float b1, float b2, float b3, float thrust);
    float GetSumBurden();
    void GetHistoryMeanBurden(size_t hist, size_t num, float burden[4]); //获得历史负担数
    void Initialize(int pinLF, int pinRF, int pinLR, int pinRR, float duty);
    void Beep(); //dshot协议可发声。
    void GetProperReferBurdenForBalance(float burden[4]);
    float GetProperReferThrustForHeightControl();
    private:
#if defined PROTOCOL_PWM
    bool pwm_freq_set(pwm_config &cfg, int freq);
    void pwm_duty_set_u16(uint16_t duty_u16, uint slice, uint channel);
    uint16_t ToU16(float burden);
#endif

#if defined PROTOCOL_DSHOT
    DShotEncoder _LF, _RF, _LR, _RR;
    #if defined USE_MOTOR_FILTER
    CMotorFilter _LF_Filter;
    CMotorFilter _RF_Filter;
    CMotorFilter _LR_Filter;
    CMotorFilter _RR_Filter;
    #endif
#elif defined PROTOCOL_PWM
    //左前，右前，左后，右后
    _motor _LF, _RF, _LR, _RR; 
    uint _top, _div;
#endif

};

#endif