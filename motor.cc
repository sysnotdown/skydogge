#include "motor.h"
#include <math.h>
#include <set>
#include <string>
//16.8v下，2212电机 kv980，burden, 电流 测试表。

//0.1  0.184
//0.2  0.516
//0.3  1.063
//0.4  1.99
//0.5  3.45
//0.6  5.39
//0.7  7.4

//电压16.8v
//2212机器不带电池测试，重量880克，平衡高度电流大约6.7安，全电流。四电机总负载大约1.5.
//增加100克总量，980克，平衡电流大约7.8安，7.6-8.0左右摇摆。电机总负载大约1.5，变化不明显。


//16V下，3110电机 kv480， burden, 电流, 功耗实测
//0.2, 0.3A, 4.8W，
//0.3, 0.8A, 13W
//0.4，1.55A, 25w
//0.5, 2.6A, 42w
//0.6, 4.1A, 66w  这个档位应该稍差一点
//0.7, 5.88A, 94W 这个挡位应该能够起飞
//0.8, 7.85A, 125W
//0.9, 10A+, 155w+ 可调电源达到电流峰值，电压下降，没测出来。轻易不要上到这个负担上来。
//1.0, 估计11.5A, 184w
//主要调节区间可能在0.5-0.9之间。

//拉力表，
//2A 402
//4A 622
//6A 812
//8A 955
//10A 1098
//11.5A  1185

//那么油门信号量和拉力的关系推算?
extern uint32_t get_time_mark();
extern void logmessage(std::string str);

//这个队列之前100，在500hz下不够用，调节为300.
//目前追溯最远是0.2秒，500hz下100就够用了，放松点限制在120的长度。主要是平衡控制需要引用历史推力参考
//高度推力参考值不使用这个数据计算，另有数据。
//120不够啊，20231012才发现，不知道何时改成了这个数字，但平衡参数一直是前推0.1秒，求0.3-0.1秒的均值，所以至少需要0.3秒的存储。
//0.3秒在500hz下，至少是150个长度，这里不知道何时范的错。现在修改为160.
NRandQueue<mortor_burden> _burden_queue(160);

#if defined PROTOCOL_PWM
bool CMotor::pwm_freq_set(pwm_config &cfg, int freq) {
    // Set the frequency, making "top" as large as possible for maximum resolution.
    // Maximum "top" is set at 65534 to be able to achieve 100% duty with 65535.
    #define TOP_MAX 65534
    uint32_t source_hz = clock_get_hz(clk_sys);
    uint32_t div16_top = 16 * source_hz / freq;
    uint32_t top = 1;
    for (;;) {
        // Try a few small prime factors to get close to the desired frequency.
        if (div16_top >= 16 * 5 && div16_top % 5 == 0 && top * 5 <= TOP_MAX) {
            div16_top /= 5;
            top *= 5;
        } else if (div16_top >= 16 * 3 && div16_top % 3 == 0 && top * 3 <= TOP_MAX) {
            div16_top /= 3;
            top *= 3;
        } else if (div16_top >= 16 * 2 && top * 2 <= TOP_MAX) {
            div16_top /= 2;
            top *= 2;
        } else {
            break;
        }
    }
    if (div16_top < 16) {
        //mp_raise_ValueError(MP_ERROR_TEXT("freq too large"));
        return false;
    } else if (div16_top >= 256 * 16) {
        //mp_raise_ValueError(MP_ERROR_TEXT("freq too small"));
        return false;
    }
    
    cfg.div=div16_top;
    cfg.top=top-1;

    return true;
}

void CMotor::pwm_duty_set_u16(uint16_t duty_u16, uint slice, uint channel) {
    uint32_t cc = duty_u16 * (_top + 1) / 65535; 
    pwm_set_chan_level(slice, channel, cc);
}

//最低油门数字32700，最高65000，
//65535这个信号几乎满格，好盈电调检测不到信号，所以高位要调低
//目前观察，0.9的油门电流已经超过了10A, 接近满载，所以估计61700+的信号电调区分度已经不高了。

//而低位信号不能少于高位的一半。
uint16_t CMotor::ToU16(float burden)
{
    return 32700 + uint16_t(burden*32300);
}

#endif

//20230216发现，需要避免相同slice下的两个通道重复初始化问题。
//以前选用的输出可能都是不同slice下的通道，电路板上选用了相邻的针脚，具有相同的slice，会有重复初始化问题。
bool CMotor::Initialize(int pinLF, int pinRF, int pinLR, int pinRR)
{

    #if defined PROTOCOL_PWM
    //记录已经初始化的slice, 避免重复初始化。
    std::set<int> slice_num_initialed;

    pwm_config cfg=pwm_get_default_config();
    pwm_freq_set(cfg, 500); //在pico下这个频率不会失败

    _div=cfg.div;
    _top=cfg.top;

    gpio_set_function(pinLF, GPIO_FUNC_PWM);
    gpio_set_function(pinRF, GPIO_FUNC_PWM);
    gpio_set_function(pinLR, GPIO_FUNC_PWM);
    gpio_set_function(pinRR, GPIO_FUNC_PWM);

    uint sna = pwm_gpio_to_slice_num(pinLF); 
    uint cna = pwm_gpio_to_channel(pinLF);

    printf("pinLF=%d, sna=%d, cna=%d\n", pinLF, sna, cna);
    if(slice_num_initialed.find(sna)==slice_num_initialed.end())
    {
        pwm_init(sna, &cfg, false); //初始化但不运行。
        slice_num_initialed.insert(sna);
        printf("pwm init slice %d\n", sna);
    }


    pwm_duty_set_u16(ToU16(0), sna, cna);  //31414是最低油门量，62500最高
    
    _LF.burden=0.0f;
    _LF.channel_num=cna;
    _LF.pin_num=pinLF;
    _LF.slice_num=sna;

    //gpio_set_function(pinRF, GPIO_FUNC_PWM);
    uint snb = pwm_gpio_to_slice_num(pinRF); 
    uint cnb = pwm_gpio_to_channel(pinRF);
    printf("pinRF=%d, snb=%d, cnb=%d\n", pinRF, snb, cnb);

    if(slice_num_initialed.find(snb)==slice_num_initialed.end())
    {
        pwm_init(snb, &cfg, false); //初始化但不运行。
        slice_num_initialed.insert(snb);
        printf("pwm init slice %d\n", snb);
    }    
    

    pwm_duty_set_u16(ToU16(0), snb, cnb);  

    _RF.burden=0.0f;
    _RF.channel_num=cnb;
    _RF.pin_num=pinRF;
    _RF.slice_num=snb;

    //gpio_set_function(pinLR, GPIO_FUNC_PWM);
    uint snc = pwm_gpio_to_slice_num(pinLR); 
    uint cnc = pwm_gpio_to_channel(pinLR);
    printf("pinLR=%d, snc=%d, cnc=%d\n", pinLR, snc, cnc);

    if(slice_num_initialed.find(snc)==slice_num_initialed.end())
    {
        pwm_init(snc, &cfg, false); //初始化但不运行。
        slice_num_initialed.insert(snc);
        printf("pwm init slice %d\n", snc);
    } 

    pwm_duty_set_u16(ToU16(0), snc, cnc);  

    _LR.burden=0.0f;
    _LR.channel_num=cnc;
    _LR.pin_num=pinLR;
    _LR.slice_num=snc;

    //gpio_set_function(pinRR, GPIO_FUNC_PWM);
    uint snd = pwm_gpio_to_slice_num(pinRR); 
    uint cnd = pwm_gpio_to_channel(pinRR);

    printf("pinRR=%d, snd=%d, cnd=%d\n", pinRR, snd, cnd);

    if(slice_num_initialed.find(snd)==slice_num_initialed.end())
    {
        pwm_init(snd, &cfg, false); //初始化但不运行。
        slice_num_initialed.insert(snd);
        printf("pwm init slice %d\n", snd);
    }

    

    pwm_duty_set_u16(ToU16(0), snd, cnd);

    _RR.burden=0.0f;
    _RR.channel_num=cnd;
    _RR.pin_num=pinRR;
    _RR.slice_num=snd;

    //设置完成后统一激活所有的slice.
    for(auto it=slice_num_initialed.begin(); it!=slice_num_initialed.end(); it++)
    {
        printf("enable slice %d\n", *it);
        pwm_set_enabled(*it, true);
        sleep_ms(200);
    }

    return true;

    #endif

    #if defined PROTOCOL_DSHOT
    bool b0=_LF.init(pinLF, pio0);
    bool b1=_LR.init(pinLR, pio0);
    bool b2=_RF.init(pinRF, pio0);
    bool b3=_RR.init(pinRR, pio0);
    if(!b0 ||!b1 ||!b2 ||!b3) {
        printf("motor init failed.\n");
        return false;
    }else{
        printf("motor init ok\n");
    }

    _LF.BeepTest();
    sleep_ms(1000);
    _LR.BeepTest();
    sleep_ms(1000);
    _RF.BeepTest();
    sleep_ms(1000);
    _RR.BeepTest();
    sleep_ms(1000);

    _LF.setThrottle(0);
    _LR.setThrottle(0);
    _RF.setThrottle(0);
    _RR.setThrottle(0);

    return true;
    #endif
}

//初始化电机，并设定初始值，主要用来校准电调。
void CMotor::Initialize(int pinLF, int pinRF, int pinLR, int pinRR, float duty)
{
    #if defined PROTOCOL_PWM
    //记录已经初始化的slice, 避免重复初始化。
    std::set<int> slice_num_initialed;

    pwm_config cfg=pwm_get_default_config();
    pwm_freq_set(cfg, 500); //在pico下这个频率不会失败

    _div=cfg.div;
    _top=cfg.top;

    gpio_set_function(pinLF, GPIO_FUNC_PWM);
    gpio_set_function(pinRF, GPIO_FUNC_PWM);
    gpio_set_function(pinLR, GPIO_FUNC_PWM);
    gpio_set_function(pinRR, GPIO_FUNC_PWM);

    uint sna = pwm_gpio_to_slice_num(pinLF); 
    uint cna = pwm_gpio_to_channel(pinLF);

    printf("pinLF=%d, sna=%d, cna=%d\n", pinLF, sna, cna);
    if(slice_num_initialed.find(sna)==slice_num_initialed.end())
    {
        pwm_init(sna, &cfg, false); //初始化但不运行。
        slice_num_initialed.insert(sna);
        printf("pwm init slice %d\n", sna);
    }


    pwm_duty_set_u16(ToU16(duty), sna, cna);  //31414是最低油门量，62500最高
    
    _LF.burden=duty;
    _LF.channel_num=cna;
    _LF.pin_num=pinLF;
    _LF.slice_num=sna;

    //gpio_set_function(pinRF, GPIO_FUNC_PWM);
    uint snb = pwm_gpio_to_slice_num(pinRF); 
    uint cnb = pwm_gpio_to_channel(pinRF);
    printf("pinRF=%d, snb=%d, cnb=%d\n", pinRF, snb, cnb);

    if(slice_num_initialed.find(snb)==slice_num_initialed.end())
    {
        pwm_init(snb, &cfg, false); //初始化但不运行。
        slice_num_initialed.insert(snb);
        printf("pwm init slice %d\n", snb);
    }    
    

    pwm_duty_set_u16(ToU16(duty), snb, cnb);  

    _RF.burden=duty;
    _RF.channel_num=cnb;
    _RF.pin_num=pinRF;
    _RF.slice_num=snb;

    //gpio_set_function(pinLR, GPIO_FUNC_PWM);
    uint snc = pwm_gpio_to_slice_num(pinLR); 
    uint cnc = pwm_gpio_to_channel(pinLR);
    printf("pinLR=%d, snc=%d, cnc=%d\n", pinLR, snc, cnc);

    if(slice_num_initialed.find(snc)==slice_num_initialed.end())
    {
        pwm_init(snc, &cfg, false); //初始化但不运行。
        slice_num_initialed.insert(snc);
        printf("pwm init slice %d\n", snc);
    } 

    pwm_duty_set_u16(ToU16(duty), snc, cnc);  

    _LR.burden=duty;
    _LR.channel_num=cnc;
    _LR.pin_num=pinLR;
    _LR.slice_num=snc;

    //gpio_set_function(pinRR, GPIO_FUNC_PWM);
    uint snd = pwm_gpio_to_slice_num(pinRR); 
    uint cnd = pwm_gpio_to_channel(pinRR);

    printf("pinRR=%d, snd=%d, cnd=%d\n", pinRR, snd, cnd);

    if(slice_num_initialed.find(snd)==slice_num_initialed.end())
    {
        pwm_init(snd, &cfg, false); //初始化但不运行。
        slice_num_initialed.insert(snd);
        printf("pwm init slice %d\n", snd);
    }

    

    pwm_duty_set_u16(ToU16(duty), snd, cnd);

    _RR.burden=duty;
    _RR.channel_num=cnd;
    _RR.pin_num=pinRR;
    _RR.slice_num=snd;

    //设置完成后统一激活所有的slice.
    for(auto it=slice_num_initialed.begin(); it!=slice_num_initialed.end(); it++)
    {
        printf("enable slice %d\n", *it);
        pwm_set_enabled(*it, true);
        sleep_ms(200);
    }
    #endif

    #if defined PROTOCOL_DSHOT
    _LF.init(pinLF);
    _LR.init(pinLR);
    _RF.init(pinRF);
    _RR.init(pinRR);
    _LF.setThrottle(duty);
    _LR.setThrottle(duty);
    _RF.setThrottle(duty);
    _RR.setThrottle(duty);
    #endif
}


void CMotor::GetBurdens(float burden[4])
{
    #if defined PROTOCOL_PWM
    burden[0]=_LF.burden;
    burden[1]=_RF.burden;
    burden[2]=_LR.burden;
    burden[3]=_RR.burden;
    #endif

    #if defined PROTOCOL_DSHOT
    burden[0]=_LF.GetThrottle();
    burden[1]=_RF.GetThrottle();
    burden[2]=_LR.GetThrottle();
    burden[3]=_RR.GetThrottle();
    #endif
}

float CMotor::GetSumBurden()
{
    #if defined PROTOCOL_PWM
    return _LF.burden+_RF.burden+_LR.burden+_RR.burden;
    #endif
    #if defined PROTOCOL_DSHOT
    return _LF.GetThrottle()+_RF.GetThrottle()+_LR.GetThrottle()+_RR.GetThrottle();
    #endif
}

//这个函数pwm波的占空比是均匀调节的，但电机的发力功耗不是均匀的线性的
//20240130，增加垂直推力分量参数
void CMotor::SetBurdens(float burden[4], float thrust)
{
    SetBurdens(burden[0], burden[1], burden[2], burden[3], thrust);
}

//20240130，增加垂直推力分量参数
void CMotor::SetBurdens(float b0, float b1, float b2, float b3, float thrust)
{
#if defined PROTOCOL_PWM
   _LF.burden=b0;
    pwm_duty_set_u16(ToU16(_LF.burden), _LF.slice_num, _LF.channel_num);
    _RF.burden=b1;
    pwm_duty_set_u16(ToU16(_RF.burden), _RF.slice_num, _RF.channel_num);  

    _LR.burden=b2;
    pwm_duty_set_u16(ToU16(_LR.burden), _LR.slice_num, _LR.channel_num);
    _RR.burden=b3;
    pwm_duty_set_u16(ToU16(_RR.burden), _RR.slice_num, _RR.channel_num);  
#endif

#if defined PROTOCOL_DSHOT
#if defined USE_MOTOR_FILTER
    float fb0=_LF_Filter.Filter(b0);
    float fb1=_RF_Filter.Filter(b1);
    float fb2=_LR_Filter.Filter(b2);
    float fb3=_RR_Filter.Filter(b3);
    _LF.setThrottle(fb0);
    _RF.setThrottle(fb1);
    _LR.setThrottle(fb2);
    _RR.setThrottle(fb3);
    mortor_burden record;
    record._burden[0]= fb0;
    record._burden[1]= fb1;
    record._burden[2]= fb2;
    record._burden[3]= fb3;
    record.tmark= get_time_mark();
    record._sum=fb0+fb1+fb2+fb3;
    record._thrust=thrust;
    _burden_queue.push(record);
#else
    _LF.setThrottle(b0);
    _RF.setThrottle(b1);
    _LR.setThrottle(b2);
    _RR.setThrottle(b3);
    mortor_burden record;
    record._burden[0]= b0;
    record._burden[1]= b1;
    record._burden[2]= b2;
    record._burden[3]= b3;
    record.tmark= get_time_mark();
    record._sum=b0+b1+b2+b3;
    record._thrust=thrust;
    _burden_queue.push(record);
#endif
#endif

}

//前推hist个数据，采样num个数据的平均值。
//20240130修改后不再使用这个函数。
void CMotor::GetHistoryMeanBurden(size_t hist, size_t num, float burden[4])
{
    mortor_burden b = _burden_queue.history_mean(hist, num);
    for(size_t i=0;i<4;i++) burden[i]=b._burden[i];
}

//特化处理的平衡推力参考值，取最后0.1秒-0.3秒的均值。
//20240130+
void CMotor::GetProperReferBurdenForBalance(float burden[4])
{
    uint32_t now=get_time_mark();

    uint32_t far_tmark= now-300; //300毫秒在系统记录范围，不必担心溢出。
    uint32_t near_tmark= now-100;

    for(size_t i=0;i<4;i++) burden[i]=0;

    int cnt=0;
    for(size_t i=0;i<160;i++) //目前有160个记录，2毫秒一个，可以覆盖最远320毫秒的数据。够用。
    {
        if(_burden_queue.last(i).tmark > near_tmark) continue;
        if(_burden_queue.last(i).tmark < far_tmark) break;
        burden[0]+= _burden_queue.last(i)._burden[0];
        burden[1]+= _burden_queue.last(i)._burden[1];
        burden[2]+= _burden_queue.last(i)._burden[2];
        burden[3]+= _burden_queue.last(i)._burden[3];
        cnt++;
    }

    if(cnt) {
        //如果当前时间和记录时间错位很大，得不到数据，cnt=0，所以这里防范。
        //比如系统停机后再启动，数据记录就超时了，所以参考推力会返回0.
        for(size_t i=0;i<4;i++) burden[i]/=cnt;
    }
}

//获得垂直推力参考值
//之前的垂直推力参考值是最后的80-120毫秒左右，没有跳过末尾段。感觉上应该跳过末尾100毫秒，
//因为控高在离开了imu的加速度辅助后，明显上下震荡严重，说明完全靠传感器调节存在不同步情况。
//过于依赖imu的加速度去稳定总是存在隐患，在机身震动时，就非常危险。
//20240130+
//20240130，已测试新的控高路径没有问题。下面争取校准推力参考延迟和跨度。目前是无延迟，跨度100毫秒。
//先尝试降低imu成分，这会导致高度上下波动，然后再找一个延迟和跨度，使得这种波动变小。
//但这样匹配的是tof模块的延迟，可能和气压延迟还是有一定的区别。
float CMotor::GetProperReferThrustForHeightControl()
{
    uint32_t now=get_time_mark();

    //暂时保留原始时间段，取最后100毫秒的推力均值。

    //前推10毫秒似乎不好，力度抖动大。似乎前推30毫秒较平稳。这里主要以气压控高来评估。
    uint32_t far_tmark= now-130;
    uint32_t near_tmark= now-30;

    int cnt=0;
    float sumt=0;
    for(size_t i=0;i<160;i++) //目前有160个记录，2毫秒一个，可以覆盖最远320毫秒的数据。够用。
    {
        if(_burden_queue.last(i).tmark > near_tmark) continue;
        if(_burden_queue.last(i).tmark < far_tmark) break;
        sumt+= _burden_queue.last(i)._thrust;
        cnt++;
    }

    if(cnt) {
        //如果当前时间和记录时间错位很大，得不到数据，cnt=0，所以这里防范。
        //比如系统停机后再启动，数据记录就超时了，所以参考推力会返回0.
        sumt/=cnt;
    }

    return sumt;
}

//只能在电机不工作时调用发声。
void CMotor::Beep()
{
    #if defined PROTOCOL_PWM
    return;
    #endif
    #if defined PROTOCOL_DSHOT
    _LF.BeepTest();
    _RF.BeepTest();
    _LR.BeepTest();
    _RR.BeepTest();
    #endif
}