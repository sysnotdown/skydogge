
#include "micro_task_exec.h"
#include <math.h>
#include "press.h"
#include "CTimeDif.h"

#include "gps_ublox.h"
#include "skydogge.h"
#include <algorithm> //for debug
#include <random>
std::default_random_engine rgen2;
std::uniform_int_distribution<int> gapgen(0, 1000);
using namespace std;

//std::vector<float> time_debug_queue; //for debug

extern CMotor motor_ctrl;
extern _micro_task micro_task;
extern NRandQueue<imu_info> imu_queue;
extern NRandQueue<tof_oflow_info> tof_oflow_queue;
extern NRandQueue<press_info> press_queue;
extern NRandQueue<float> current_queue;
extern critical_section_t press_queue_section;
extern critical_section_t imu_queue_section;
extern critical_section_t tof_oflow_queue_section;

#if defined USE_GPS
extern NRandQueue<gps_info> gps_queue;
extern critical_section_t gps_queue_section;
extern gps_info initial_gps_info;
#endif


extern tof_oflow_info initial_tof_oflow_info; //初始tof信息, 参考这个用来降落
extern press_info initial_press_info;

extern float system_power_current; //电流消耗，用于微控制参考。pin_to_location里使用
extern float system_power_voltage; //电压。
//全局共享空中标记
extern bool _inair;

extern bool _intof; //高度控制是否处于测距模式下。

//指向正北的yaw角度。
extern float _heading_yaw;
//extern float _heading_reliable; //20240405+
#if defined USE_QMC5883 || defined USE_HMC5883
extern uint32_t _heading_yaw_last_update_time;
#endif
//20231205+ 在做控高代码优化时，容易出现意外，这个标记强制使用老的稳定代码。
extern bool _force_stable_baro_height_control;
extern initial_heading_method _initial_heading_update_method;

//发送遥控消息，用来调试，一般微任务不发送信息给遥控
extern void Send_remote_message(std::string msg);
extern void Send_remote_message(uint8_t msgid);
extern float heading_angle(float x, float y);
extern float Compass_Heading();
extern bool Compass_Heading(float& ch, float& cur_yaw);
extern float air_press_height_relation(float gps_height);
extern float height_by_air_press(float press);
extern float height_dif_by_air_press(float press_a, float press_b);
extern float air_press_height_relation(float gps_height);

extern uint8_t power_control_solution; 
extern float _baro_height;

extern float balance_main_ratio; //平衡函数主参数，现在写入配置表可配置，20250324
extern float balance_p_ratio; //BALP
extern float balance_i_ratio; //BALI
extern float balance_d_ratio; //BALD
extern float balance_x_ratio; //BALX
extern float balance_w_ratio; //BALW
extern float balance_z_ratio; //BALZ
//20250110+ 由控高函数灌入，平飞函数参考使用，用来避免角度死锁。
//float height_bias=0; //真实高差，单位米，平飞时需关注这个高差，出现异常时可能是陷入了角度死锁，此时需要减小倾角。

//动态的气压高度比例系数，海拔越高这个值越小，影响垂直方向的速度控制。默认值是地面值。每米高度的气压变化值。
float dynamic_air_press_height_ratio=11.719;

//似乎50个数据即可，最远0.5秒，当前在tof的2次调节里引用了30个数据。
//20231130,增加记录到155个，以便气压降落时判断准确
//20240305,现在降落集成在控高函数里，不需要调用landingcheck这个函数去检查了，所以不需要存储太多高度调节信息。降低为50个，大约节省了1.9k运行内存
//tof控高的2次调节里最远引用了30个数据。
__not_in_flash("data") NRandQueue<height_adjust_record> _height_adjust_queue(50);

//光流不怎么用了，暂时关掉
#if defined USE_OFLOW
//为求oflow的加速度引用了30个历史记录。
__not_in_flash("data") NRandQueue<oflow_bias> oflow_bias_queue(40); //用来保存oflow调节偏角历史
#endif

//修改存储长度，目前不用太长的记录。之前是80，现在设置为5以节省内存。大约节省了3.9k运行内存
//20240706, 记录修改为15长度，因需要分析加速度变化。
__not_in_flash("data") NRandQueue<gps_major_bias> gps_major_bias_queue(15); //用来保存gps调节偏角历史


#define isNAN(x) ((x)!=(x))

#define INF (std::numeric_limits<float>::infinity()) //无穷大
#define PINF INF //正无穷大
#define NINF -INF //负无穷大
#define isINF(x) (((x)==PINF)||((x)==NINF))
#define isPINF(x) ((x)==PINF)
#define isNINF(x) ((x)==NINF)

#define FLOAT_LIMIT(a, low, high) { if(a<low) a=low; if(a>high) a=high;}

//20221219实验表明，后方电机产生的风力确实影响气压值。
//油门量，气压降低【pa】
//0.2  3
//0.3  4
//0.4  5
//0.5  7.6
//0.6  估计9-10，没有测试。

//20221220, 把气压计装入密封盒子内测试。
//油门量，气压降低【pa】
//0.2  -0.796
//0.3  3.6
//0.4  4.4
//0.5  4.8
//0.6  5.4 //估计
//0.7  6.2 //估计
//0.8  7.2 //估计
//0.9  8.4 //估计

//可见密封有一定的效果。但不能消除气压降低。

//如果能根据软件去调节测量的气压值，则不需要做物理调节，否则要在气压测量点位去做改变。

//安装方法是，logo朝上放置，正面字体，
//则机头朝向Y方向。机身右侧朝向X方向，Z方向垂直地面指向上方
//则：
//俯仰角是第一个数，滚转角是第二个数，航向角是第三个。
//俯仰角<0,机头向下，>0,机头向上
//滚转角<0,机身左低右高，>0,机身左高右低。
//航向角是-180-180度，-180和180之间是突变点。左旋角度增加，右旋角度减小。增加超过180时突变为-180，减小超过-180时突变为180.

//第一个角速度>0, 代表正在抬头，<0, 代表低头
//第二个角速度>0, 代表右侧变低左侧变高的滚转，<0,相反
//第三个角速度是航向角的变化率。增大左旋(逆时针)，减小右旋（顺时针）


//机器全重可能在2850克附近。
//12/4测试过，上升速度太快了。可能在第二阶段发力增加太快。
//t.press需要设置起飞感应高度，到达这个高度就算成功。

//现在高度调节统一了，不使用这个来起飞。
//直接标记micro_task.help_takeoff, 标记后统一用height_adjust,那里有起飞的高度控制码。
//起飞时仅使用光流来定点，因为位置较低。

//杂牌 2308 kv1100 9寸2叶，悬停油门1.3，电流小，4.3安左右，电机不热。功耗65瓦左右。悬停45分钟。
//tmotor 2207v3 kv1750，9寸2叶桨，悬停油门0.8，电流大，7-8安，电机发热。效率极低，怀疑桨叶的问题。
//tmotor 2207v3 kv1750, 7040三叶桨，悬停油门1.1，电流6安左右，电机不热。
//tmotor 2027v3 kv1750, 8040三叶乾丰桨，悬停油门1.0左右，电流5.7左右，电机温热，效率比7寸3叶要高。全重825克左右，悬停估计36分钟左右。
//这个悬停功耗约82瓦，按36分钟算，能积分到17.7万瓦输出能量。力效约10克/瓦。电流如果再大，电池发热会损耗更多能量，则积分出来会更低。
//桨越大，悬停油门越低，电机发热越高。主要是电机kv值偏大了。

float CMicroTaskExec::SigmoidTrans(float a)
{
    return 2.0/(1+exp(-a)) -1.0;
}


//20240109+
float CMicroTaskExec::GpsQuality(uint8_t ns, float hacc)
{
    if(ns >= 24)
    {
        if(hacc < 0.4)
        {
            return 1.0;
        }
        else if(hacc < 0.6)
        {
            return 0.95;
        }
        else if(hacc < 0.8)
        {
            return 0.85;
        }
        else if(hacc < 1.0)
        {
            return 0.75;
        }
        else
        {
            return 0.7;
        }
    }
    else if(ns >= 20)
    {
        if(hacc < 0.4)
        {
            return 0.95;
        }
        else if(hacc < 0.6)
        {
            return 0.92;
        }
        else if(hacc < 0.8)
        {
            return 0.82;
        }
        else if(hacc < 1.0)
        {
            return 0.72;
        }
        else
        {
            return 0.65;
        }
    }
    else if(ns >= 16)
    {
        if(hacc < 0.4)
        {
            return 0.90;
        }
        else if(hacc < 0.6)
        {
            return 0.85;
        }
        else if(hacc < 0.8)
        {
            return 0.75;
        }
        else if(hacc < 1.0)
        {
            return 0.65;
        }
        else
        {
            return 0.55;
        }
    }
    else if(ns >= 12)
    {
        if(hacc < 0.4)
        {
            return 0.85;
        }
        else if(hacc < 0.6)
        {
            return 0.75;
        }
        else if(hacc < 0.8)
        {
            return 0.65;
        }
        else if(hacc < 1.0)
        {
            return 0.55;
        }
        else
        {
            return 0.45;
        }
    }
    else if(ns >= 8)
    {
        if(hacc < 0.4)
        {
            return 0.8;
        }
        else if(hacc < 0.6)
        {
            return 0.7;
        }
        else if(hacc < 0.8)
        {
            return 0.6;
        }
        else if(hacc < 1.0)
        {
            return 0.45;
        }
        else
        {
            return 0.40;
        }
    }
    else
    {
        return 0.4;
    }

}


void CMicroTaskExec::micro_task_ground_up()
{
    if(micro_task.status==_micro_task_status_none)
    {

        motor_ctrl.SetBurdens(0,0,0,0,0);
        micro_task.roll=0;
        micro_task.pitch=0;
        //micro_task.vspeed= -6; //向上的速度。之前的速度反了。-6表示向上0.6m/s的速度。
        micro_task.vspeed= 0; //直接让高差去调节。现在如果设定了速度则高差不调节。
        micro_task.gps_anchor=false; //需要注销锚定以避免上次锚定的影响。
        micro_task.status=_micro_task_status_inprogress;
        micro_task.timemark=get_time_mark();
        micro_task.help_takeoff=true;
        //gps_major_bias_queue.last().gps_tmark=0; //清除调节序列，使其不连续使用。
        for(size_t i=0;i<10;i++) {
            gps_major_bias_queue.last(i).clear(); //清除调节序列，避免上次起飞残留调节。
        }
        //macro设置，不必修改。
        //micro_task.tof_height = (micro_task.init_press - micro_task.press)/12.0; //这里错误写为*12.0，20230304改正
        // char dbg[128];
        // sprintf(dbg, "first in groundup, press=%f, tof_height=%f\n", micro_task.press, micro_task.tof_height);
        // logmessage(dbg);
    }

    if(micro_task.status==_micro_task_status_done) {
        
        //状态已经修改为完成，但上层可能还没发现，维持t.press指示的高度即可。
        //在宏任务发现起飞成功前，这里有一个短暂的代管期。也许会因为惯性而继续上升。记录一下情况。
        bool landed;
        height_adjust_with_landing_detect(INAIR_BOTTHRUST, INAIR_TOPTHRUST, landed);

        float exec_pitch;
        float exec_roll;
        
    #if defined USE_OFLOW
        oflow_position_adjust_minor_change();
        float pb= oflow_bias_queue.last().pitch_bias;
        float rb= oflow_bias_queue.last().roll_bias;    
    #elif defined USE_GPS
        gps_position_adjust();
        float pb= gps_major_bias_queue.last().pitch_bias;
        float rb= gps_major_bias_queue.last().roll_bias;
    #endif

        if(fabs(micro_task.pitch)<0.01 && fabs(micro_task.roll)<0.01)
        {
            exec_pitch=pb;
            exec_roll=rb;
        }
        else if(fabs(micro_task.pitch)<0.01)
        {
            exec_pitch=pb;
            exec_roll= micro_task.roll;
        }
        else if(fabs(micro_task.roll)<0.01)
        {
            exec_pitch=micro_task.pitch;
            exec_roll= rb; 
        }
        else
        {
            exec_pitch=micro_task.pitch;
            exec_roll= micro_task.roll;
        }

        
        float new_burdens[4];
        float new_thrust;
        balance(new_burdens, new_thrust, exec_pitch, exec_roll);

        motor_ctrl.SetBurdens(new_burdens, new_thrust);

        return;
    }
    else if(micro_task.status==_micro_task_status_fail)
    {
        logmessage("takeoff failed\n");
        return;
    }

#if defined USE_TOF
    //起飞成功检查。原来以气压为判断基准。现在修改为先使用tof.
    tof_oflow_info toflast;
    critical_section_enter_blocking(&tof_oflow_queue_section);
    toflast= tof_oflow_queue.last();
    critical_section_exit(&tof_oflow_queue_section);

    if(toflast.tof_valid)
    {
        //if(toflast.fixed_tof_distance + 0.1 >= micro_task.tof_height) //仅适合老版本控高，不带降落检测的，不带刹车的。
        if(toflast.fixed_tof_distance + 0.1 >= micro_task.help_takeoff_target_height) //20240309修正
        {
            micro_task.status=_micro_task_status_done;
            micro_task.vspeed=0;
            _inair=true;
            
            // char buf[128];
            // sprintf(buf, "gndup ok by tof, tof=%5.2f, target height=%f\n",  toflast.fixed_tof_distance, micro_task.tof_height); 
            // logmessage(buf);
            return;
        }

    }
    else if(!toflast.tof_valid)
    {
        //tof不可用时，使用气压判断。

        press_info presslast;
        critical_section_enter_blocking(&press_queue_section);
        presslast= press_queue.history_mean(0,5);
        critical_section_exit(&press_queue_section);

        //if(presslast.press <= micro_task.press + 0.2)
        if(presslast.press <= micro_task.help_takeoff_target_press + 0.2) //20240309修正
        {
            micro_task.status=_micro_task_status_done;
            micro_task.vspeed=0;
            _inair=true;

            // char buf[128];
            // sprintf(buf, "gndup ok by press,  press=%f, target press=%f\n",  presslast.press, micro_task.press); 
            // logmessage(buf);

            return;
        }
    }
 #else
        press_info presslast;
        critical_section_enter_blocking(&press_queue_section);
        presslast= press_queue.history_mean(0,3);
        critical_section_exit(&press_queue_section);

        if(presslast.press <= micro_task.press + 0.2)
        {
            micro_task.status=_micro_task_status_done;
            micro_task.vspeed=0;
            _inair=true;

            char buf[128];
            sprintf(buf, "gndup ok by press,  press=%f, target press=%f\n",  presslast.press, micro_task.press); 
            logmessage(buf);

            return;
        }
 #endif

    //以下都是_micro_task_status_inprogress状态。
    bool landed;

    height_adjust_with_landing_detect(INAIR_BOTTHRUST, INAIR_TOPTHRUST, landed);

    float exec_pitch=0;
    float exec_roll=0;
    
#if defined USE_OFLOW
    if(oflow_position_adjust_minor_change())
    {
        exec_pitch= oflow_bias_queue.last().pitch_bias;
        exec_roll=oflow_bias_queue.last().roll_bias;
    }
#elif defined USE_GPS
    if(gps_position_adjust())
    {
        exec_pitch= gps_major_bias_queue.last().pitch_bias;
        exec_roll= gps_major_bias_queue.last().roll_bias;

        //起飞时可能有GPS数据波动，导致倾角变化，但不应该太大。
        //但如果限制的过小，则大风中起飞时，会被吹跑很远。
        //FLOAT_LIMIT(exec_pitch, -2.0, 2.0);
        //FLOAT_LIMIT(exec_roll, -2.0, 2.0);

        //新限制整体侧倾16度以内。20250114 修改。
        float want_fx= tan(exec_roll/57.2958);
        float want_fy= -tan(exec_pitch/57.2958);
        float want_fmod= sqrt(want_fx*want_fx+want_fy*want_fy);
        if(want_fmod>0.2867) { //tan(16)=0.2867
            exec_pitch*= 0.2867/want_fmod;
            exec_roll*= 0.2867/want_fmod;
        }

    }
#endif

    float new_burdens[4];
    float new_thrust;
    balance(new_burdens, new_thrust, exec_pitch, exec_roll);

    motor_ctrl.SetBurdens(new_burdens, new_thrust);

    uint32_t now = get_time_mark();

    //下面是检查起飞失败代码。
    if(now - micro_task.timemark > 30*1000)
    {
        if(_inair) {
            micro_task.status=_micro_task_status_done;
            micro_task.vspeed=0;
            return;
        }
        else
        {
            motor_ctrl.SetBurdens(0,0,0,0, 0);
            micro_task.status=_micro_task_status_fail;

            char buf[128];
            sprintf(buf, "gndup fail, time runs out\n");
            logmessage(buf); 
        }

        //Send_remote_message("gndup fail");
    }

}

// //原版，仅返回加速度。已不再使用
// float CMicroTaskExec::get_refer_imu_vacc_for_height_control()
// {
//     float sumval=0; //过去若干个imu加速度均值。
//     float maxval;
//     float minval;
//     float meanval=0;
//     uint16_t above1g=0; //高于1g的加速度数量。好的机身这个数据为0
//     uint16_t abovemean=0; //高于均值的数量。好的数据在一半，上下偏差5个左右。
//     float meanviarant=0; //好的数据在0.2以内，大的也不会高于0.2太多

//     critical_section_enter_blocking(&imu_queue_section);
//     maxval= imu_queue.last().acc_gnd[2];
//     minval= imu_queue.last().acc_gnd[2];
//     for(size_t i=0;i<50;i++) {
//         float v= imu_queue.last(i).acc_gnd[2];
//         sumval+= v;
//         maxval=(std::max)(maxval, v);
//         minval=(std::min)(minval, v);
//         if(fabs(v)>1.0) above1g++;
//     }
//     meanval=sumval/50.0; //平均值

//     for(size_t i=0;i<50;i++) {
//         float v= imu_queue.last(i).acc_gnd[2];
//         if(v>=meanval) abovemean++;
//         meanviarant += fabs(v-meanval);
//     }
//     meanviarant/=50.0; //波动性。
//     critical_section_exit(&imu_queue_section);

//     //依据平均值，波动性，高于平均的数量，以及超过1g的数量来计算其可靠性，是否抖动强烈。
//     //首先高于平均值数量和低于平均值数量最好各半，偏离太大的可靠性打折。
//     float r0,r1,r2;

//     //最佳数为25.
//     if(abovemean>=21 && abovemean <=29) {
//         r0=1.0;
//     }else if(abovemean>=17 && abovemean <=33) {
//         r0=0.8;
//     }else if(abovemean>=13 && abovemean <=37) {
//         r0=0.4;
//     }else if(abovemean>=9 && abovemean <=41) {
//         r0=0.2;
//     }else if(abovemean>=5 && abovemean <=45) {
//         r0=0.1;
//     }else if(abovemean>=1 && abovemean <=49) {
//         r0=0.05;
//     }else{
//         r0=0.025;
//     }

    

//     //差机身above1g量很大。适度放宽。
//     if(above1g<5)
//     {
//         r1=1.0;
//     }else if(above1g<10) {
//         r1=0.85;
//     }else if(above1g<15) {
//         r1=0.7;
//     }else if(above1g<20) {
//         r1=0.5;
//     }else if(above1g<25) {
//         r1=0.3;
//     }else if(above1g<30) {
//         r1=0.2;
//     }else if(above1g<35) {
//         r1=0.1;
//     }else if(above1g<40) {
//         r1=0.05;
//     }else{
//         r1=0.025;
//     }

//     //meanviarant 静态0.001，摇摆0.03，上下快速来回 0.2
//     //差的机器震动大，平均在1.0左右。震动小的机器平均0.2左右。
//     if(meanviarant<0.15) {
//         r2=1.0;
//     }else if(meanviarant<0.2){
//         r2=0.95;
//     }else if(meanviarant<0.3){
//         r2=0.9;
//     }else if(meanviarant<0.4){
//         r2=0.8;
//     }else if(meanviarant<0.5){
//         r2=0.7;
//     }else if(meanviarant<0.6){
//         r2=0.6;
//     }else if(meanviarant<0.7){
//         r2=0.5;
//     }else if(meanviarant<0.8){
//         r2=0.4;
//     }else if(meanviarant<0.9){
//         r2=0.3;
//     }else if(meanviarant<1.0){
//         r2=0.2;
//     }
//     else{
//         r2=0.1;
//     }

//     float r3,r4;
//     if(maxval - meanval > 1.9)
//     {
//         r3=0.1;
//     }
//     else if(maxval - meanval > 1.7)
//     {
//         r3=0.3;
//     }
//     else if(maxval - meanval > 1.5)
//     {
//         r3=0.4;
//     }
//     else if(maxval - meanval > 1.3)
//     {
//         r3=0.5;
//     }
//     else if(maxval - meanval > 1.1)
//     {
//         r3=0.6;
//     }
//     else if(maxval - meanval > 0.9)
//     {
//         r3=0.7;
//     }
//     else if(maxval - meanval > 0.7)
//     {
//         r3=0.8;
//     }
//     else if(maxval - meanval > 0.5)
//     {
//         r3=0.9;
//     }
//     else
//     {
//         r3=1.0;
//     }

//     if( meanval - minval > 1.9)
//     {
//         r4=0.1;
//     }
//     else if(maxval - meanval > 1.7)
//     {
//         r4=0.3;
//     }
//     else if(maxval - meanval > 1.5)
//     {
//         r4=0.4;
//     }
//     else if(maxval - meanval > 1.3)
//     {
//         r4=0.5;
//     }
//     else if(maxval - meanval > 1.1)
//     {
//         r4=0.6;
//     }
//     else if(maxval - meanval > 0.9)
//     {
//         r4=0.7;
//     }
//     else if(maxval - meanval > 0.7)
//     {
//         r4=0.8;
//     }
//     else if(maxval - meanval > 0.5)
//     {
//         r4=0.9;
//     }
//     else
//     {
//         r4=1.0;
//     }

//     float r5=sqrt(r3*r4);

    
//     return r0*r1*r2*r5*sumval/-5.0; //反向，并转标准单位m/s2, 向下为正
// }

//返回垂直方向的Imu加速度，识别震动，过滤震动。返回标准加速度单位，下为正。
//tof/baro共用。主要是用作阻尼。
//扩展返回数据，震动性数据和侧倾角数据。以便气压控高可以判断降落条件。20240228+
float CMicroTaskExec::get_refer_imu_vacc_for_height_control(float& zaxis, float& vib_z)
{
    float sumval=0; //过去若干个imu加速度均值。
    float maxval;
    float minval;
    float meanval=0;
    uint16_t above1g=0; //高于1g的加速度数量。好的机身这个数据为0
    uint16_t abovemean=0; //高于均值的数量。好的数据在一半，上下偏差5个左右。
    float meanviarant=0; //好的数据在0.2以内，大的也不会高于0.2太多

    critical_section_enter_blocking(&imu_queue_section);
    maxval= imu_queue.last().acc_gnd[2];
    minval= imu_queue.last().acc_gnd[2];
    zaxis= imu_queue.last().zaxis; //多返回一个数据。
    for(size_t i=0;i<50;i++) {
        float v= imu_queue.last(i).acc_gnd[2];
        sumval+= v;
        maxval=(std::max)(maxval, v);
        minval=(std::min)(minval, v);
        if(fabs(v)>1.0) above1g++;
    }
    meanval=sumval/50.0; //平均值

    for(size_t i=0;i<50;i++) {
        float v= imu_queue.last(i).acc_gnd[2];
        if(v>=meanval) abovemean++;
        meanviarant += fabs(v-meanval);
    }
    meanviarant/=50.0; //波动性。
    critical_section_exit(&imu_queue_section);

    vib_z=meanviarant;

    //依据平均值，波动性，高于平均的数量，以及超过1g的数量来计算其可靠性，是否抖动强烈。
    //首先高于平均值数量和低于平均值数量最好各半，偏离太大的可靠性打折。
    float r0,r1,r2;

    //最佳数为25.
    if(abovemean>=21 && abovemean <=29) {
        r0=1.0;
    }else if(abovemean>=17 && abovemean <=33) {
        r0=0.8;
    }else if(abovemean>=13 && abovemean <=37) {
        r0=0.4;
    }else if(abovemean>=9 && abovemean <=41) {
        r0=0.2;
    }else if(abovemean>=5 && abovemean <=45) {
        r0=0.1;
    }else if(abovemean>=1 && abovemean <=49) {
        r0=0.05;
    }else{
        r0=0.025;
    }

 
    //差机身above1g量很大。适度放宽。
    //20250327 调小这个值，避免打杆下不来。
    if(above1g<5)
    {
        r1=1.0;
    }else if(above1g<10) {
        r1=0.6;
    }else if(above1g<15) {
        r1=0.4;
    }else if(above1g<20) {
        r1=0.2;
    }else if(above1g<25) {
        r1=0.1;
    }else if(above1g<30) {
        r1=0.05;
    }else if(above1g<35) {
        r1=0.03;
    }else if(above1g<40) {
        r1=0.02;
    }else{
        r1=0.01;
    }

    //meanviarant 静态0.001，摇摆0.03，上下快速来回 0.2
    //差的机器震动大，平均在1.0左右。震动小的机器平均0.2左右。
    //差的imu也导致meanviarant奇大，普通的在0.2-0.3正常范围。
    //20240229 稍放松这个限制，有些机架即使有强滤波，加速度噪音也大。
    //20250327 调小这个值，避免共振时下不来。
    if(meanviarant<0.15) {
        r2=1.0;
    }else if(meanviarant<0.2){
        r2=0.95;
    }else if(meanviarant<0.3){
        r2=0.9;
    }else if(meanviarant<0.4){
        r2=0.8;
    }else if(meanviarant<0.5){
        r2=0.7;
    }else if(meanviarant<0.6){
        r2=0.6;
    }else if(meanviarant<0.7){
        r2=0.5;
    }else if(meanviarant<0.8){
        r2=0.4;
    }else if(meanviarant<0.9){
        r2=0.3;
    }else if(meanviarant<1.0){
        r2=0.2;
    }
    else{
        r2=0.1;
    }

    float r3,r4;
    if(maxval - meanval > 1.9)
    {
        r3=0.1;
    }
    else if(maxval - meanval > 1.7)
    {
        r3=0.3;
    }
    else if(maxval - meanval > 1.5)
    {
        r3=0.4;
    }
    else if(maxval - meanval > 1.3)
    {
        r3=0.5;
    }
    else if(maxval - meanval > 1.1)
    {
        r3=0.6;
    }
    else if(maxval - meanval > 0.9)
    {
        r3=0.7;
    }
    else if(maxval - meanval > 0.7)
    {
        r3=0.8;
    }
    else if(maxval - meanval > 0.5)
    {
        r3=0.9;
    }
    else
    {
        r3=1.0;
    }

    if( meanval - minval > 1.9)
    {
        r4=0.1;
    }
    else if(maxval - meanval > 1.7)
    {
        r4=0.3;
    }
    else if(maxval - meanval > 1.5)
    {
        r4=0.4;
    }
    else if(maxval - meanval > 1.3)
    {
        r4=0.5;
    }
    else if(maxval - meanval > 1.1)
    {
        r4=0.6;
    }
    else if(maxval - meanval > 0.9)
    {
        r4=0.7;
    }
    else if(maxval - meanval > 0.7)
    {
        r4=0.8;
    }
    else if(maxval - meanval > 0.5)
    {
        r4=0.9;
    }
    else
    {
        r4=1.0;
    }

    float r5=sqrt(r3*r4);

    // printf("abovemean=%d, above1g=%d, meanviarant=%6.5f\n", abovemean, above1g, meanviarant);
    // printf("r0,r1,r2,r3,r4=%5.2f,%5.2f,%5.2f,%5.2f,%5.2f\n", r0, r1, r2,r3,r4);

    //两个不同机身的震动对比。
    //1.above1g的数量差异巨大，好的机身记录全是0，而差的机身是4-44不等，几乎覆盖全部的数据。
    //不过差机身在多数情况下，平均值上下数据量差别不大，偏离不明显。
    //2.差机身meanviarant特别大。好的机身平均在0.2左右，差机身平均在1.0左右，大的超过2.6，小的也在0.39以上
    //3.差机身数据差异可能和imu安装位置不居中有关。目前靠近右前臂，离中央较远。
    //4.above1g的数量对加速度返回值影响大，衰减大。可以考虑适度放宽。否则返回值是非常小的。
    //5. minval,maxval偏离均值的量对于高震动机身影响也很大，r5常在0.1，小幅放开宽容度。

    // if(gapgen(rgen2)>950) //0-1000随机数。
    // {
    //     //部分记录
    //     char dbg[128];
    //     //sprintf(dbg, "%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%4.3f,%d,%d\n", sumval/-5.0, r0, r1, r2, r5, meanviarant,abovemean, above1g);
    //     sprintf(dbg, "%4.3f,%4.3f\n", meanviarant, sumval/-5.0);
    //     logmessage(dbg);
    // }

    return r0*r1*r2*r5*sumval/-5.0; //反向，并转标准单位m/s2, 向下为正
}



bool CMicroTaskExec::hyper_height_adjust_by_baro_with_landing_detect(float botlmt, float toplmt, bool& landed)
{

    //默认使用测试版，在观察到气压差和设定值相差较大时，使用稳定版。
    if(_force_stable_baro_height_control) {
        return height_adjust_by_baro_with_landing_detect_stable(botlmt, toplmt, landed);
    }

    //在超过高度20米时，使用稳定版，且以后一直使用稳定版。
    float lp;
    critical_section_enter_blocking(&press_queue_section);
    lp=press_queue.last().press;
    critical_section_exit(&press_queue_section);

    if(lp < initial_press_info.press - 1400.0)  //120m
    {
        _force_stable_baro_height_control=true; //标记以后强制使用稳定版。
        //logmessage("mark stable height ctrl\n");
        return height_adjust_by_baro_with_landing_detect_stable(botlmt, toplmt, landed);
    }

    //经过上面的检查和限制后，这里使用测试版。
    return height_adjust_by_baro_with_landing_detect(botlmt, toplmt, landed); 
}


//需要传入50个信号间隔时间，微秒。
//这个还是返回balance的基点负载，否则balance无法平衡。
void CMicroTaskExec::get_refer_burdens_for_balance(float burdens[4])
{
    //20240130修改，这个固定取最后0.1-0.3秒的记录均值，不需要再访问imu数据。
    motor_ctrl.GetProperReferBurdenForBalance(burdens);

}

float CMicroTaskExec::get_refer_thrust_for_height_control()
{
    return motor_ctrl.GetProperReferThrustForHeightControl();
}

//设定降落速度。输入净高度，距离。返回>=0的数值，代表向下速度。
//20240217+ 辅助micro_task_vertical_landing_target_location，最大降落速度8米/秒，需要测试。
//20240228 修改。
//20240419 增加200m以上档位，最大速度10m/s，减少高空降落时间。放松最大距离到16米
int CMicroTaskExec::auto_landing_vertical_speed(float height, float dist)
{
    if(dist>16.0) return 0;
    if(height<20.0 && dist > 3.0) return 0;
    if(height<50.0 && dist > 6.0) return 0;
    if(height<100.0 && dist > 9.0) return 0;

    float r;
    float vspd;
    if(height>=300.0)
    {
        //高度>300 且距离<15的。
        if(dist<8.0) r=1.0; //8米以内全速降落
        else r= 8.0/dist; //dist最大是16.0，此时r=0.5
        vspd = height*r/15;
        FLOAT_LIMIT(vspd, 3.0, 12.0); //tmotor 2207v3 kv1750 8m/s的降速基本没有姿态问题，会有小幅晃动，低KV值电机可能反应会慢点，速度太快可能有风险。
    }
    else if(height>=200.0)
    {
        //高度>200 且距离<15的。
        if(dist<8.0) r=1.0; //8米以内全速降落
        else r= 8.0/dist; //dist最大是16.0，此时r=0.5
        vspd = height*r/15;
        FLOAT_LIMIT(vspd, 3.0, 9.0); //tmotor 2207v3 kv1750 8m/s的降速基本没有姿态问题，会有小幅晃动，低KV值电机可能反应会慢点，速度太快可能有风险。
    }
    else if(height>=100.0)
    {
        //高度>100 且距离<16的。
        if(dist<6.0) r=1.0; //6米以内全速降落
        else r= 6.0/dist; //dist最大是16.0，此时r=0.375
        vspd = height*r/15;
        FLOAT_LIMIT(vspd, 2.0, 5.0); //tmotor 2207v3 kv1750 8m/s的降速基本没有姿态问题，会有小幅晃动，低KV值电机可能反应会慢点，速度太快可能有风险。
    }
    else if(height>=70.0)
    {
        //高度（50，100） 且距离<9的。
        if(dist<3.0) r=1.0; //3米以内全速降落
        else r= 3.0/dist; //dist最大是9.0，此时r=1/3
        vspd = height*r/15;
        FLOAT_LIMIT(vspd, 1.0, 3.0);
    }
    else if(height>=50.0)
    {
        //高度（50，100） 且距离<9的。
        if(dist<3.0) r=1.0; //3米以内全速降落
        else r= 3.0/dist; //dist最大是9.0，此时r=1/3
        vspd = height*r/15;
        FLOAT_LIMIT(vspd, 1.0, 2.5);
    }
    else if(height>=25.0)
    {
        //20-50之间
        if(dist<1.6) r=1.0;
        else r= 1.6/dist; //dist最大是6.0，此时r=1/5
        vspd = height*r/15;
        FLOAT_LIMIT(vspd, 0.5, 2.0);
    }
    else 
    {
        //h<20.0
        if(dist<1.2) r=1.0;
        else r= 1.2/dist; //dist最大是3.0，此时r=0.266
        vspd = height*r/15;
        FLOAT_LIMIT(vspd, 0.5, 1.5);
    }

    return vspd*10; //vspeed速度是0.1m/s
}

//指定位置的降落。如果位置不在本地，会有平移。
//自主返回也用这个代码，机身可旋转。
void CMicroTaskExec::micro_task_vertical_landing_target_location()
{

    //已经关机了。
    if(micro_task.status == _micro_task_status_shutdown)
    {
        return;
    }

    if(micro_task.status == _micro_task_status_none)
    {
        micro_task.pitch=0;
        micro_task.roll=0;
        //micro_task.voyage_bias=0; //宏任务必须更新这个航线偏离角，否则置零。
        //micro_task.yawspeed=0; //不设定，继承宏任务的设定。
        micro_task.vspeed=0; //>0 降落  30现在代表3m/s的速度，向下。
        micro_task.status=_micro_task_status_inprogress;

        //应该没有必要。
        //最好清理掉原始的调节记录，防止继承之前的调节角度。
        // for(size_t i=0;i<20;i++) {
        //     gps_major_bias_queue.last(i).pitch_bias=0;
        //     gps_major_bias_queue.last(i).roll_bias=0;
        // }


        //20240304+ 将头部指向目标点。这个任务从宏任务下沉过来。
        gps_info last_gps;
        float dist, angle;
        critical_section_enter_blocking(&gps_queue_section);
        last_gps=gps_queue.last();
        critical_section_exit(&gps_queue_section);

        global_gps_dist_angle(last_gps.latitude, last_gps.longitude, micro_task.target_gps_latitude, micro_task.target_gps_logitude, dist, angle);

        if(dist>20.0)
        {
            //距离较大，掉头
            float yaw= _heading_yaw - angle;

            if(yaw > 180.0) yaw-=360.0;
            if(yaw < -180.0) yaw+=360.0;

            micro_task.yaw=yaw; //初始指向往往不是很准，尤其是在有风的情况下。在pin_to_location里随时微调。
            //micro_task.turn_head_mark=false; //20240304 新增的一个标记，记录抵达目标点后是否执行了掉头。
            micro_task.heading_forward=true;
            //logmessage("dist>20, turn head, mark turn back at end\n");
        }
        else
        {
            //距离很小的不掉头处理。
            //micro_task.turn_head_mark=true;
            micro_task.heading_forward=false;
            //logmessage("dist<20, no turn head\n");
        }

        //logmessage("first in landing target\n");
    }



    float curdist, curspd, curdir, curheight; //当前距离，当前速度，当前速度方向
    bool newdata;
    bool b= pin_to_location(curdist, curspd, curdir, curheight, newdata); 

    if(b)
    {
        //20240217修改
        if(newdata)
        {
            //通常都有参考高度。
            if(micro_task.target_refer_gps_height!=0.0)
            {
                float h=curheight - micro_task.target_refer_gps_height; //可能小于0
                micro_task.vspeed= auto_landing_vertical_speed(h, curdist);
            }
            else
            {
                //没有参考高度。
                if(curdist< 5.0)
                {
                    micro_task.vspeed=30; //3m/s
                }
                else if(curdist< 16.0)
                {
                    micro_task.vspeed=20; //2m/s
                }
                else
                {
                    micro_task.vspeed=0;
                }
            }
            
            //检查终点掉头是否应该执行。
            if(micro_task.heading_forward && curdist < 5.0)
            {
                //当前距离较小，可以执行掉头了。
                float yaw= micro_task.yaw + 180.0;
                if(yaw>180.0) yaw-= 360.0;
                micro_task.yaw=yaw;
                micro_task.heading_forward=false;
            }
        }

        bool landed=false;
        bool hadj=height_adjust_with_landing_detect(INAIR_BOTTHRUST, INAIR_TOPTHRUST, landed);

        if(hadj && landed) {
            motor_ctrl.SetBurdens(0,0,0,0,0);
            micro_task.status=_micro_task_status_shutdown;
            _inair=false; //全局空中标记
            //logmessage("landing done, mark micro_task_status done in mtvltl\n");
            return;
        }

        float new_burdens[4];
        float new_thrust;
        balance(new_burdens, new_thrust, gps_major_bias_queue.last().pitch_bias, gps_major_bias_queue.last().roll_bias); 
        motor_ctrl.SetBurdens(new_burdens, new_thrust); 

    }
    else
    {
        //位置调节失败可能是gps信号丢失。后期完善。
        //可以考虑继续使用最后的数据，可以考虑用指南针等。
        //如果考虑到可能的GPS干扰，可以以一个固定的角度飞行一段时间，避过干扰点。

        //一般这种情况不会发生降落。
        micro_task.vspeed=0;
        bool landed=false;

        bool hadj = height_adjust_with_landing_detect(INAIR_BOTTHRUST, INAIR_TOPTHRUST, landed);

        if(hadj && landed) {
            motor_ctrl.SetBurdens(0,0,0,0,0);
            micro_task.status=_micro_task_status_shutdown;
            _inair=false; //全局空中标记
            return;
        }

        float new_burdens[4];
        float new_thrust;
        balance(new_burdens, new_thrust, 0, 0); 
        motor_ctrl.SetBurdens(new_burdens, new_thrust); 

    }

}


//垂直降落代码
//t.vspeed参数为降落速度。测试时设定为0.5m/s，不同的高度上可以有不同的设定
//将来如果增加测地传感器则可以在高空提高降落速度。
//没有测试，不完善。
//没有水平定点，只是保持水平姿态的下降。目前不使用
void CMicroTaskExec::micro_task_vertical_landing()
{

    if(micro_task.status == _micro_task_status_none)
    {
        //首次进入调节高度状态，标记高度气压。
        if(micro_task.vspeed <=0) micro_task.vspeed=10; //防范速度设定错误，正值是下降。
        //防御
        micro_task.roll=0;
        micro_task.pitch=0;
        micro_task.yawspeed=0;

        //首次进入，无需处理降落，忽略。
        bool landed=false;
        height_adjust_with_landing_detect(INAIR_BOTTHRUST, INAIR_TOPTHRUST, landed); 

        //float ref_thrust[4];
        //get_balance_refer_thrust(ref_thrust);
        

        float new_burdens[4];
        float new_thrust;
        balance(new_burdens, new_thrust, 0, 0);
        motor_ctrl.SetBurdens(new_burdens, new_thrust);

        micro_task.status=_micro_task_status_inprogress;

        return;
    }
    else if(micro_task.status == _micro_task_status_done)
    {//已经落地，无需更多调节。
        return;
    }
    else if(micro_task.status == _micro_task_status_inprogress)
    {
        bool landed=false;
        bool hadj=height_adjust_with_landing_detect(INAIR_BOTTHRUST, INAIR_TOPTHRUST, landed);  //自适应定高。

        if(hadj && landed) {
            motor_ctrl.SetBurdens(0,0,0,0,0);
            micro_task.status= _micro_task_status_done;
            _inair=false;
            return;
        }
 
        float pb=0;
        float rb=0;


        if(micro_task.position_keep_switch==1)
        {
            bool b0=gps_position_adjust(); 

            if(b0){
                pb= gps_major_bias_queue.last().pitch_bias;
                rb= gps_major_bias_queue.last().roll_bias;
            }else{

    #if defined USE_OFLOW
                bool b1=oflow_position_adjust_minor_change();
                if(b1) {
                    pb= oflow_bias_queue.last().pitch_bias;
                    rb= oflow_bias_queue.last().roll_bias;
                }
    #endif
            }
        }
        else if(micro_task.position_keep_switch==0)
        {
            //0仅使用光流。
    #if defined USE_OFLOW
            bool b0=oflow_position_adjust_minor_change(); 
            if(b0){
                pb= oflow_bias_queue.last().pitch_bias;
                rb= oflow_bias_queue.last().roll_bias;
            }
    #endif 
        }
        else
        {
            // //光流，gps定点都取消，测试光流无锚点的稳定性。
        }
 

        float new_burdens[4];
        float new_thrust;
        balance(new_burdens, new_thrust, pb, rb);
        motor_ctrl.SetBurdens(new_burdens, new_thrust);

        // if(micro_task.help_landing && LandingCheck()) {
        //     micro_task.status= _micro_task_status_done;
        //     _inair=false;
        //     height_adjust_record har;
        //     har.tmark= get_time_mark();
        //     har.thrust = 0; 
        //     har.hd = 0;
        //     _height_adjust_queue.push(har);
        // }

    }

}


//t.press 指定了盯住的气压值
//t.press_height 指定了盯住的气压值推算高度
//t.tof_height指定了盯住的高度。
//t.vspeed应该为0
//t.yaw 指定了盯住的航向角
//没有状态变化。状态只是正在执行中。由宏任务打断任务状态。
void CMicroTaskExec::micro_task_keep_stable_with_yaw()
{

    //micro_task.status=_micro_task_status_inprogress;

    //高度调节给出的变化量基于上一个负担是最好的表现，这可能是因为气压计延迟很小
    bool landed;
    height_adjust_with_landing_detect(INAIR_BOTTHRUST, INAIR_TOPTHRUST, landed);


    float pb=0;
    float rb=0;

#if defined(USE_GPS) && defined(USE_OFLOW) 
    //gps定点优先使用。
    bool b1=gps_position_adjust(); 
    if(b1) {
            pb= gps_major_bias_queue.last().pitch_bias;
            rb= gps_major_bias_queue.last().roll_bias;
    }else{

        bool b0=oflow_position_adjust_minor_change(); 
        if(b0){
            pb= oflow_bias_queue.last().pitch_bias;
            rb= oflow_bias_queue.last().roll_bias;
        }
    }
#elif defined(USE_GPS)
    bool b1=gps_position_adjust(); 
    if(b1) {
        pb= gps_major_bias_queue.last().pitch_bias;
        rb= gps_major_bias_queue.last().roll_bias;
    }
#elif defined(USE_OFLOW)
    //光流定点优先使用。
    bool b0=oflow_position_adjust_minor_change(); 
    if(b0){
        pb= oflow_bias_queue.last().pitch_bias;
        rb= oflow_bias_queue.last().roll_bias;
    }
#endif

    //这是用来执行自动任务的，不考虑摇杆位置。
    float new_burdens[4];
    float new_thrust;
    balance(new_burdens, new_thrust, pb , rb );
    motor_ctrl.SetBurdens(new_burdens, new_thrust);

}



//保持前飞，盯住航向角
//t.obliquity 主要前倾角度，1-15度代表不同的力度范围
//t.yaw 要保持的航向角度
//t.press 要保持的气压
//t.press_height 要保持的气压推算高度
//t.pitch要为负值，负的越大飞的越快
//目前在航行中调整航向角采用静态调整航向角的方法，即交叉轴的不同发力，效果如何不好说
//还有一种是设定一个滚转角度，使得航线形成弧形，这样也可以纠偏。两种方法需要测试。
//这个微任务目前没有使用。
void CMicroTaskExec::micro_task_move_ahead_with_yaw()
{

    if(micro_task.status==_micro_task_status_none)
    {
        if(micro_task.vspeed!=0) micro_task.vspeed=0; //保持高度，气压协同GPS
        if(micro_task.roll!=0) micro_task.roll=0;
        if(micro_task.yawspeed!=0) micro_task.yawspeed=0;

        micro_task.status=_micro_task_status_inprogress;
    }
    else if(micro_task.status==_micro_task_status_done)
    {
        //外部依据gps定位，可能修改为done. 但微任务必须应对这个状态。
        //不需要特别处理。此时pitch,roll都设置为0，yaw不动。这个状态很快过去，任务会被删除。
    }

    bool landed;
    height_adjust_with_landing_detect(INAIR_BOTTHRUST, INAIR_TOPTHRUST, landed);

    float new_burden[4];
    float new_thrust;

    //oflow_position_adjust();
    //balance(ref_burden, burden, micro_task.pitch + oflow_bias_queue.last().pitch_bias, micro_task.roll + oflow_bias_queue.last().roll_bias);

    //balance(ref_burden, burden, micro_task.pitch, micro_task.roll);
    balance(new_burden, new_thrust, micro_task.pitch, micro_task.roll);

    motor_ctrl.SetBurdens(new_burden, new_thrust);

}

//简单平移，用来微调位置，
//t.obliquity 主要前倾角度，指定前后移动及力度
//t.minor_obliquity，指定左右移动及力度
//t.press 要保持的气压
//t.press_height 要保持的气压推算高度
//这个微任务目前没有使用
void CMicroTaskExec::micro_task_move_around()
{

    if(micro_task.status==_micro_task_status_none)
    {
        micro_task.status=_micro_task_status_inprogress;
    }

    bool landed;
    height_adjust_with_landing_detect(INAIR_BOTTHRUST, INAIR_TOPTHRUST, landed);

    float new_burdens[4];
    float new_thrust;
    balance(new_burdens, new_thrust, 0, 0);
    motor_ctrl.SetBurdens(new_burdens, new_thrust);

}

//电机推力调节，为balance的辅助函数，帮助调节异常值。
//total_thrust是控高函数计算出来的垂直推力，再由平衡函数结合当前倾角算出的电机总推力。
//thrust是平衡函数算出的各电机推力。
//在推力饱和时，减小总推力以提高平衡性。
//在推力过低时，增加总推力以提高平衡性。
//在调节总推力时仍不能达到要求时，压缩电机推力的差异，降低平衡性以防止饱和。
//20240128 微调差异性压缩的步长，原0.9，现0.95，会更细。
//总力调节范围原来是0.8-1.2，改为0.9-1.1
//增加一个返回比例值，代表总推力被调节的比例，没有调节返回1.0，被调低返回<1.0，调高返回>1.0，这个数值
//可以在控高中做参考，之前调整推力时，控高函数没有感知，导致高度控制上会有问题，尤其是在大风影响平衡的情况下。
bool CMicroTaskExec::adjust_burdens(float total_burden, float burdens[4], float& burden_adj_ratio)
{
 
	//总力可调节的上下范围
    //在超高速降落时，单个电机推力可能跌破最低限制，此时需要增加总推力才可以维持平衡

#define TT_UP_BORDER (1.03f)
#define TT_DW_BORDER (0.97f)

	float tmean = 0; //平均
	for (size_t i = 0; i < 4; i++)
	{
		tmean += burdens[i];
	}
	tmean /= 4.0;

	//扣除均值的裸推力。
	float rt[4];
	for (size_t i = 0; i < 4; i++)
	{
		rt[i] = burdens[i] - tmean;
	}

	//寻找最大最小推力，计算他们的差异性。
	float tmin = rt[0];
	float tmax = rt[0];

	for (size_t i = 1; i < 4; i++)
	{
		if (rt[i] < tmin) {
			tmin = rt[i];
		}
		if (rt[i] > tmax) {
			tmax = rt[i];
		}
	}

	//检查正常推力是否满足。
	float tt_m = total_burden / 4.0f; 
    //总推力特别小时，所有推力平均分配无差异。这一点在超级推重比的机器上要注意。那种情况下应降低BOTDUTY的值。
	if (tt_m <= BOTDUTY) {
		//特殊情况。
		for (size_t i = 0; i < 4; i++)	burdens[i] = tt_m;
        burden_adj_ratio = 1.0;
		return true;
	}

	float tt_min = TT_DW_BORDER * tt_m; //推力下界, 总是正值
	float tt_max = TT_UP_BORDER * tt_m; //推力上界, 总是正值


	//首先检查在仅考虑单个上下限时是否一定要压缩平衡差。
	if (tmax - tmin > TOPDUTY - BOTDUTY)
	{
        //最大最小推力超过了范围，推力向内自然收缩。
        //因为裸推力的和是0，所以比例调节不影响总推力。只是缩小推力不平衡性。
		float r = (TOPDUTY - BOTDUTY) / (tmax - tmin);
		for (size_t i = 0; i < 4; i++)
		{
			rt[i] *= r;
		}
		tmax *= r;
		tmin *= r;
	}

	//系统可接受的下限推力和上限推力。
	//两者都为正，但不一定哪个大。
	float t0 = BOTDUTY - tmin;
	float t1 = TOPDUTY - tmax;
	float ts_max = (std::max)(t0, t1);
	float ts_min = (std::min)(t0, t1);
	bool bret = false;

repeat:
	if (ts_max >= tt_m && tt_m >= ts_min)
	{
		//系统能接受的力覆盖了请求的推力，
		//是正常情况，无需调节。
		for (size_t i = 0; i < 4; i++)
		{
			burdens[i] = rt[i] + tt_m;
		}

        //如果是在这里返回的，总推力不变，但可能经过了差异性压缩。
        burden_adj_ratio=1.0;
		return bret;
	}
	else if (tt_max >=ts_max  && tt_min <= ts_min)
	{
		//推力调节范围完全包裹了系统可接受范围。
		//但系统可接受的范围又不包含中间值tt_m, 所以只有两种情况。
		//一个是系统可接受的区域在上半段，一个是下半段。
		//选取一个最接近tt_m的值做推力。
		if (ts_min > tt_m) {
            //在这里，必须提高整体推力来保持平衡。垂直推力加强，飞机意外升高。
            //例子：0.1，0.9，0.2，0.8是平衡要求，1.6是总推力要求，于是总推力被提高到了2.0
			for (size_t i = 0; i < 4; i++)
			{
				burdens[i] = rt[i] + ts_min;
			}

            burden_adj_ratio= ts_min/tt_m;
			return true;
		}

		if (ts_max < tt_m) {
            //在这里，必须牺牲系统向上的推力，降低这个推力来保持平衡。垂直推力减弱，飞机意外降低。
			for (size_t i = 0; i < 4; i++)
			{
				burdens[i] = rt[i] + ts_max;
			}

            burden_adj_ratio= ts_max/tt_m;
			return true;
		}
	}
	else if (ts_min <=tt_max && ts_max >= tt_max) //高端部分重叠
	{
		for (size_t i = 0; i < 4; i++)
		{
			burdens[i] = rt[i] + ts_min;
		}
        burden_adj_ratio= ts_min/tt_m;
		return true;
	}
	else if (ts_max >= tt_min && ts_min <= tt_min) //低端部分重叠
	{
		for (size_t i = 0; i < 4; i++)
		{
			burdens[i] = rt[i] + ts_max;
		}
        burden_adj_ratio= ts_max/tt_m;
		return true;
	}
	else
	{
		//系统可接受的推力范围和推力可调节范围完全无交集。区间游离在外。
		//此时需要压缩平衡差来获得更大的可接受范围。
		//不断以0.95的比例去压缩平衡差，然后检查是否能够重叠推力范围。
			for (size_t i = 0; i < 4; i++)
			{
				rt[i] *= 0.95f;
			}
			tmax *= 0.95f;
			tmin *= 0.95f;

			t0 = BOTDUTY - tmin;
			t1 = TOPDUTY - tmax;
			ts_max = (std::max)(t0, t1);
			ts_min = (std::min)(t0, t1);
			bret = true;
			goto repeat;

	}

    burden_adj_ratio= 1.0;
    return false;

#undef TT_UP_BORDER
#undef TT_DW_BORDER
}

//20230414新增，用于替换balance_mod
//主动考虑水平倾角的平衡调节，实现对垂直分力的更好控制
//不需要传入一个总力度修正值。主动从高度调节的最后数据中提取，然后根据最后的倾角来灵活调节总力大小。
//新的方法应该能更好的把握垂直方向的推力。在1/50秒的高度调节时间内，系统有不同的倾角可以有不同的总力。
//而老的方法总力在每次高度调节后总力是固定的，没有考虑到当前的倾角情况。从完全水平到有倾角时，适应能力不强
//不能即时根据倾角去调节总力，而是等待传感器感知到坠落后再去纠正，容易引起震荡。
//20230419试飞证实可行。
//200hz似乎造成f450极大机身震动。比较适合500hz.
//20230529新增一个功能，Micro_task.yawspeed指定了Yaw角的旋转，这个功能原来在上层处理，现在下沉到底层处理。
//现在添加在这个平衡函数里，老的平衡函数里不添加。因为处理这个旋转速度，取消原来的参数yaw，直接使用micro_task.yaw做目标。
//20230710突然新发现一个问题。平衡在空中会突然侧翻，说明平衡代码不够稳定，怀疑和角速度的处理有关，测试风力较大
//20230710 已确认突然从左打杆最大转入右打杆最大时翻车。说明加力偏大了。怀疑是KP调节的问题。
//目前KP调节限制1度的夹角，然后参数放的很大，这样的问题是小角度时调节相应很灵敏，大角度调节时持续增力，导致力量累积过大。
//应该基于真实角度差，降低调节系数。
//20230712 将函数修改为固定500hz, 另外取名balance_500hz, 保留此函数不动。可以简化一些计算。
//20230716 在小机器上有轻微摆动现象。可能和电池下挂重心低有关。重心低从侧倾到回正不需要那么大的力。力量过头了就引起摆动。
//尝试缩小积分数量到10个。Kp由0.002*RATIO->0.0015*RATIO，摆动现象似有好转。
//20230717 昨日测试中又侧翻，电机可能过热，怀疑平衡函数调节参数偏大，导致电机容易高温，地面平稳测试中，电机发热并不大。
//20230719 似乎所有的侧倾都是右侧倾倒？要检查右侧迅速减力，左侧迅速加力的情况。
//20231006 为了处理颠倒的情况，观察imu的计算数据如下
//机头朝前方，机身水平，顺时针旋转机身，即只动滚转方向，最初滚转角度roll=0, zxis=1.0, 
//由水平旋转到垂直地面时, roll=90, zxis=0, 继续旋转90度时，刚好机身上下颠倒，此时roll=0, zxis=-1.0
//在此过程中，roll由90减小为0， zxis由0减小为-1.0
//再继续旋转90度，roll=-90, zxis=0, 这期间roll是减小的，zxis增加。
//再旋转90度就是回正，此时roll=0, zxis=1.0;
//在完全颠倒的情况下，如果是水平状态，则roll=0,抬高右侧时，roll增大，降低右侧，roll减小，这和正面朝上时刚好相反。
//另外，pitch此时变化也不同，压低机头pitch<0, 抬高机头pitch>0, 但他们的起点不是0，而是180/-180
//即压低机头，pitch由-180开始向-179，-178方向走，抬高机头pitch由180向179，178方向走。

//20231008 通过高空录像的情况，可以看到横向摆动比较严重。想到由于电池是纵向摆放。横向转动惯量会较小
//所以在滚转调节方面，可以力度较小，而前后倾角调节方面，需要力度较大，这两者如果共用一个调节力度则很难
//使得两个方向的调节都达到稳定，所以设置一个新的参数，即滚转调节力度参数，是一个相对前后倾角调节力度的参数
//由于机器滚转方向的转动惯量比倾角方向的要小，所以参数小于1.

//20231127 一些极端情况下，比如空中遇到了大的对流。某一侧纠正平衡会达到电机上限。此时要保平衡而短暂放弃高度。
//即此时放弃推力的总和值，使他略微下降，但保证电机之间的推力差，否则由于电机达到推力上限，而力差不够时
//可能造成平衡无法及时恢复，导致侧翻。电机达到上限1.0时的溢出量在其他电机上均匀扣除可能是一个处理办法。
//平衡和高度控制这里是有矛盾的。只能有一个取舍。保平衡就不能确保高度。
//另一个修改是倾角限制方式。修改已测试，至少看起来没问题。
//在完全迁就平衡的情况下，也有一些新的问题。当垂直推力要求很低时，比如0.5，通常是降落情况，此时，由于不平衡的累积
//四个电机的不平衡性会较大，这样初次调节可能会得到类似于 -0.2，-0.4，0.3，0.8这样的分配力。其和为0.5，但在下边界抑制下
//所有电机都要统一增加一个0.4以上的推力来保证没有推力为负的情况。这样就变成了0.2，0，0.7，1.0的推力。如果底限不是0，
//还会更高。比如0.3，0.1，0.8，1.0，这样的推力也许完全可以保持悬空状态。干扰了降落判断。

//20231202新观察
//为了保证平衡算法，而大幅度的牺牲了垂直推力的匹配。这是另一个极端。之前的算法没有这方面的问题。严格保证平衡和严格保证
//可能都存在问题。考虑平衡算法可以在推力要求的一定范围内浮动，以增强平衡自主性，同时维持一个合理的推力要求。
//总推力不超过垂直推力的120%，不少于推力的80%，在此范围内自行调整，以增强平衡性。在平衡性不受影响时，保持推力要求。
//在推力很低时，如果平衡差异很大导致需要增加推力的，要抑制推力的不均衡性以降低平衡控制保证推力满足要求不超过范围。
//在推力很高时，降低推力到下限都不能达到平衡要求的，也要抑制不均衡性降低平衡控制以满足推力不会过低。

//20240128修改，adjust_thrust限制范围是0.9-1.1，相对之前缩小了借力范围，并且此函数返回对总推力的调节量
//以便控高函数可以准确计算下一个推力，不至于平衡借力时盲目推高高度。
//本平衡函数在控高记录序列里留下执行推力的记录，以便控高更好的分析下一个合理推力。
//跑一次留下一个执行推力记录。

//平衡函数曾经为了灵活性，减小了转动阻尼，这会导致稳定性减弱。笨重的机器遇到大风时，其稳定性大幅降低。
//由此又产生借高问题，导致高度控制的稳定性不足。目前的阻尼，比较适合推重比较大的机器，对于笨重的机器偏低了。
//可以考虑每次发生借高时，都尝试增加一点阻尼，降低灵活性增强稳定性。暂时没做。对于笨重的机器可调高Kd值，增加旋转阻尼。

//20240130修改，取消参考推力输入，新增垂直推力返回值。区分burden/thrust两个概念。thrust指推力垂直分量，burden指电机实际负荷。
//20240229 目前倾角欠缺灵活性，但稳定性较好，这主要是因为阻尼较多，角速度，角加速度都用作阻尼，
//如果将角加速度作为一个调节项或许会更灵活些。但目前这不是主要问题，暂时放放。

//20240315 新增了角加速度调节项。所以现在有角度差异调节项kp，角度差异积分调节项kd，角速度阻尼项kd，角速度调节项kw
//角加速度阻尼项kx，角加速度调节项kxw。外加yaw调节项，yaw调节没有分那么细，只有角度差异项和角速度项，没有加速度项。
void CMicroTaskExec::balance(float new_burdens[4], float& new_thrust, float pitch, float roll)
{

//之前用60，降低为50，测试在4电池机身上灵活性太差。
//因为陀螺仪滤波调节的较大。陀螺仪数据延时就大了。这样系统调节的慢了就会摆动。
//所以陀螺仪延迟大的，这个系数可能要调大点。角速度阻尼也可能要调大点。
//2306 kv1150 用60左右合适。无分桨叶。
//2806 kv1050 三叶8寸桨，用85+合适，之前用60太差了，平衡力不强，目前感觉85还可以往上一些，机架还是q380机臂加定制碳板。
//#define RATIO (100) //20241005 再度调高到110，之前100，担心高空空气稀薄平衡力不够用。测试感觉110太大了。

//#define RATIO (90) //20241021 全碳机身，2806， 900kv, 机臂较软，应该不能太大

//#define RATIO (75) //2306电机，1150kv, 全碳机身。80似乎过头了。

    //20231127，修改倾角限制，将两个倾角联合起来考虑计算总倾角然后再限制。
    //20240123，这里的倾角限制放大到35度。之前是25度，抗风能力弱。

    //20240723修改限制算法。
    float want_fx= tan(roll/57.2958);
    float want_fy= -tan(pitch/57.2958);
    float want_fmod= sqrt(want_fx*want_fx+want_fy*want_fy);
    if(want_fmod>0.7) { //tan(35)=0.7
        pitch*= 0.7/want_fmod;
        roll*= 0.7/want_fmod;
    }


    //10个数据的时间差。可计算调节频率和每次调节的力度系数。
    //如果在100左右，说明调节频率是100次。50左右则是200次，200左右是50次。
    imu_info wi0; //取两个点是为了计算角速度的加速度，距离如果很近则噪音会很大
    imu_info wi1; //自动选择两个采样的距离，需要计算来定。

    float last_yaw; 
    //20230704取消积分，节约计算力。
    float adj10=0;  
    float adj11=0; //角度误差积分项。20230519，积分似乎作用很小，暂时关闭他，目前算力不足。

    float zaxis_cos; //30度 0.866，45度 0.707， 45度是极限了。
    size_t num_acum=50; //积分数量
//涉及到imu的数据一次性加锁读出来，减少主线程的延迟。后期就不需要访问这个数据了。

    //20240229+ 由于角速度为了低延迟，滤波不是很强，所以计算角加速度波动较大，
    //这里尝试用多均值去平滑角加速度。
    float gyrox_dif=0; //5个数据跨度的角速度差。即角加速度的度量。20240229
    float gyroy_dif=0; //5个数据跨度的角速度差。即角加速度的度量。20240229

    critical_section_enter_blocking(&imu_queue_section);

    wi0= imu_queue.last();

    //wi1= imu_queue.last(5); //长期以来这里选择5个数据之前的数据做角加速度运算。由于距离近，可能波动较大。现改为10个数据差距。
    wi1= imu_queue.last(10);  //10个数据差距实际是20毫秒。

    //5个差异值累计，5个数据的跨度。用来当作角加速度去调节。20240229
    for(size_t num=0; num<5; num++)
    {
        gyrox_dif+= imu_queue.last(num).gyro[0] - imu_queue.last(5+num).gyro[0];
        gyroy_dif+= imu_queue.last(num).gyro[1] - imu_queue.last(5+num).gyro[1];
    }


    //积分角度误差项。原始积分距离200个数据。为适应变化的频率，修改。
    //20230519 为减轻计算压力，积分作用很小，考虑关闭他。
    //num_acum = 100*1000*50/tg50; //要积分的个数
    //20230704取消偏角积分

    for(size_t i=1;i<num_acum+1;i++) {
        adj10 += (imu_queue.last(i).angle[0]-pitch);
        adj11 += (imu_queue.last(i).angle[1]-roll);
    }

    critical_section_exit(&imu_queue_section);
    zaxis_cos= wi0.zaxis;
    last_yaw = wi0.angle[2];


    //垂直推力限制要求
    float vertical_thrust= _height_adjust_queue.last().thrust;
    float total_burden;
    if(std::isnan(zaxis_cos)||std::isinf(zaxis_cos))
    {//不该发生
        total_burden=vertical_thrust;
    }
    else if(zaxis_cos >1.0) {
        //不该发生
        total_burden= vertical_thrust;
    }
    else if(zaxis_cos < 0.3 && zaxis_cos >0) {
        //机身失控，倾角过大。但还没有颠倒，45度以上的倾角就认为过大。
        total_burden= vertical_thrust/0.3;
        if(total_burden > 3.0) total_burden=3.0; //特殊情况，限制严格点
        vertical_thrust= total_burden*zaxis_cos; //大倾角下，实际垂直推力变小了。

    }
    else if(zaxis_cos <0) {

        if(zaxis_cos < -0.2)
        {
            //严重颠倒，直接关断
            new_burdens[0]=0;
            new_burdens[1]=0;
            new_burdens[2]=0;
            new_burdens[3]=0;

            //这里可能有问题，负值可能导致处理上发生错误。
            new_thrust = 0;
            return;
        }
        else
        {
            //或可抢救。
            if(wi0.angle[1]>0) {
                new_burdens[0]=0.1*vertical_thrust;
                new_burdens[1]=0.4*vertical_thrust;
                new_burdens[2]=0.1*vertical_thrust;
                new_burdens[3]=0.4*vertical_thrust;
            }else{
                new_burdens[0]=0.4*vertical_thrust;
                new_burdens[1]=0.1*vertical_thrust;
                new_burdens[2]=0.4*vertical_thrust;
                new_burdens[3]=0.1*vertical_thrust; 
            }

            new_thrust = vertical_thrust;
            return;
        }

    }else{
        //正常，理论上是这样，实际由水平转倾斜的过程中高度总是会先有个降低。
        //这有可能是倾角先变化到位，而拉力有延迟。电机的KV值比较低，响应的速度就慢，拉力有延迟是肯定的。
        //低KV值电机不可能像高KV电机那么灵活。所以可能有必要限制倾角的变化速度，从而同步这两者。
        total_burden = vertical_thrust/zaxis_cos;
        FLOAT_LIMIT(total_burden, 0, 4.0);
    }

    //因为控高限制最大INAIR_TOPTHRUST, 经过倾角调节后，总量甚至可能大于4.0
    //理论上这里可以是4.0，
    //if(total_burden >3.9) total_burden=3.9;

    //之前是输入参数，现在参考负载自取
    float ref_burdens[4];
    get_refer_burdens_for_balance(ref_burdens);

    float sum_ref_burden=0;
    for(size_t i=0;i<4;i++) {
        sum_ref_burden+= ref_burdens[i];
    }


    float change= total_burden - sum_ref_burden; //此次增加的总力度。

    //如果每秒100次调姿，这个系数是1.0, 200hz=0.5; 500hz=0.2
    //每次平衡调节的步长系数。是为了不同的调姿频率保持稳定的调节速度。
    //当前速率500hz,所以设定0.2
#define fstep (0.2f)

//#define Kp (0.0004*RATIO)  //轻微调大以加强角度响应速度。20240314, 这个kp在2306上无问题，但在2806上似乎太大。
//#define Kp (0.03)  //降低了KP之后似乎2806机器不抖了。
//#define Kp (0.5)  //非线性化处理后的参数。20250327
//参数过强时，和强阻尼产生对抗，则导致机身震动。
    float angle_dif0 = wi0.angle[0] - pitch;
    float angle_dif1 = wi0.angle[1] - roll;

    //不限制角度差，但非线性化。
    float adj00 = balance_p_ratio *SigmoidTrans(angle_dif0/10.0);
    float adj01 = balance_p_ratio *SigmoidTrans(angle_dif1/10.0) * ROLL_ADJUST_RATIO;

//#undef Kp
//角度差积分调节项，积分时间较短，防止摇摆的目的
//#define Ki (0.00012*RATIO)
//#define Ki (0.1) 

    adj10/=num_acum;
    adj11/=num_acum;
    //FLOAT_LIMIT(adj10, -5.0, 5.0);
    //FLOAT_LIMIT(adj11, -5.0, 5.0);

    adj10= balance_i_ratio*SigmoidTrans(adj10/5.0);
    adj11= balance_i_ratio*SigmoidTrans(adj11/5.0)* ROLL_ADJUST_RATIO;

    //积分是辅助性的微调，要适度限制范围。范围大了摆动大。
    // FLOAT_LIMIT(adj10, -2.0, 2.0);
    // FLOAT_LIMIT(adj11, -2.0, 2.0);

    //adj10*= Ki;
    //adj11*= Ki * ROLL_ADJUST_RATIO;

//#undef Ki
//D调节，角速度阻尼器，增强系统稳定性，抑制角速度，太小会导致大幅晃动失稳，这个阻尼是稳定的保证。

    //20230802对阻尼做了修改，首先是远离阻尼放松到15度侧倾，15度以上阻尼二次方增加，在15度时阻尼连续无突变。
    //在15度以内是常规阻尼区。15度以外回正依旧是常规阻尼，远离的有加强阻尼。
    float adj22;
    float adj23;
    //上面是常规阻尼，各处相同，下面是远离阻尼，不对称。
    //为了灵活性，上面的常规阻尼降低。4.0->3.0，远离阻尼放宽到10度以上。
    //为加强灵活性，在15度以后才引入远离阻尼。20230802
    //曾经引入不对称阻尼，回正阻尼更小，但感觉稳定性下降了，尤其是在高速降落时，机身严重摆动。取消不对称阻尼。

    //20240123，考虑当前的整体倾斜度再判断是否需要加强远离阻尼。原来的方法是两侧倾角独立判断，20度以上加强远离阻尼。
    //现在放开到整体30度倾斜后再加强远离阻尼。

    //obliquity = acos(cos(wi0.angle[0] / 57.2958)*cos(wi0.angle[1] / 57.2958))*57.2958; 
    //考察当前倾角。
    //20240723 修改算法
    float cur_fx= tan(wi0.angle[1]/57.2958);
    float cur_fy= -tan(wi0.angle[0]/57.2958);
    float cur_fmod= sqrt(cur_fx*cur_fx+cur_fy*cur_fy);

    if(cur_fmod > 0.5773) //tan(30)=0.5773
    {
        //倾角偏大，应用远离阻尼。
        //两个方向，任何一个是远离状态时，都加强阻尼。

        //20250323改为非线性调节
        //float tg0=SigmoidTrans(wi0.gyro[0]/45.0); //45度/秒角速度不算太快，变为0.462，如果这个速度调节力不变，系数需要放大97.4倍。
        //float tg1=SigmoidTrans(wi0.gyro[1]/45.0);

        //先计算正常阻尼值。
        // adj22 = Kd * wi0.gyro[0];
        // adj23 = Kd * wi0.gyro[1];
        adj22 = balance_d_ratio * SigmoidTrans(wi0.gyro[0]/60.0);
        adj23 = balance_d_ratio * SigmoidTrans(wi0.gyro[1]/60.0);

        float adjr=1.0; //阻尼调节增强系数。
        if(cur_fmod > 1.0355) {
            adjr=1.35; //46+
        }else if(cur_fmod > 0.9657) {
            adjr=1.3; //44-46
        }else if(cur_fmod > 0.9004) {
            adjr=1.25; //42-44
        }else if(cur_fmod > 0.8391) {
            adjr=1.2; //40-42
        }else if(cur_fmod > 0.7813) {
            adjr=1.16; //38-40
        }else if(cur_fmod > 0.7265) {
            adjr=1.12; //36-38
        }else if(cur_fmod > 0.6745) {
            adjr=1.08; //34-36
        }else if(cur_fmod > 0.6248) {
            adjr=1.05;  //32-34
        }else{
            adjr=1.02; //30-32
        }
 
        if(wi0.angle[0] > 0.0 && wi0.gyro[0] > 0) 
        {
            //正远离，增加阻尼。
            adj22*=adjr;
        }
        else if(wi0.angle[0] < 0.0 && wi0.gyro[0] < 0)
        {
            //负远离，增加阻尼。
            adj22*=adjr;
        }

        if(wi0.angle[1] > 0.0 && wi0.gyro[1] > 0) 
        {
            //正远离，增加阻尼。
            adj23*=adjr;
        }
        else if(wi0.angle[1] < 0.0 && wi0.gyro[1] < 0)
        {
            //负远离，增加阻尼。
            adj23*=adjr;
        }
    }
    else 
    {
        //正常范围倾角，正常阻尼
        // adj22 = Kd * wi0.gyro[0];
        // adj23 = Kd * wi0.gyro[1];
        //20250323改为非线性调节
        adj22 = balance_d_ratio * SigmoidTrans(wi0.gyro[0]/60.0);
        adj23 = balance_d_ratio * SigmoidTrans(wi0.gyro[1]/60.0);
    }
 
    adj23*= ROLL_ADJUST_RATIO; //20231008+ 滚转方向调节力度系数调节，用于不对称结构。

//#undef Kd
    //最后取消adj20,adj21, 用adj22,adj23代替。

//自定义扩展调节，角加速度阻尼器，和当前角加速度反方向，抑制所有角加速度
//0.2也可以，现在稍调小一点，阻尼过强易高频震动。
//#define Kx (0.15)
    //角加速度可以是一个相当大的值，由于角速度存在噪音，求差间隔短，可能比角速度大的多。
    float gyrox_acc= gyrox_dif*20.0; //5个数据采样，间隔5个数据的差，数据频率500hz,所以*20.0换算为真实角加速度。
    float gyroy_acc= gyroy_dif*20.0;
    //系数降低10倍，这里的底数3000->300
    float adj30 = balance_x_ratio * SigmoidTrans(gyrox_acc/3000.0f); //>0, 则需要压低机头
    float adj31 = balance_x_ratio * SigmoidTrans(gyroy_acc/3000.0f)*ROLL_ADJUST_RATIO; //>0, 则需要压低左侧 20231008+调节系数
//#undef Kx
//角速度调节项，之前角速度只作为阻尼项存在，机器的灵活性较差，反应迟钝
//调节基于期望的角速度和当前角速度的差
//#define Kw (0.1)
    //依照上面计算的角度差，期望的角速度是0.333秒抵达目标位，所以*3.0
    //在运动型机器上，可以期望更少的时间抵达设定角度，那么乘数可以放大到5.0，8.0等。
    //angle_dif0是当前角-期望角，比如当前角是0，期望角是5，那么期望的角速度应该是5*3.0
    //angle_dif1是当前角-期望角，比如当前角是0，期望角是5，那么期望的角速度应该是5*3.0
    // float wanted_gyro0 = -angle_dif0*3.0; //在软机架上不要期望太快匹配到角速度
    // float wanted_gyro1 = -angle_dif1*3.0;

    //不同的角度差，期望不同的角速度。20250406
    float wanted_gyro0, wanted_gyro1;

    float abs_dif0= fabs(angle_dif0);
    float abs_dif1= fabs(angle_dif1);

    if(abs_dif0>15.0) {
        wanted_gyro0 = -angle_dif0*7.0;
    }else if(abs_dif0>10.0) {
        wanted_gyro0 = -angle_dif0*5.0;
    }else if(abs_dif0>5.0) {
        wanted_gyro0 = -angle_dif0*3.0;
    }else{
        wanted_gyro0 = -angle_dif0*2.0;
    }

    if(abs_dif1>15.0) {
        wanted_gyro1 = -angle_dif1*7.0;
    }else if(abs_dif1>10.0) {
        wanted_gyro1 = -angle_dif1*5.0;
    }else if(abs_dif1>5.0) {
        wanted_gyro1 = -angle_dif1*3.0;
    }else{
        wanted_gyro1 = -angle_dif1*2.0;
    }
//为了稳定性，不管角度偏差多大，限制角速度，太快的角速度电机也不一定能反应过来，那就失去平衡了。
//在运动型机器上这个限制应该可以放的很大。比如180，360等。
    FLOAT_LIMIT(wanted_gyro0, -120.0, 120.0);//20240315+
    FLOAT_LIMIT(wanted_gyro1, -120.0, 120.0);

    //期望的角速度和当前角速度做差异，得到差值。
    float gyro0_to_want = wanted_gyro0 - wi0.gyro[0]; //>0 说明期望比实际大，需要前电机增力，后电机减力，抬高机头
    float gyro1_to_want = wanted_gyro1 - wi0.gyro[1]; //>0 说明期望比实际大，需要左电机增力，右电机减力，抬高左侧

    //非线性，参数放大百倍
    float adj50= balance_w_ratio * SigmoidTrans(gyro0_to_want/45.0);  //前加后减
    float adj51= balance_w_ratio * SigmoidTrans(gyro1_to_want/45.0) * ROLL_ADJUST_RATIO; //左加右减
//#undef Kw

//20240315+ 角加速度调节项。为了提高运动性能。
//#define Kz (0.02) 
    //上面做了角加速度阻尼项，没有调节功能，这里添加调节成分。
    //如果机臂刚性不好，期望角速度迅速匹配是不现实的，因为迅速加力被机臂弹性吸收，机身反应过来有很大延迟，
    //这样容易引发震荡，机臂引起的柔性震荡，既然追求远航，机臂的刚性就不是很强，所以这里之前期望0.2秒匹配角速度修改为0.5秒
    //之前乘系数5.0，现在乘2.0。20250322修改。上一个调节项也是期望0.5秒达到匹配。
    //20250326 还原，不改动，恢复期望0.2秒匹配，即*5.0
    float gyrox_want_acc= gyro0_to_want*3.0; //差异希望在0.333秒内匹配，所以*3.0
    float gyroy_want_acc= gyro1_to_want*3.0; //在软机架上不要期望太快匹配到加速度
    float adj60 = balance_z_ratio*SigmoidTrans((gyrox_want_acc- gyrox_acc)/3000.0f); //左增右减
    float adj61 = balance_z_ratio*SigmoidTrans((gyroy_want_acc- gyroy_acc)/3000.0f)*ROLL_ADJUST_RATIO; //前增后减
//#undef Kz

    //将所有调节系数抽取出来求和，再乘统一系数，比每个调节乘系数更简洁。
    float all0= (- adj00 - adj01 - adj10 - adj11 - adj22 - adj23 - adj30 - adj31 + adj50 + adj51 + adj60 + adj61)*balance_main_ratio*fstep;
    float all1= (- adj00 + adj01 - adj10 + adj11 - adj22 + adj23 - adj30 + adj31 + adj50 - adj51 - adj60 + adj61)*balance_main_ratio*fstep;
    float all2= (+ adj00 - adj01 + adj10 - adj11 + adj22 - adj23 + adj30 - adj31 - adj50 + adj51 + adj60 - adj61)*balance_main_ratio*fstep;
    float all3= (+ adj00 + adj01 + adj10 + adj11 + adj22 + adj23 + adj30 + adj31 - adj50 - adj51 - adj60 - adj61)*balance_main_ratio*fstep;

    //增加角速度调节项20240308+
    new_burdens[0] = ref_burdens[0] + all0 + change/4.0; //左前
    new_burdens[1] = ref_burdens[1] + all1 + change/4.0; //右前
    new_burdens[2] = ref_burdens[2] + all2 + change/4.0; //左后
    new_burdens[3] = ref_burdens[3] + all3 + change/4.0; //右后

    new_thrust = vertical_thrust; //新增返回值，垂直推力。正常就按控高给出的推力来。

//对角失衡检查，限制范围，同时保持增减力不变。
//因为机身刚性不足，起飞时常有极大不平衡的情况。

//交叉轴在调节姿态时可能会造成不平衡，这本身没有逻辑问题。
//为避免单一对角线上的力过大，需要平衡两对角线的发力，限制差距。
#define CROD (0.4)

//斜对角差别矫正。限制对角差在0.6以内，虽然会限制纠正倾斜的能力，但避免极端情况。
#define CRO_DIF (0.6)

    float xd0=new_burdens[0]-new_burdens[3];
    float xd1=new_burdens[1]-new_burdens[2];

    if(xd0>CRO_DIF) {
        new_burdens[0]-= (xd0-CRO_DIF)/2.0f;
        new_burdens[3]+= (xd0-CRO_DIF)/2.0f;
    }
    else if(xd0<-CRO_DIF) {
        new_burdens[0]-= (xd0+CRO_DIF)/2.0f;
        new_burdens[3]+= (xd0+CRO_DIF)/2.0f;
    }
    
    if(xd1>CRO_DIF) {
        new_burdens[1]-= (xd1-CRO_DIF)/2.0f;
        new_burdens[2]+= (xd1-CRO_DIF)/2.0f;
    }else if(xd1<-CRO_DIF) {
        new_burdens[1]-= (xd1+CRO_DIF)/2.0f;
        new_burdens[2]+= (xd1+CRO_DIF)/2.0f;
    }

#undef CRO_DIF


//检查越界情况，迁就高度控制，完全不允许越界，并保持总力符合高度调节给出的值。
//老的算法很可能会改变总值。
    //20231203 新算法，优先考虑平衡，其次考虑控高，平衡算法可以在一定范围内调节控高推力以稳定平衡。
    //函数内部会检查单个电机的上下限。
    float burden_adj_ratio;
    bool abnormal= adjust_burdens(total_burden, new_burdens, burden_adj_ratio);
    if(abnormal) {
        new_thrust= vertical_thrust*burden_adj_ratio; //垂直推力修正。
        return;
    }

    //不平衡角度越大，yaw角调节力度越小，避免yaw角调节带来的不稳定性。
    //重写这个调节比例。zaxis_cos反应了这个倾斜度，不需要算。
    float yaw_adj_ratio=zaxis_cos*zaxis_cos;
    FLOAT_LIMIT(yaw_adj_ratio, 0.6, 1.0); //20250329+


//现在在这里新增yawspeed的调节。
//yawspeed!=0时，直接计算出rot_angle, 而yaw随动，盯住当前的yaw角。
//当yawspeed==0时，用原来的方法，盯住yaw的设定去调节。
//这样当yawspeed由！=0转为=0时，可以立即停止旋转。因为yaw一直盯住当前实际角。

//新方法, 统一调整到0-360度计算。逆时针为角度增加方向。

    float rot_angle; //旋转角度，带方向，逆时针为正。这样就不必区分旋转方向去加力了。

    if(micro_task.yawspeed)
    {
        
        micro_task.yaw=last_yaw; //如果修改这个追踪角度，不是当前角，而是顺着旋转方向增加一个提前量，那么可能就会减小摆动。
        //micro_task.yaw += wi0.gyro[2]*0.3; //当前角速度0.3秒提前量。缓解摆动。这样调节后，效果确实好些，基本没有摆动。
        //if(micro_task.yaw >180) micro_task.yaw -= 360;
        //else if(micro_task.yaw < -180) micro_task.yaw += 360;

        rot_angle = float(micro_task.yawspeed)*3.0; //这里乘10.0似乎大了，速度太快，而且yawspeed=3就饱和了。改为3.0
        //80->90放松点限制，20230720
        //90->30收紧限制，20231009，因为现在pin_to_location增加了自动调节yaw，那么大倾角飞行时
        //yaw调节的太快可能导致机身旋转过快，姿态会不稳定。
        FLOAT_LIMIT(rot_angle, -45.0, 45.0); //放大为45，之前是30
    }
    else
    {
        rot_angle = micro_task.yaw - last_yaw; //20230628修改，不能采用简单平均的yaw角度
        if(rot_angle > 180.0) rot_angle -=360;
        else if(rot_angle < -180.0) rot_angle +=360;
        //80->90放松点限制，20230720
        //90->30收紧限制，20231009
        FLOAT_LIMIT(rot_angle, -45.0, 45.0); //上面是圆周角范围调节，这里是限制旋转速度。
    }

//之前数值相同，感觉阻尼不够，转起来不容易停下容易摆动。
//关于yaw调节力度，机臂形态影响很大，原始的F450的机臂居中。而定制的两个垂直的机臂对yaw角调节不敏感，而且yaw角漂移大。
//后定制的两个水平的机臂对yaw角调节非常敏感，指向的稳定度也还行。根据新定制机臂的表现，两系数暂时都改为0.04，原来是0.06.
//两者改为0.04还是大，现改为0.02/0.03
//BALANCE_RATIO缩小了百倍，所以前面放大百倍

//kv值高的电机可能需要增加这个调节系数，因为电机的扭矩小。
//之前2.0/3.0 现在各增加30%成为2.6，3.9 因昨日平飞测试发现yaw角易在顶风飞行中旋转。用的8寸桨，似乎扭力小。
//之前非线性调节的底数是15，15度偏角以内调节力度较大，超过则不能提高很快。
//现在修改为底数30，这样系数就要放大一倍。
#define Kzp (6.0) //原始这里是0.02，老机架够用，定制机架发飘，可能是定制机架刚性不足。
#define Kzd (9.0)

    //对于新老机架，这些调节表现完全不同，老机架yaw角稳定，新机架yaw角非常不稳定。
    //在同样的调节参数下，老机架表现出调节过头，而新机架表现的调节不够。这可能是新机架刚性不够造成的。

    float yaw_angle_trans= SigmoidTrans(rot_angle/30.0);
    //float yaw_angle_trans= rot_angle/5.0; //-1~1之间。上面限制了90度
    //FLOAT_LIMIT(yaw_angle_trans, -1.0, 1.0);

    float adjyaw0 = Kzp * yaw_angle_trans;  //线性调节项，负值是顺时针转角，正值是逆时针转角，

    float wanted_yaw_gyro = rot_angle/2.0; //希望有个角速度符合2秒到达旋转的角度。

    float yaw_gyro_dif= wanted_yaw_gyro - wi0.gyro[2]; //期望角速度和当前角速度的差值。
    //float yaw_gyro_dif= wanted_yaw_gyro - gyro2_last_mean3; //20240325

    //FLOAT_LIMIT(yaw_gyro_dif, -30.0, 30.0); //这可以限制加速度。

    //限制这个差距可以减少机身震颤，如果当前顺时针转，突然期望逆时针转，这个差值很大时会导致机身强烈震颤，甚至可能失去平衡。
    //始终维持期望的角速度和当前的角速度之间的差值不至于过大，可以使旋转加减速更平滑。旋转更平滑的好处应该是yaw角更准确，因为加速度比较小。
    //限制使得角加速度控制在小范围。20230709+
    //简单限制幅度是错误的，导致yaw角摆动，阻尼效应很差。
    //一个办法是针对角速度加入阻尼，一个办法是回归原代码，不限制角速度的增减。考虑加入加速度阻尼，并适当放松加速度限制。
    //阻尼值和当前角速度相反，wi0.gyro[2]如果是大于0，是逆时针旋转，则应该给出一个负数做阻尼。

    //float adj42 = 2.0/(1+exp(wi0.gyro[2]/50.0)) -1.0; //这里已经对角速度取反了。做阻尼。
    //float adj42 = SigmoidTrans(-wi0.gyro[2]/50.0); //角速度做阻尼。
    //这里似乎少了系数。
    //float adj42 = Kzd*SigmoidTrans(-wi0.gyro[2]/50.0); //角速度做阻尼。增强阻尼。
    //float adj42 = Kzd*(-wi0.gyro[2]/60.0); //角速度阻尼项。
    //如果机器本身角速度是0，这个算出来和wanted_yaw_gyro同向，也和rot_angle同向
    //底数由60调节为20，稳定性大大增强。现在有速度这个阻尼就够了。不需要加速度。
    //float yaw_gyro_trans = yaw_gyro_dif/30.0; //角速度调节项
    float adjyaw1 = Kzd * SigmoidTrans(yaw_gyro_dif/30.0); //角速度调节项，负值是顺时针旋转，正值是逆时针旋转

#undef Kzp
#undef Kzd

    //正值使顺时针的电机加力，机身逆时针转。
    float adjyawsum = (adjyaw0+adjyaw1) * yaw_adj_ratio * balance_main_ratio * fstep; //似乎是这个。

    //20230719, 之前没有限制这个，恐怕不合适。
    //增加这个限制后，航向角摆动，显得十分不稳定。需要放松这个限制。
    //FLOAT_LIMIT(adj4sum, -0.02, 0.02); 

    FLOAT_LIMIT(adjyawsum, -0.5, 0.5);

//cro_a 增加可以提高机身逆时针转动

//调节交叉力度之前，先检查当前交叉力的差异，超过阈值之后不能再度强化调节。
//可以防止起飞时无法加力，可以防止累计调整过大旋转失控。
//为了防止累加的调节量过大，设置一个阈值。

    float cro_a= new_burdens[0] + new_burdens[3];
    float cro_b= new_burdens[1] + new_burdens[2];

    abnormal=false;

    if(cro_a - cro_b >= CROD && adjyawsum > 0)
    {
        abnormal=true;
    }
    else if(cro_b - cro_a >= CROD && adjyawsum < 0)
    {
        abnormal=true;
    }


//对角负载差异过大，不能再调节了。也不用再检查范围了，上面有检查。
    if(abnormal) {
        new_thrust= vertical_thrust*burden_adj_ratio; //垂直推力修正。
        return;
    }

//第三个角速度是航向角的变化率。增大左旋（逆时针），减小右旋（顺时针）
//从上往下看，桨叶的旋转方向为：
//左前：顺时针，右前：逆时针
//左后：逆时针，右后：顺时针
//目前也是这样安装的桨叶
//要机身顺时针转，加强逆时针旋转的叶片
//要机身逆时针转，加强顺时针旋转的叶片


    ///4.0使得一次调节后对角力差变化adj4sum
    new_burdens[0] += adjyawsum/4.0;//左前
    new_burdens[1] -= adjyawsum/4.0;//右前
    new_burdens[2] -= adjyawsum/4.0;//左后
    new_burdens[3] += adjyawsum/4.0;//右后

//交叉轴调节完成后，同样执行一次限制范围，以确保调整完以后交叉轴的力差不会越界

    cro_a= new_burdens[0] + new_burdens[3]; //顺时针转
    cro_b= new_burdens[1] + new_burdens[2]; //逆时针转

    if(cro_a - cro_b > CROD) {
        float m= (cro_a - cro_b - CROD)/4.0; //要除以4.0，曾经写成2.0，结果调节反向了
        new_burdens[0] -=m;
        new_burdens[3] -=m;
        new_burdens[1] +=m;
        new_burdens[2] +=m;
    }else if(cro_b - cro_a > CROD) {
        float m= (cro_b - cro_a - CROD)/4.0; //要除以4.0，曾经写成2.0，结果调节反向了
        new_burdens[1] -=m;
        new_burdens[2] -=m;
        new_burdens[0] +=m;
        new_burdens[3] +=m;
    }
    
#undef CROD
#undef fstep

    abnormal=adjust_burdens(total_burden, new_burdens, burden_adj_ratio);
    if(abnormal) {
        new_thrust= vertical_thrust*burden_adj_ratio; //垂直推力修正。
    }
}

//仿照gps定点的方法，做一个微任务去往锚点。适合距离目标比较近时的情况。
//这个微任务共用了GPS定点的两个数据存放队列，但没关系，
//这个微任务不会调用GPS定点算法。只使用一般平衡算法。
//虽然可以调节高度，但因为不执行降落检查，所以想降落就切换到垂直降落任务去。
//任务期间，航向角保持不变。必要条件是已经找到方向
//可以在这个任务里降低高度，但并不检查降落，因为任务的done状态用来表示定位成功了。
//需要降落则切换到垂直降落任务，盯住GPS，那边的done状态表示降落成功。
//依赖于指南针的准确性是一个缺陷。
//从一个Pin任务结束到另一个pin任务开始，估计有一定的概率micro_task.status不是none而是done. 
//这是由于主线程和次线程的同步性所引起，主线程会设置状态为none，但次线程可能会保持在上次的任务里并设置done.
//曾经由于自动任务切换为遥控控制时出现低概率的状态保持为done而空中关机，所以推测每个微任务的切换都有可能保持
//上一个微任务的状态。所以需要保证即使是done状态进入到本任务，也不会有麻烦。出现这种情况只是不会掉头。
void CMicroTaskExec::micro_task_pin_to_location()
{
#if defined USE_GPS

    if(micro_task.status==_micro_task_status_none)
    {

        micro_task.roll=0;
        micro_task.pitch=0;
        micro_task.status=_micro_task_status_inprogress;


        //清理掉原始的调节记录，防止误判方向。
        //在一个点转向另一个点时，pin_to_location操作是连续的，但目标点变了，引用老的数据匹配，会造成误判方向。
        for(size_t i=0;i<10;i++) {
            gps_major_bias_queue.last(i).inuse=0;
        }

        //20240304+ 将头部指向目标点。这个任务从宏任务下沉过来。
        gps_info last_gps;
        float dist, angle;
        critical_section_enter_blocking(&gps_queue_section);
        last_gps=gps_queue.last();
        critical_section_exit(&gps_queue_section);

        global_gps_dist_angle(last_gps.latitude, last_gps.longitude, micro_task.target_gps_latitude, micro_task.target_gps_logitude, dist, angle);

        if(dist>30.0)
        {
            //距离较大，掉头
            float yaw= _heading_yaw - angle;

            if(yaw > 180.0) yaw-=360.0;
            if(yaw < -180.0) yaw+=360.0;
            FLOAT_LIMIT(yaw, -180.0, 180.0);

            micro_task.yaw=yaw;
            //micro_task.turn_head_mark=false; //20240304 新增的一个标记，记录抵达目标点后是否执行了掉头。
            micro_task.heading_forward=true;
            //logmessage("pintoloc turn head, dist>30\n");
        }
        else
        {
            //距离很小的不掉头处理。
            //micro_task.turn_head_mark=true;
            micro_task.heading_forward=false;
        }

        micro_task.gps_anchor=true; //20240307+ 加强兼容性。便于平滑过渡到gps_position_adjust
        // char dbg[64];
        // sprintf(dbg, "pin start, hy=%4.2f\n", _heading_yaw);
        // logmessage(dbg);
    }


    float curdist, curspd, curdir, curheight;
    bool newdata;
    bool b= pin_to_location(curdist, curspd, curdir, curheight, newdata);

    if(b) {

        float pitch=gps_major_bias_queue.last().pitch_bias;
        float roll=gps_major_bias_queue.last().roll_bias;

        //执行平衡算法。
        bool landed = false;
        bool hadj=height_adjust_with_landing_detect(INAIR_BOTTHRUST, INAIR_TOPTHRUST, landed);

        if(hadj && landed) {
            //此函数不带降落意图，如果发生降落，则为侧翻。正常不会到这里。
            motor_ctrl.SetBurdens(0,0,0,0,0);
            micro_task.status=_micro_task_status_shutdown;
            return;
        }

        float burdens[4];
        float new_thrust;
        balance(burdens, new_thrust, pitch, roll);
        motor_ctrl.SetBurdens(burdens, new_thrust);

        if(newdata)
        {
            if(curdist <= micro_task.target_distance_tolerance && curspd < 0.8)
            {
                //掉头处理。
                if(micro_task.heading_forward) {

                    //20240720 新掉头办法。
                    gps_info last_gps;
                    float dist, angle;
                    critical_section_enter_blocking(&gps_queue_section);
                    last_gps=gps_queue.last();
                    critical_section_exit(&gps_queue_section);

                    global_gps_dist_angle(last_gps.latitude, last_gps.longitude, micro_task.start_gps_latitude, micro_task.start_gps_logitude, dist, angle);

                    float yaw= _heading_yaw - angle;
                    if(yaw > 180) yaw-=360;
                    if(yaw < -180) yaw+=360;

                    micro_task.yaw=yaw;

                    // char buf[64];
                    // sprintf(buf,"turn head, hy=%4.1f, dest angle=%4.1f, set yaw=%4.1f\n", _heading_yaw, angle, yaw);
                    // logmessage(buf);
                }

                micro_task.status=_micro_task_status_done;  //距离很近时标识出，宏任务可继续维持也可打断。
            }
            else
            {
                micro_task.status=_micro_task_status_inprogress; 
            }
        }


    }else{

        //GPS丢失信号。
        bool landed=false;
        bool hadj=height_adjust_with_landing_detect(INAIR_BOTTHRUST, INAIR_TOPTHRUST, landed);

        if(hadj && landed) {
            //此函数不带降落意图，如果发生降落，则为侧翻。
            motor_ctrl.SetBurdens(0,0,0,0,0);
            micro_task.status=_micro_task_status_shutdown;
        }
        else
        {
            //为防止漂移可以采用光流定点，等待GPS信号。
            float burdens[4];
            float new_thrust;
            balance(burdens, new_thrust, 0, 0);
            motor_ctrl.SetBurdens(burdens, new_thrust);
        }
        
    }

#else
    //执行平衡算法。
    //height_adjust(INAIR_BOTTHRUST, INAIR_TOPTHRUST);
    bool landed;
    height_adjust_with_landing_detect(INAIR_BOTTHRUST, INAIR_TOPTHRUST, landed);

    float burden[4]; 
    float new_thrust;
    balance(burden, new_thrust, 0, 0);
    motor_ctrl.SetBurdens(burden, new_thrust);
    micro_task.status=_micro_task_status_fail;
#endif
}

//爬高时的电流代价及速度限制
//cur_vspd: 当前垂直速度，负值为向上， m/s
//cur_current: 当前电流
//current_grow: 电流增加值
//ref_current: 上一个参考电流
//ref_vspd_limit: 上一个垂直速度参考值做输入，输出是最大上行速度限制
//此函数为气压控高的辅助函数，不在测距控高使用。
//float CMicroTaskExec::PowerCostForClimbing(float cur_current, int psolution, float ref_current, float& ref_vspd_limit)
float CMicroTaskExec::PowerCostForClimbing(int psolution, float ref_current, float ref_vspd)
{
    float vspd_limit=0;

    if(psolution==0)
    {
        vspd_limit=-12.0; //爬高速度无限制，这里设置为 12m/s的值。
    }
    else if(psolution==1)
    {

        if(ref_vspd >= 0) {
            //参考速度向下，则没有参考意义。
            vspd_limit=-1.0;
        }
        else if(ref_current > 10.0) {
            //触碰最大电流值。限制最大爬升速度。
            vspd_limit= ref_vspd*10.0/ref_current;
        }
        else if(ref_current > 9.0) {
            vspd_limit= ref_vspd*1.02;
        }else if(ref_current > 8.0) {
            vspd_limit= ref_vspd*1.04;
        }else if(ref_current > 7.0) {
            vspd_limit= ref_vspd*1.06;
        }else if(ref_current > 6.0) {
            vspd_limit= ref_vspd*1.08;
        }else{
            vspd_limit= ref_vspd*1.1;
        }

        if(ref_vspd > -1.0) vspd_limit= -1.0; //保证1m/s的速度
    }
    else if(psolution==2)
    {

        if(ref_vspd >= 0) {
            //参考速度向下，则没有参考意义。
            vspd_limit=-1.0;
        }
        else if(ref_current > 12.0) {
            //触碰最大电流值。限制最大爬升速度。
            vspd_limit= ref_vspd*12.0/ref_current;
        }else if(ref_current > 11.0) {
            vspd_limit=ref_vspd*1.02;
        }else if(ref_current > 10.0) {
            vspd_limit=ref_vspd*1.04;
        }else if(ref_current > 9.0) {
            vspd_limit=ref_vspd*1.06;
        }else if(ref_current > 8.0) {
            vspd_limit=ref_vspd*1.08;
        }else{
            vspd_limit=ref_vspd*1.1;
        }

        if(vspd_limit> -1.0) vspd_limit= -1.0;
    }
    else if(psolution==3)
    {

        if(ref_vspd >= 0) {
            //参考速度向下，则没有参考意义。
            vspd_limit=-1.0;
        }
        else if(ref_current > 14.0) {
            //触碰最大电流值。限制最大爬升速度。
            vspd_limit= ref_vspd*14.0/ref_current;
        }else if(ref_current > 13.0) {
            vspd_limit= ref_vspd*1.02;
        }else if(ref_current > 12.0) {
            vspd_limit= ref_vspd*1.04;
        }else if(ref_current > 11.0) {
            vspd_limit= ref_vspd*1.06;
        }else if(ref_current > 10.0) {
            vspd_limit= ref_vspd*1.08;
        }else{
            vspd_limit= ref_vspd*1.1;
        }

        if(vspd_limit> -1.0) vspd_limit= -1.0;
    }
    else if(psolution==4)
    {

        if(ref_vspd >= 0) {
            //参考速度向下，则没有参考意义。
            vspd_limit=-1.0;
        }
        else if(ref_current > 16.0) {
            //触碰最大电流值。限制最大爬升速度。
            vspd_limit= ref_vspd*16.0/ref_current;
        }else if(ref_current > 15.0) {
            vspd_limit= ref_vspd*1.02;
        }else if(ref_current > 14.0) {
            vspd_limit= ref_vspd*1.04;
        }else if(ref_current > 13.0) {
            vspd_limit= ref_vspd*1.06;
        }else if(ref_current > 12.0) {
            vspd_limit= ref_vspd*1.08;
        }else{
            vspd_limit= ref_vspd*1.1;
        }

        if(vspd_limit> -1.0) vspd_limit= -1.0;
    }
    else if(psolution==5)
    {

        if(ref_vspd>= 0) {
            //参考速度向下，则没有参考意义。
            vspd_limit=-1.0;
        }
        else if(ref_current > 18.0) {
            //触碰最大电流值。限制最大爬升速度。
            vspd_limit= ref_vspd*18.0/ref_current;
        }else if(ref_current > 17.0) {
            vspd_limit=ref_vspd*1.02;
        }else if(ref_current > 16.0) {
            vspd_limit=ref_vspd*1.04;
        }else if(ref_current > 15.0) {
            vspd_limit=ref_vspd*1.06;
        }else if(ref_current > 14.0) {
            vspd_limit=ref_vspd*1.08;
        }else{
            vspd_limit=ref_vspd*1.1;
        }

        if(vspd_limit> -1.0) vspd_limit= -1.0;
    }
    else if(psolution==6)
    {

        if(ref_vspd >= 0) {
            //参考速度向下，则没有参考意义。
            vspd_limit=-1.0;
        }
        else if(ref_current > 20.0) {
            //触碰最大电流值。限制最大爬升速度。
            vspd_limit= ref_vspd*20.0/ref_current;
        }else if(ref_current > 19.0) {
            vspd_limit= ref_vspd*1.02;
        }else if(ref_current > 18.0) {
            vspd_limit= ref_vspd*1.04;
        }else if(ref_current > 17.0) {
            vspd_limit= ref_vspd*1.06;
        }else if(ref_current > 16.0) {
            vspd_limit= ref_vspd*1.08;
        }else{
            vspd_limit= ref_vspd*1.1;
        }

        if(vspd_limit> -1.0) vspd_limit= -1.0;
    }

    return vspd_limit;

}

//电力代价函数，用来控制水平飞行，低速时代价低，高速时代价高
//当前速度，电流，电流增量，电力方案（0-6)
//返回值越大，越要抑制速度，限制电流消耗，返回值（0-1) 越小值越可以大加速
//ref_current是之前参考电流值，ref_mod_limit是之前参考倾角值，当电流过大时用来调节最大倾角。
//ref_mod_limit是传入值，也是传出修正值，硬性最大倾角限制。
//20250331修改，返回值可以大于1.0
float CMicroTaskExec::PowerCostForPin(float cur_spd, float current, float current_grow, int psolution, float ref_current, float& ref_mod_limit)
{

    float curr =0; //电流代价因子
    if(psolution==0)
    {
        curr=0;
        ref_mod_limit*=1.08;
    }
    else if(psolution==1)
    {
        if (current > 10.0) {
            curr = current/10.0; 
        }
        else if (current > 9.0) curr = 0.7 + (current - 9.0)*0.3; //0.7-1.0
        else if (current > 8.0) curr = 0.5 + (current - 8.0)*0.2; //0.5-0.7
        else if (current > 7.0) curr = 0.35 + (current - 7.0)*0.15; //0.35-0.5
        else if (current > 6.0) curr = 0.25 + (current - 6.0)*0.1; //0.25-0.35
        else if (current > 5.0) curr = 0.15 + (current - 5.0)*0.1; //0.15-0.25
        else if (current > 3.0) curr = 0.0 + (current - 3.0)*0.075; //0.0-0.15
        else curr = 0;

        if(ref_current > 10.0) {
            //触碰最大电流值。限制最大倾角。
            ref_mod_limit*= 10.0/ref_current;
        }
        else if(ref_current > 9.0) {
            ref_mod_limit*=1.02;
        }else if(ref_current > 8.0) {
            ref_mod_limit*=1.04;
        }else if(ref_current > 7.0) {
            ref_mod_limit*=1.06;
        }else if(ref_current > 6.0) {
            ref_mod_limit*=1.08;
        }else{
            ref_mod_limit*=1.1;
        }

        if(ref_mod_limit<0.2679) ref_mod_limit=0.2679;
    }
    else if(psolution==2)
    {
        if (current > 12.0) curr = current/12.0; //1.0
        else if (current > 11.0) curr = 0.7 + (current - 11.0)*0.3; //0.7-1.0
        else if (current > 10.0) curr = 0.5 + (current - 10.0)*0.2; //0.5-0.7
        else if (current > 9.0) curr = 0.35 + (current - 9.0)*0.15; //0.35-0.5
        else if (current > 8.0) curr = 0.25 + (current - 8.0)*0.1; //0.25-0.35
        else if (current > 7.0) curr = 0.15 + (current - 7.0)*0.1; //0.15-0.25
        else if (current > 5.0) curr = 0.0 + (current - 5.0)*0.075; //0.0-0.15
        else curr = 0;

        if(ref_current > 12.0) {
            //触碰最大电流值。限制最大倾角。
            ref_mod_limit*= 12.0/ref_current;
        }else if(ref_current > 10.0) {
            ref_mod_limit*=1.02;
        }else if(ref_current > 8.0) {
            ref_mod_limit*=1.04;
        }else if(ref_current > 7.0) {
            ref_mod_limit*=1.06;
        }else if(ref_current > 6.0) {
            ref_mod_limit*=1.08;
        }else{
            ref_mod_limit*=1.1;
        }

        if(ref_mod_limit<0.2679) ref_mod_limit=0.2679;
    }
    else if(psolution==3)
    {
        if (current > 14.0) curr = current/14.0; //1.0
        else if (current > 13.0) curr = 0.7 + (current - 13.0)*0.3; //0.7-1.0
        else if (current > 12.0) curr = 0.5 + (current - 12.0)*0.2; //0.5-0.7
        else if (current > 11.0) curr = 0.35 + (current - 11.0)*0.15; //0.35-0.5
        else if (current > 10.0) curr = 0.25 + (current - 10.0)*0.1; //0.25-0.35
        else if (current > 9.0) curr = 0.15 + (current - 9.0)*0.1; //0.15-0.25
        else if (current > 7.0) curr = 0.0 + (current - 7.0)*0.075; //0.0-0.15
        else curr = 0;

        if(ref_current > 14.0) {
            //触碰最大电流值。限制最大倾角。
            ref_mod_limit*= 14.0/ref_current;
        }else if(ref_current > 12.0) {
            ref_mod_limit*=1.02;
        }else if(ref_current > 10.0) {
            ref_mod_limit*=1.04;
        }else if(ref_current > 8.0) {
            ref_mod_limit*=1.06;
        }else if(ref_current > 7.0) {
            ref_mod_limit*=1.08;
        }else{
            ref_mod_limit*=1.1;
        }

        if(ref_mod_limit<0.2679) ref_mod_limit=0.2679;
    }
    else if(psolution==4)
    {
        if (current > 16.0) curr = current/16.0; //1.0
        else if (current > 15.0) curr = 0.7 + (current - 15.0)*0.3; //0.7-1.0
        else if (current > 14.0) curr = 0.5 + (current - 14.0)*0.2; //0.5-0.7
        else if (current > 13.0) curr = 0.35 + (current - 13.0)*0.15; //0.35-0.5
        else if (current > 12.0) curr = 0.25 + (current - 12.0)*0.1; //0.25-0.35
        else if (current > 11.0) curr = 0.15 + (current - 11.0)*0.1; //0.15-0.25
        else if (current > 9.0) curr = 0.0 + (current - 9.0)*0.075; //0.0-0.15
        else curr = 0;

        if(ref_current > 16.0) {
            //触碰最大电流值。限制最大倾角。
            ref_mod_limit*= 16.0/ref_current;
        }else if(ref_current > 14.0) {
            ref_mod_limit*=1.02;
        }else if(ref_current > 12.0) {
            ref_mod_limit*=1.04;
        }else if(ref_current > 10.0) {
            ref_mod_limit*=1.06;
        }else if(ref_current > 8.0) {
            ref_mod_limit*=1.08;
        }else{
            ref_mod_limit*=1.1;
        }

        if(ref_mod_limit<0.2679) ref_mod_limit=0.2679;
    }
    else if(psolution==5)
    {
        if (current > 18.0) curr = current/18.0; 
        else if (current > 17.0) curr = 0.7 + (current - 17.0)*0.3; //0.7-1.0
        else if (current > 16.0) curr = 0.5 + (current - 16.0)*0.2; //0.5-0.7
        else if (current > 15.0) curr = 0.35 + (current - 15.0)*0.15; //0.35-0.5
        else if (current > 14.0) curr = 0.25 + (current - 14.0)*0.1; //0.25-0.35
        else if (current > 13.0) curr = 0.15 + (current - 13.0)*0.1; //0.15-0.25
        else if (current > 11.0) curr = 0.0 + (current - 11.0)*0.075; //0.0-0.15
        else curr = 0;

        if(ref_current > 18.0) {
            //触碰最大电流值。限制最大倾角。
            ref_mod_limit*= 18.0/ref_current;
        }else if(ref_current > 16.0) {
            ref_mod_limit*=1.02;
        }else if(ref_current > 14.0) {
            ref_mod_limit*=1.04;
        }else if(ref_current > 12.0) {
            ref_mod_limit*=1.06;
        }else if(ref_current > 10.0) {
            ref_mod_limit*=1.08;
        }else{
            ref_mod_limit*=1.1;
        }

        if(ref_mod_limit<0.2679) ref_mod_limit=0.2679;
    }
    else if(psolution==6)
    {
        if (current > 20.0) curr = current/20.0; //1.0
        else if (current > 19.0) curr = 0.7 + (current - 19.0)*0.3; //0.7-1.0
        else if (current > 18.0) curr = 0.5 + (current - 18.0)*0.2; //0.5-0.7
        else if (current > 17.0) curr = 0.35 + (current - 17.0)*0.15; //0.35-0.5
        else if (current > 16.0) curr = 0.25 + (current - 16.0)*0.1; //0.25-0.35
        else if (current > 15.0) curr = 0.15 + (current - 15.0)*0.1; //0.15-0.25
        else if (current > 13.0) curr = 0.0 + (current - 13.0)*0.075; //0.0-0.15
        else curr = 0;

        if(ref_current > 20.0) {
            //触碰最大电流值。限制最大倾角。
            ref_mod_limit*= 20.0/ref_current;
        }else if(ref_current > 18.0) {
            ref_mod_limit*=1.02;
        }else if(ref_current > 16.0) {
            ref_mod_limit*=1.04;
        }else if(ref_current > 14.0) {
            ref_mod_limit*=1.06;
        }else if(ref_current > 12.0) {
            ref_mod_limit*=1.08;
        }else{
            ref_mod_limit*=1.1;
        }

        if(ref_mod_limit<0.2679) ref_mod_limit=0.2679;
    }

    if (cur_spd < 1.0) curr *= 0.93;
	else if (cur_spd < 3.0) curr *= 0.95;
	else if (cur_spd < 5.0) curr *= 0.97;
	else if (cur_spd < 7.0) curr *= 0.99;

	if (current_grow < -2.0) curr *= 0.92;
	else if (current_grow < -1.0) curr *= 0.95;
	else if (current_grow < -0.5) curr *= 0.97;
	else if (current_grow < -0.2) curr *= 0.99;

    FLOAT_LIMIT(curr, 0.0, 1.5); //必须在这个范围内。
    return curr;
}

//近距离的盯住某个点位。当盯住一个很远的目标点位时，短时间不易探测到速度围绕这个点的一个方向的改变
//于是较难处理方向的偏离问题。可能不能够对准目标点运动。
//给出了调节值就返回true,否则返回false. 只在GPS丢失信号时/定位精度极差时返回false
//新的代码基于新写的gps_position_adjust, 准备取代老的pin_to_location, 老代码处理太复杂。
//给出的调节值也暂时放在微调节队列里。虽然没有微调节。
//20230725+ 指南针处理新增一个概念，锚定时做时间标记，这样不会使用无效的数据判定方向。
//这样的情况下，首次调用这个函数时，要标记micro_task.last_gps_anchor_tmark.
//如果换一个目标点，又需要重新标记这个时间戳。这个记录由调用此函数的上层去处理。先标记时间戳，然后持续调用这个函数。
//另外新修改的地方还有根据距离的不同，检测速度方向的跨度不同，理论上这个函数可以用来盯住长距离目标了。
//调用这个函数时，上层必须更新航向偏离角micro_task.voyage_bias，如果不更新，必须置零。
//20230725 新考虑，目前这个用来一键返航，速度有点太慢了。
//考虑距离情况，在不同的距离下限制不同的速度。
//20230730目前返航速度可以。到达目标点后似乎有摆动，盯住目标点的能力似乎不如GPS定点函数。
//newdata:基于一个新数据计算的数。
//20230926,新增返回一个GPS高度。
//20231002的测试表明，一键返航可能存在问题。当方向差别很大时，系统无法按照原算法来纠正方向，系统可能飞向目标点的反向。这和yaw角的漂移有关。
//在高空中温度差异很大，yaw角可能漂移很大，此时需要指南针来校准。
//另一种可能是，当时电机发力几乎满了，而且在上升中，不一定有能力来处理倾角问题，那样当时的速度方向不对就是被风吹的，而不是因为方向判断有问题。
//在几乎满力状态下，当前代码是优先保证高度，还是平衡呢？这是个问题，需要查看分析代码。
bool CMicroTaskExec::pin_to_location(float& cur_dist, float& cur_spd, float& cur_dir, float& cur_height, bool& newdata)
{

#ifdef USE_GPS

    uint32_t now=get_time_mark();
    uint32_t dtime;
    gps_info g0,g1,g2,g3,g4; //多取几个数据是为了更准确判断加速度，因为速度没有滤波

    critical_section_enter_blocking(&gps_queue_section);
    g0=gps_queue.last(0);
    g1=gps_queue.last(1);
    g2=gps_queue.last(2);
    g3=gps_queue.last(3);
    g4=gps_queue.last(4);
    critical_section_exit(&gps_queue_section);

    dtime=g0.tmark;

    if(now - dtime > 700) {

        //做一个GPS干扰标记。
        if(!micro_task.gps_bad_mark) {
            micro_task.gps_bad_mark=true; //标记GPS信号中断。
            logmessage("lost gps signal in pin-loc!\n");
        }

        //20231001 试验中遇到许多GPS干扰中断情况，新增处理代码。
        //原来的处理方式是直接姿态回正，剩余的交给惯性和风力。
        //新的方式是，考察到目标的距离，如果距离较远，在一定时间限制内，保持原有的姿态，
        //超过时间限制时，姿态回正，剩余交给惯性和风力。这样可以避免恶意的GPS干扰立即看见干扰效果。

        newdata=false;

        //不同的信号丢失时间和到目标的距离，不同的处理方法。
        if(now - dtime < 2500)
        {
            //上一个GPS信号不超过2.5秒，考虑维持之前的姿态。
            //要考察和目标点的距离，如果距离很大，则维持之前的姿态角度不变
            float dist, angle;
            global_gps_dist_angle(g0.latitude, g0.longitude, micro_task.target_gps_latitude, micro_task.target_gps_logitude, dist, angle);
            
            if(dist>20.0) {
                //信号丢失时，距离目标大于20米，维持之前的状态不变。
                gps_major_bias mb=gps_major_bias_queue.last();
                mb.gps_tmark=dtime;
                gps_major_bias_queue.push(mb);
                return true;

            }else{
                gps_major_bias majorb;
                majorb.gps_tmark=dtime;
                gps_major_bias_queue.push(majorb);
                return false;
            }  
            
        }
        else if(now - dtime < 5000)
        {
            //上一个GPS信号不超过5秒，考虑维持之前的姿态。
            //要考察和目标点的距离，如果距离很大，则维持之前的姿态角度不变
            float dist, angle;
            global_gps_dist_angle(g0.latitude, g0.longitude, micro_task.target_gps_latitude, micro_task.target_gps_logitude, dist, angle);
            
            if(dist>60.0) {
                //信号丢失时，距离目标大于60米，维持之前的状态不变。
                gps_major_bias mb=gps_major_bias_queue.last();
                mb.gps_tmark=dtime;
                gps_major_bias_queue.push(mb);
                return true;

            }else{
                gps_major_bias majorb;
                majorb.gps_tmark=dtime;
                gps_major_bias_queue.push(majorb);
                return false;
            }
            
        }
        else if(now - dtime < 8000)
        {
            //上一个GPS信号不超过8秒，考虑维持之前的姿态。
            //要考察和目标点的距离，如果距离很大，则维持之前的姿态角度不变
            float dist, angle;
            global_gps_dist_angle(g0.latitude, g0.longitude, micro_task.target_gps_latitude, micro_task.target_gps_logitude, dist, angle);
            
            if(dist>100.0) {
                //信号丢失时，距离目标大于100米，维持之前的状态不变。
                gps_major_bias mb=gps_major_bias_queue.last();
                mb.gps_tmark=dtime;
                gps_major_bias_queue.push(mb);
                return true;

            }else{
                gps_major_bias majorb;
                majorb.gps_tmark=dtime;
                gps_major_bias_queue.push(majorb);
                return false;
            } 
        }
        else
        {
            //gps超时太长了，超过了8秒，姿态回正。
            gps_major_bias majorb;
            majorb.gps_tmark=dtime;
            gps_major_bias_queue.push(majorb);
            return false;
        }

        return false;

    }else if(micro_task.gps_bad_mark){
        //如果标记了坏的信号，清除标记
        micro_task.gps_bad_mark=false;
        logmessage("gps resume in pin-loc\n");

    }

    if(dtime == gps_major_bias_queue.last().gps_tmark) {
        //信号间微调位置。似乎没有也还行。
        newdata=false;
        return true;
    }


    //这里取不同的间隔是为了更准确的加速度
    uint32_t tgap2 = g0.tmark - g2.tmark;
    float t2_inverse= 1000.0/float(tgap2);
    uint32_t tgap3 = g0.tmark - g3.tmark;
    float t3_inverse= 1000.0/float(tgap3);
    uint32_t tgap4 = g0.tmark - g4.tmark;
    float t4_inverse= 1000.0/float(tgap4);

    //当前位置，用来求距离。现在坐标有滤波 20230806+
    //因为取消了滤波，所以这里增加平均值采样。20240105
    double lati0= (g0.latitude+g1.latitude+g2.latitude+g3.latitude)/4.0;
    double logi0= (g0.longitude+g1.longitude+g2.longitude+g3.longitude)/4.0;

    //最近速度。
    //20240106修改为3组数据的平均值。
    float spdEast1= (g0.speed_east+g1.speed_east+g2.speed_east)/3.0; //标准单位
    float spdNorth1= (g0.speed_north+g1.speed_north+g2.speed_north)/3.0;
    float spd1= (g0.speed+g1.speed+g2.speed)/3.0; //模量速度 m/s。


    //加速度
    float accEast2 = (g0.speed_east- g2.speed_east)*t2_inverse; //标准单位,m/s/s
    float accNorth2= (g0.speed_north-g2.speed_north)*t2_inverse;
    float accEast3 = (g0.speed_east- g3.speed_east)*t3_inverse; //标准单位,m/s/s
    float accNorth3= (g0.speed_north-g3.speed_north)*t3_inverse;
    float accEast4 = (g0.speed_east- g4.speed_east)*t4_inverse; //标准单位,m/s/s
    float accNorth4= (g0.speed_north-g4.speed_north)*t4_inverse;


    //2个加速度等权重处理
    float accEast= (accEast2+accEast3)/2.0;
    float accNorth= (accNorth2+accNorth3)/2.0;

    //稍早的加速度。用于差分调节。
    float accEast_long= (accEast3+accEast4)/2.0;
    float accNorth_long= (accNorth3+accNorth4)/2.0;

    FLOAT_LIMIT(accEast, -5.0, 5.0);
    FLOAT_LIMIT(accNorth, -5.0, 5.0);
    FLOAT_LIMIT(accEast_long, -5.0, 5.0);
    FLOAT_LIMIT(accNorth_long, -5.0, 5.0);

    //到锚定点的两个距离。
    float dist=0;
    float angle=0;
    float distEast=0;
    float distNorth=0;

    //锚点是微任务里的目标点。属于必有锚点。
    global_gps_dist_angle(lati0, logi0, micro_task.target_gps_latitude, micro_task.target_gps_logitude, dist, angle);

    //三个夹角，间隔2个数据，可以更准确判断弧线内卷或外卷
    float spd_angle0=0; //当前速度和到目标连线的夹角
    float spd_angle1=0; //速度夹角
    float spd_angle2=0; //速度夹角
    float spd_angle3=0; //速度夹角

    //现在是当前速度方向和期望方向的夹角。区分顺逆旋转方向。
    spd_angle0= angle - g0.direction; //>0 目标点在速度方向的右侧， <0 目标点在速度方向的左侧。
    if(spd_angle0>180) spd_angle0-=360;
    if(spd_angle0<-180) spd_angle0+=360;

    spd_angle1= angle - g1.direction; //>0 目标点在速度方向的右侧， <0 目标点在速度方向的左侧。
    if(spd_angle1>180) spd_angle1-=360;
    if(spd_angle1<-180) spd_angle1+=360;

    spd_angle2= angle - g2.direction; //>0 目标点在速度方向的右侧， <0 目标点在速度方向的左侧。
    if(spd_angle2>180) spd_angle2-=360;
    if(spd_angle2<-180) spd_angle2+=360;

    spd_angle3= angle - g3.direction; //>0 目标点在速度方向的右侧， <0 目标点在速度方向的左侧。
    if(spd_angle3>180) spd_angle3-=360;
    if(spd_angle3<-180) spd_angle3+=360;


    //返回四个当前数据。
    cur_dist=dist; //当前到目标的距离
    cur_dir=g0.direction; //当前速度方向
    cur_spd=g0.speed; //当前速度
    cur_height=g0.height; //当前高度

    //距离信度调节，如果距离很近，大概率是漂移误差，可信度低，如果距离较远，则可信度高。

    distEast= dist*sin(angle/57.2958);
    distNorth= dist*cos(angle/57.2958);


    float sr= gps_speed_reliable(spd1);

    spdEast1*=sr;
    spdNorth1*=sr;


    //imu水平加速度噪音可能较大，取多个数据平均。做阻尼用。
    float imu_accx=0;
    float imu_accy=0; //y方向和光流方向相反
    float cur_yaw;
    float cur_pitch;//20240706+ 当前倾角
    float cur_roll;
    //20240123 修改为100个数据的均值。之前是50个
    //改回50个数据，20240305
    critical_section_enter_blocking(&imu_queue_section);
    cur_pitch= imu_queue.last().angle[0];
    cur_roll= imu_queue.last().angle[1];
    cur_yaw= imu_queue.last().angle[2];
    for(size_t i=0; i<50; i++)
    {
        imu_accx+= imu_queue.last(i).acc_gnd[0];
        imu_accy+= imu_queue.last(i).acc_gnd[1];
    }
    critical_section_exit(&imu_queue_section);

    imu_accx/=5.0; //求均值，转标准单位
    imu_accy/=5.0; 

    float heading= _heading_yaw - cur_yaw;
    if(heading<0) heading+=360;
    if(heading>360) heading-=360;
    heading/=57.2958;

    float cosheading= cos(heading);
    float sinheading= sin(heading);

    //机身坐标的加速度信息转到世界坐标
    float imu_waccx, imu_waccy;
    Flight2World(cosheading, sinheading, imu_accx, imu_accy, imu_waccx, imu_waccy);

    FLOAT_LIMIT(imu_waccx, -5.0, 5.0); //同GPS加速度限制范围
    FLOAT_LIMIT(imu_waccy, -5.0, 5.0);

    //将这个实际倾角转到世界坐标下，应该也可以当成是参考推力，但现在暂时不改动。
    float flight_cur_castx= tan(cur_roll/57.2958);
    float flight_cur_casty= -tan(cur_pitch/57.2958); //pitch是负值时，推力是正向的，要加负号

    //取参考调节位置，从主调节序列里提取。
    //使用参考姿态容易受到平衡函数的影响，因为参考姿态可能和实际姿态有时间差，如果姿态的执行速度偏低，则调节可能引起震荡。
    float ref_castx=0;
    float ref_casty=0;
    float ref_current=0; //顺便计算平滑电流 20240327+
    float ref_spd=0; //20240418+
    //引用多个历史数据平滑处理
    for(size_t i=0;i<3;i++)
    {
        ref_castx+= gps_major_bias_queue.last(i).world_castx;
        ref_casty+= gps_major_bias_queue.last(i).world_casty;
        ref_current+= gps_major_bias_queue.last(i).current;
        ref_spd += gps_major_bias_queue.last(i).gps_spd;
    }

    ref_castx/=3;
    ref_casty/=3;
    ref_current/=3;
    ref_spd/=3;

    float gps_quality=GpsQuality(g0.ns, g0.hacc); //信号质量调节系数。20240109+

    //_heading_yaw更新。和定点统一为一种形式。

        //统一的调节 20240304，1，航向调节，2，弧线调节，3.加速度方向和速度方向的夹角，4.加速度匹配机身姿态
        //在近距离环绕时，由于震荡，机器远离目标点，那么这个夹角会很大，比如说-180， 180，而不是一个很小的锐角。
        //此时的调节应该朝着180度方向去调整。
        // float dr2=1.0; //方向角度的信度，距离很近时，方向角偏差大，可靠性不高。
        // if(dist<5*g0.hacc) dr2=0.1;
        // else if(dist<10*g0.hacc) dr2=0.2;
        // else if(dist<15*g0.hacc) dr2=0.35;
        // else if(dist<20*g0.hacc) dr2=0.5;
        // else if(dist<30*g0.hacc) dr2=0.65;
        // else if(dist<40*g0.hacc) dr2=0.8;
        // else if(dist<50*g0.hacc) dr2=0.95;

        // float sr2=1.0;
        // if(spd1 <0.2) sr2=0.05;
        // else if(spd1 < 0.5) sr2=0.1;
        // else if(spd1 < 1.0) sr2=0.2;
        // else if(spd1 < 2.0) sr2=0.5;
        // else if(spd1 < 4.0) sr2=0.7;
        // else if(spd1 < 5.0) sr2=0.9;


    //加速度方向和速度方向的夹角，越是接近0的越不可靠。可靠的时候加速度方向应和速度方向相反。
    //这里利用了天然的定点特性，即抑制一切速度。所以在平飞时不能用这个方法分辨方向。
    //20240404+
    //20240717 这个目前纠偏力度占比最大。
    //如果距离目标点很远，这个调节就很可疑，因为加速度可能会指向目标点，和速度同向。
    //20240717 增加随距离衰减dr3, 近距离作用大，远距离作用小。
    if(gps_major_bias_queue.last(6).inuse==3)
    {
        //现在在定点中，速度方向和加速度方向夹角调节的作用仅次于最后一种。风中似乎无偏。

        //两种方法，一种是速度方向和目标方向的夹角调节。远距离作用大，近距离作用小
        //另一种是速度方向和加速度方向夹角调节。远距离作用小，近距离作用大

        //加速度方向和目标方向的夹角，如果不限制一个偏差范围，则在侧风波动下可能是不对称的调节。
        //比如右前侧风加强，则加速度是-135度方向，右前侧风减弱，加速度是45度方向。多次反复后总体调节量就偏离了0.
        //所以限制每次调节在一个小角度是必要的。
        //如何利用好加速度方向和目标方向的夹角是个难题。
        //在最初加速段，其匹配目标方向可以很好的校准方向。但路途中的风力很难处理。风力可以推动加速度朝向任何方向，
        //也可能是某个方向及其反向，比如某方向的恒风，大小改变，则造成的加速度是180度对称的。

        float cur_acc_ang= atan2(accEast, accNorth)*57.2958; //20240712+ 忘记了*57.2958。
        float angle_trans=angle; //angle (0-360) angle_trans(-180, 180) 把范围调节成相同的。
        if(angle>=180.0) angle_trans=angle - 360.0;

        float acc_dir = angle_trans - cur_acc_ang; //目标角和加速度角的夹角。
        if(acc_dir>180.0) acc_dir-=360.0;
        if(acc_dir<-180.0) acc_dir+=360.0;

        float accnorm= sqrt(accEast*accEast+accNorth*accNorth);
        float accr=gps_speed_reliable(accnorm); //加速度信度，采用速度信度
        float acc_vert= sin(acc_dir/57.2958)*accnorm; //垂直于路径方向的加速度分量。>0 指向左侧，<0 指向右侧。
        //float acc_para= cos(acc_dir/57.2958)*accnorm; //平行路径方向的加速度分量。 >0 指向目标点。
        //沿着目标方向的加速度可能指向目标，也可能反向，不调节。


        //20240727 将曲线旋转调节移动到这里。并区分加速减速
        float dc1= g0.direction - g1.direction; //(-180,180) >0曲线右旋。

        if(dc1>180.0) dc1-= 360.0;
        if(dc1<-180.0) dc1+= 360.0;
        FLOAT_LIMIT(dc1, -10.0, 10.0); //修改限制到3.0度，避免强风造成剧烈影响。20241124+

        //新增距离调节项 20241124+，距离越远调节越强。
        float distadj=1.0; 
        if(dist>500.0) distadj=1.0;
        else if(dist>100.0) distadj=0.9;
        else if(dist>30.0) distadj=0.8;
        else if(dist>15.0) distadj=0.7;
        else if(dist>6.0) distadj=0.6;
        else distadj=0.5; 

        float adj0= 0.10*dc1*distadj; 
        _heading_yaw -= adj0; 


        //不限制方向角度的调节，防止偏调。但希望180度对称的加速度在调节上互相抵消。
        //如果希望所有180度对称反向的加速度在调节上都互相抵消，则可能不能使用目标角。
        //可以分解加速度，一个沿指向目标点的连线的投影，一个是垂直投影，分别调节。那么对称的加速度可以互相抵消。
        //20240718 新思路，将加速度沿着目标指向方向分解为平行和垂直分量。
        //只适合加速度段和稳定平分段调节，适合远离目标点使用。

        //目前认为，垂直加速度的调节是加减速一致的，所以20米外不限制使用，20米内线性递减。
        //之前一直避免在减速段使用这个调节 20241126
        float distadj2=1.0;
        if(dist<5.0) {
            distadj2= dist/5.0;
        }

        //20241124+ 这里调节逻辑和圆弧环绕一样的，担心容易受侧风影响，减小调节力度。0.15->0.12
        //20241126 垂直方向加速度调节似乎是加减速一致的，如目标偏右侧，加速时会有偏右侧的加速度，hy减小，
        //减速时有偏左侧的加速度，hy也减小。所以可以考虑把限制在80米外使用去掉。减速段使用应该没有问题。

        FLOAT_LIMIT(acc_vert, -0.5, 0.5); //限制特别大的值，避免强风下的大加速度造成巨大影响。20241124+
        float adj = 0.15*acc_vert*accr*distadj2;
        _heading_yaw += adj; 

    }

//gps加速度匹配机身姿态变化。 
//测试表明，似乎这个调节是有利的
//20240727 带摄像头测试，似乎显示在逆风飞行时，方向角有持续增大趋势。
    //逆侧风飞行时，由风力变化造成的加速度在机身推力上找不到原因，会造成持续的偏调。
    //在找机身原因时，需要有一个可信度。机身推力变化很小的，可信度很低。可认为是外界干扰。
    //在调节时，多乘一个force_chg_norm，这样当力度变化很小时，调节力大幅减弱。
    //可能要考虑加强这个调节，目前在平稳飞行时，偏差的角度很难纠正，因为平稳飞行时加减速的机会不多，只在两头纠正量较大，但常常也不够。
    //为了利用途中细微的加速度变化纠正偏角，加强系数并限制最大调节角可能是一个办法。需要在长距离飞行中让方向角偏差越来越小。
    //现系数调节为0.3，比原来的0.15增加1倍，夹角差限制为3.0度，之前为5.0
    {

        float spd_east_chg= gps_major_bias_queue.last(1).gps_spdEast - gps_major_bias_queue.last(6).gps_spdEast;
        float spd_north_chg= gps_major_bias_queue.last(1).gps_spdNorth - gps_major_bias_queue.last(6).gps_spdNorth;

        //在有锚点的时候，主动调节产生的速度变化总是指向锚点，要么就是在刹车时反向指向锚点。垂直方向的调节应该都是风力影响产生的。
        //刹车时的调节难度较大，这种逆向加速也可能是顶风风力产生的，也可能是近距离刹车，也可能是电流限制刹车。
        //有可能会发生调节错误，所以暂时过滤掉逆向加速的调节，即如果速度变化方向大体指向锚点的就调节，偏差较大的不调节。
        float gps_spd_chg_ang= atan2(spd_east_chg, spd_north_chg)*57.2958; //以世界坐标为准的方位角，（-180，180）
        //力的变化导致速度变化可能有滞后性。比如路线左偏时，会有一个旋转舵的修正，自然导致力的右偏来纠正航线，然后当去匹配GPS的速度变化时，
        //由于测量的滞后，速度也右偏，但慢一点，结果两个角度形成角度差，如果用测量角度减去推力变化角度，则为负，于是方向角持续增大。
        //这是测量数据滞后的结果，就是导致这个调节持续偏向于一侧，要么持续增加，要么持续减小。修改力的变化为2+3+4+5-6-7-8-9，而测量效应是0-5
        //有时似乎误判严重，所以需要适当抑制调节。
        //之前这里没有考虑旋转问题，实际在掉头时，旋转可以达到每秒40度，如果忽视会有很大误差。
        float world_real_xa = gps_major_bias_queue.last(3).world_real_x + gps_major_bias_queue.last(4).world_real_x + gps_major_bias_queue.last(5).world_real_x + gps_major_bias_queue.last(6).world_real_x + gps_major_bias_queue.last(7).world_real_x;
        float world_real_ya = gps_major_bias_queue.last(3).world_real_y + gps_major_bias_queue.last(4).world_real_y + gps_major_bias_queue.last(5).world_real_y + gps_major_bias_queue.last(6).world_real_y + gps_major_bias_queue.last(7).world_real_y;
        float world_real_xb = gps_major_bias_queue.last(8).world_real_x + gps_major_bias_queue.last(9).world_real_x + gps_major_bias_queue.last(10).world_real_x + gps_major_bias_queue.last(11).world_real_x + gps_major_bias_queue.last(12).world_real_x;
        float world_real_yb = gps_major_bias_queue.last(8).world_real_y + gps_major_bias_queue.last(9).world_real_y + gps_major_bias_queue.last(10).world_real_y + gps_major_bias_queue.last(11).world_real_y + gps_major_bias_queue.last(12).world_real_y;
        //基于历史数据计算当时的世界推力

        float world_chg_x = world_real_xa - world_real_xb;
        float world_chg_y = world_real_ya - world_real_yb;

        float world_chg_ang= atan2(world_chg_x, world_chg_y)*57.2958; //推力变化方向
        float force_chg_norm= sqrt(world_chg_x*world_chg_x+ world_chg_y*world_chg_y); //推力变化强度

        float acc_ang_dif = gps_spd_chg_ang - world_chg_ang; //角度差，正值是机身推力方向在GPS方向的左旋方向。
        float acc_norm= sqrt(spd_east_chg*spd_east_chg+spd_north_chg*spd_north_chg);
        //也转为（-180，180）范围
        if(acc_ang_dif > 180.0) acc_ang_dif-=360.0;
        if(acc_ang_dif < -180.0)  acc_ang_dif+=360.0;

        // //限制角度可能会发生偏调
        FLOAT_LIMIT(acc_ang_dif, -3.0, 3.0); //限制一次性调节范围，避免误判抖动。
        FLOAT_LIMIT(acc_norm, 0.0, 1.2); 
        FLOAT_LIMIT(force_chg_norm, 0.0, 1.2); //5个数据差的和，可能会较大。

        //机身倾角变化量也应该考虑，如果变化很小，很可能是风力引起的速度变化。
        //顶侧风飞行时，很可能持续增加或减小导致偏调。其原因可能是主动电流调节减速导致的。
        float hyadj = 0.3*acc_norm*force_chg_norm*force_chg_norm*acc_ang_dif; //太大似乎不稳定。

        //因为经过了机身转世界坐标，利用了方向角，方向角右旋变小，转换为世界角左旋变大。所以这里可能是加法。
        //20240804 测试确定是用加法，减法会发散。
        _heading_yaw += hyadj; 

    }

    //20250309+
    //可考虑一个干涉项，判断系统的roll，如果roll偏大，则可以适当干涉以避免过大影响拍摄画面的倾角。
    //这个干涉项主要是因为系统无法准确判断方向，存在一定的误差，这个误差有时可以比较大，这主要是陀螺仪的积分累计误差较大
    //尤其是上升到高空中，气温变化剧烈，方向漂移可以很大，但通过运动识别又存在一定的限制无法非常准确。
    //roll当然可以在大侧风中变得很大，但多数时候不会在大风中飞行，roll过大影响画面观感，检测roll过大时，适度调节方向角可以减小roll
    //g0.speed>8.0基本可以避开起步时的旋转，那时的roll参考价值不大。
    if(micro_task.heading_forward && g0.speed>8.0 && fabs(cur_roll)>6.0)
    {
        //有一定的速度，且系统有头部指向，且roll较大。此时可以考虑适度调节_hy以缓解roll
        //roll<0,_hy调大，roll>0,_hy调小可缓解roll的增加趋势。
        if(cur_roll < -14.0) {
            _heading_yaw += 0.05; //每秒0.5度调节力
        }else  if(cur_roll < -12.0) {
            _heading_yaw += 0.04; //每秒0.4度调节力
        }else if(cur_roll< -10.0) {
            _heading_yaw += 0.03; //每秒0.3度调节力
        }else if(cur_roll < -8.0) {
            _heading_yaw += 0.02; //每秒0.2度调节力
        }else if(cur_roll < -6.0) {
            _heading_yaw += 0.01; //每秒0.1度调节力
        }
        else if(cur_roll > 14.0) {
            _heading_yaw -= 0.05; //每秒0.5度调节力
        }else if(cur_roll > 12.0) {
            _heading_yaw -= 0.04; //每秒0.4度调节力
        }else if(cur_roll > 10.0) {
            _heading_yaw -= 0.03; //每秒0.3度调节力
        }else if(cur_roll > 8.0) {
            _heading_yaw -= 0.02; //每秒0.2度调节力
        }else if(cur_roll > 6.0) {
            _heading_yaw -= 0.01; //每秒0.1度调节力
        }
    }

    if(_heading_yaw >= 180) _heading_yaw -= 360;
    if(_heading_yaw < -180) _heading_yaw += 360;

    //_heading_yaw更新到此完成。

    float world_real_x, world_real_y;
    Flight2World(cosheading, sinheading, flight_cur_castx, flight_cur_casty, world_real_x, world_real_y);



    //20231014, 可以考虑去除速度阻尼，有这个阻尼在很难匹配上期望的速度。速度调节项本身也是速度阻尼项。
    //如果去掉速度阻尼，则可以把速度调节差限制的更小些，这样加速减速都可以平缓些。
    //但是速度阻尼是定点时引入的调节因素，如果这里去除，那么和定点函数就不连贯了。
    //现在定点那边也统一去除了速度阻尼项。需要测试

    //20231014，速度阻尼如果没有，则运行过程更平滑，但最后定点的摆动稍大，如果有速度阻尼，则定点摆动小
    //但运动过程中平滑度稍差，所以现在限制速度阻尼在较小范围，对低速敏感而高速不敏感。

    //20231014，低速度阻尼问题也很大，到目标点刹车来不及，会大幅冲过头。
    //20231016分析，假设最高限速20m/s, 在160米距离时就开始减速，匀减速下16秒后到达终点。
    //期间平均速度10m/s, 加速度只要-1.25m/s/s即可达到减速目的，加速度限制在-2.0，2.0之间是没有问题的。
    //在减速期间有速度调节和加速度调节应该就可以控制的比较好，现在有纯速度阻尼和纯imu加速度阻尼是为了
    //在定点时更稳定，那时速度很小，噪音很大，计算速度和加速度不准确
    //考虑两个纯阻尼项修改为可变项，随距离的远近而变化，距离越近阻尼越大，越远纯阻尼越小。

    //加速度阻尼项依旧维持原样，只是速度阻尼项按距离调节。
    float dist_acc_damping_ratio; 
    float dist_spd_damping_ratio; 
    //5-55米范围内是线性的。从1.0->0.1降低，再远就是0.1
    //这里速度阻尼系数最低0.2的设置可能偏高，在20231022的测试中，往来两段的平均速度一个是8.9m/s,一个是11.5m/s
    //在8.9m/s的那一段，平均倾斜角度是17.8度（去除最后一个减速时的倾角）
    //在11.5m/s的那一段，平均倾角是16.96度 （去除最后一个减速时的倾角）都未达到最大倾角限制20度，限速16m/s, 说明有阻尼限制了倾角继续扩大。
    //设置的速度限制是16m/s,方向是东南方/西北方。估计是阻尼太大导致速度上不来。

    //100米外速度阻尼为0, 加速度阻尼0.333
    if(dist<=5.0) {
        dist_spd_damping_ratio=1.0;
        dist_acc_damping_ratio=1.0;
    }
    else if(dist>=105.0) {
        dist_spd_damping_ratio=0.0;
        dist_acc_damping_ratio=0.333;
    }
    else {
        dist_spd_damping_ratio= 1.0- (dist-5.0)/100;
        dist_acc_damping_ratio= 1.0- (dist-5.0)/150;
    }


    //水平速度
    float want_hspd = get_proper_hspd(dist, gps_quality); //20240415新方案，采用分段连续函数，20240416改为平滑函数。

    if(want_hspd > HORI_SPD_LMT) want_hspd= HORI_SPD_LMT;
    //这个速度可能是负数，如果冲过了目标点就是负数，或者方向偏离超过90度也是负数。
    float spd_heading_target_new = g0.speed * cos(spd_angle0/57.2958); //速度*夹角，是到目标方向的速度。
    float spd_heading_target_old = g3.speed * cos(spd_angle3/57.2958); //速度*夹角，是到目标方向的速度。

    //考虑电力代价调节速度。
    //当前电流，电流增量速度，当前速度三个指标决定电力代价调节。
    float cur_dif=current_queue.last() - current_queue.last(5); //5个数据差，0.1秒
    //float ref_mod_limit= sqrt(ref_castx*ref_castx+ref_casty*ref_casty);  //推力模量限制值。
    float ref_mod= sqrt(ref_castx*ref_castx+ref_casty*ref_casty);  //当前推力模量。
    //20250110+
    //高度显著低于设定高度时，限制倾角继续增加，以平衡水平运动和垂直运动，避免陷入角度死锁中。
    //当倾角过大时，无论电机怎么发力，机器高度都无法提高，这就是角度死锁。
    //角度死锁概率并不算高，但依旧有小概率发生，尤其是顶风飞行速度较大时，推力增大也不能有足够的推力来维持高度。
    //这里检测是否高度能够维持，如果不能则减小倾角避免死锁。
    float mod_limit_by_height=0; //0表示不使用限制

    //g0.speed>4.0 改为 spd_heading_target_new>4.0，这样在高速退行时不会被限制调节角度，大风场景下有用。
    //这里之前限制16度以上就开始观察掉高并限制倾角，这导致抗风性较差，大风中飞行迟缓，不能爆发出电池的动力
    //放松到22度
    //20250404略改，变化的强风可能导致气压高度短时间轻微波动，如果太敏感则容易出现抗风偏弱的问题。
    //所以改为宽容一定程度的高度下降。由1.2米以上开始限制侧倾角度。
    if(ref_mod > 0.404 && spd_heading_target_new > 4.0) //大于22度角
    {
        float hd=_height_adjust_queue.last().hd;
        //倾角大于16度，且速度大于4.0，此时可能存在角度死锁问题。
        //此时观察高度偏离，如果偏离较大，就抑制倾角。
        if(hd> 6.0) {
            mod_limit_by_height = 0.98*ref_mod;
        }else if(hd >4.8) {
            mod_limit_by_height = 0.99*ref_mod;
        }else if(hd >3.6) {
            mod_limit_by_height = 0.995*ref_mod;
        }else if(hd >2.4) {
            mod_limit_by_height = 0.997*ref_mod;
        }else if(hd >1.6) {
            mod_limit_by_height = 0.999*ref_mod;
        }
    }


    //ref_mod_limit在里面被调节，作为最大限制值。
    float mod_limit_by_power= ref_mod;

    float want_hspd1=want_hspd;
    float want_hspd2=want_hspd;
    //g0.speed 改为 spd_heading_target_new，防范大风场景下退行。
    //退行时spd_heading_target_new<0 , want_hspd - spd_heading_target_new会较大。
    if(want_hspd > spd_heading_target_new ) {

        float power_caution= PowerCostForPin(g0.speed, current_queue.last(), cur_dif, power_control_solution, ref_current, mod_limit_by_power);
        //当前指向速度太低时不限制推力模量
        if(spd_heading_target_new < 4.0) {
            mod_limit_by_power=0;
        }
        //目标是加速状态，根据电力限制打折处理。power_caution越大，则打折越多。
        //0-1.0 对应的拉扯速度可以是5.0 ~ -0.5
        float sd=want_hspd - g0.speed; //>0
        //20250329 修改了
        if(power_caution < 0.2) {
            if(sd>9.0) sd=9.0;
        }else if(power_caution < 0.3) {
            if(sd>8.0) sd=8.0;
        }else if(power_caution < 0.4) {
            if(sd>7.0) sd=7.0;
        }else if(power_caution < 0.5) {
            if(sd>6.0) sd=6.0;
        }else if(power_caution < 0.6) {
            if(sd>4.0) sd=4.0;
        }else if(power_caution < 0.7) {
            if(sd>2.5) sd=2.5;
        }else if(power_caution < 0.8) {
            if(sd>1.8) sd=1.8;
        }else if(power_caution < 0.9) {
            if(sd>1.2) sd=1.2;
        }else if(power_caution < 0.95) {
            if(sd>0.5) sd = 0.5;
        }else if(power_caution < 1.0) {
            if(sd>0.3) sd = 0.3;
        }else if(power_caution < 1.2) {
            sd=-0.2;
        }else if(power_caution < 1.4) {
            sd=-0.3;
        }else{
            sd=-0.5;
        }


        //退行时spd_heading_target_new < 0, 不调节。
        want_hspd1 = g0.speed + sd;
        
        //电流调节后不能超过最初期望的速度，否则减速阶段有问题。
        if(want_hspd1 > want_hspd) want_hspd1=want_hspd; 

        //保证一个最低的速度，哪怕是电流超过了上限，这可能在大风中是必要的。
        //这里也保证了在退行时不会设置一个反方向的速度。
        if(want_hspd>4.0 && want_hspd1<4.0) want_hspd1=4.0; //不应太小
        else if(want_hspd>3.0 && want_hspd1<3.0) want_hspd1=3.0; //不应太小
        else if(want_hspd>2.0 && want_hspd1<2.0) want_hspd1=2.0; //不应太小
        else if(want_hspd>1.0 && want_hspd1<1.0) want_hspd1=1.0; //不应太小

    }

    //这里的效率比较可能有问题？20241123飞丢一架，速度极高电流极大引发电力问题。
    //如果不考虑指向目标的速度，而是只考虑水平速度，则可能在顺风方向更有飞行效率。于是系统可能会一直沿顺风方向去加速。
    //所以要考虑指向目标的速度的效率问题。

    //飞行效率比较，越大越好。电流到电机，再影响到测量数据，应该有0.2秒以上的延迟。
    //当前GPS数据应该对应于0.2秒前的电流，10个数据。
    //近距离不比对飞行效率，因处于刹车阶段，效率肯定越来越低。速度也要有一定限制，必须要大于一定的速度，有调节性。
    //这里没有在可能的高速高效率情况下提高飞行速度，只是检测高速低效率而打压速度，因为在距离够大时其他机制总是倾向于提高速度。
    if(dist > 20.0 && spd_heading_target_new > 4.0 && spd_heading_target_old > 4.0) { 
 
        //20250315 修改为3个电流均值，或许会控制的更准确。
        // float cur_old= current_queue.last(28)+ current_queue.last(29)+ current_queue.last(30);
        // float cur_new= current_queue.last(8)+ current_queue.last(9)+ current_queue.last(10);
        // float effi_old= spd_heading_target_old/cur_old;
        // float effi_new= spd_heading_target_new/cur_new;

        float effi_old= spd_heading_target_old/current_queue.last(30);
        float effi_new= spd_heading_target_new/current_queue.last(10); //当前数据前移10个电流数据是0.2秒

        if(g0.speed > g4.speed) {
            //更高的速度，观察是否更低的效率，如果是就降速。
            if(effi_old > effi_new *1.2)
            {
                if(want_hspd2 > g0.speed*0.994) {
                    want_hspd2= g0.speed*0.994;
                }
            }
            else if(effi_old > effi_new*1.1)
            {
                if(want_hspd2 > g0.speed*0.996) {
                    want_hspd2= g0.speed*0.996;
                }
            }
            else if(effi_old > effi_new)
            {
                if(want_hspd2 > g0.speed*0.998) {
                    want_hspd2= g0.speed*0.998;
                }
            }
        }else{
            //g0.speed < g4.speed
            //更低的速度，观察是否更高的效率，如果是就继续降低速度。
            if(effi_old*1.2 < effi_new )
            {
                if(want_hspd2 > g4.speed*0.994) {
                    want_hspd2= g4.speed*0.994;
                }
            }
            else if(effi_old*1.1 < effi_new )
            {
                if(want_hspd2 > g4.speed*0.996) {
                    want_hspd2= g4.speed*0.996;
                }
            }
            else if(effi_old < effi_new)
            {
                if(want_hspd2 > g4.speed*0.998) {
                    want_hspd2= g4.speed*0.998;
                }
            }
        }

        //if(want_hspd2 > HORI_SPD_LMT) want_hspd2= HORI_SPD_LMT; //20241123+
        //if(want_hspd2 > want_hspd) want_hspd2=want_hspd; //不应过大 want_hspd2只会比want_hspd小，所以不用这个判断。
        //这里want_hspd2不会比want_hspd大。
        //即使有效率限制，也必须保证最低4.0速度。
        if(want_hspd>4.0 && want_hspd2<4.0) want_hspd2=4.0; //不应太小
        else if(want_hspd>3.0 && want_hspd2<2.0) want_hspd2=3.0; //不应太小
        else if(want_hspd>2.0 && want_hspd2<2.0) want_hspd2=2.0; //不应太小
        else if(want_hspd>1.0 && want_hspd2<1.0) want_hspd2=1.0; //不应太小
    }

    float want_hspd_final= std::min(want_hspd1, want_hspd2); //两个限制下选择较小的速度。
    FLOAT_LIMIT(want_hspd_final, 0, HORI_SPD_LMT);

    //debug, 寻找意外限速的原因。怀疑是多个电流平均判断效率的修改存在问题。但目前已经改动了。
    // if(want_hspd > 8.0 && want_hspd_final < 7.0) {
    //     char dbg[128];
    //     sprintf(dbg, "whs=%4.2f, %4.2f, %4.2f, %4.2f\n", want_hspd, want_hspd_final, want_hspd1, want_hspd2);
    //     logmessage(dbg);
    // }

    //在此函数里，倾角的最大值不受信号质量限制，因为这个函数主要用来处理运动，如果受到GPS干扰
    //立刻表现出倾角受限，速度降低，很容易给干扰者信心并持续干扰。在高速运动时，更容易摆脱小
    //范围的信号干扰。

    //电流调节后置似乎压不住运动姿态调节。所以前置。
    //前置调节基于调节基准，电流过大时，打压基准，在经过调节算法后，再度砍掉超出基准的部分。
    //float cur_current=system_power_current; //如果不关注功耗，则记录为POWER_WARN，避免出现0的情况。
    //新的调节后的数值，需要保留老的数值，在电流超出时，用来做限制。

    //保留参考数据，随后参考数据会做为上限使用。
    //20240407 纠正错误，之前这个赋值放在了ref_cast调节之后。
    float new_castx=ref_castx;
    float new_casty=ref_casty;    //新的调节后的数值，需要保留老的数值，在电流超出时，用来做限制。

    //随着电流超过额定值，水平速度上限可以打折以抑制电流消耗，或可避免使用封顶的方式去调节侧倾角。

#define RATIO_PIN (0.1)
    //可能是没有起点到终点的中心线做基准，导致路线容易弯曲。
    //如果考虑设定基线，每个点都去对比基线，可能就不太会出现路径弯曲了。

    //上边计算了期望的速度。但下面是分解了两个方向来调节的。
    //需要分解期望的速度到两个方向上去。
    //之前0.15太大，castx,casty变化很大，调节不温柔

    //东向调节
        float want_hspd_east = want_hspd_final*sin(angle/57.2958); 
        float spdx_dif = want_hspd_east - spdEast1;
        float want_hacc_east = (want_hspd_east - spdEast1)/3.0;
        float accx_dif = want_hacc_east - accEast;
        float accx_time_dif= accEast - accEast_long; //加速度差分
        float spdx_trans = SigmoidTrans(spdEast1); //速度阻尼项
        float accx_trans= SigmoidTrans(accEast); //加速度阻尼。

    //北向调节。
        float want_hspd_north = want_hspd_final*cos(angle/57.2958); 
        float spdy_dif = want_hspd_north - spdNorth1;
        float want_hacc_north = (want_hspd_north - spdNorth1)/3.0;
        float accy_dif = want_hacc_north - accNorth;
        float accy_time_dif= accNorth - accNorth_long; //加速度差分
        float spdy_trans= SigmoidTrans(spdNorth1);  //基于当前速度的阻尼项
        float accy_trans= SigmoidTrans(accNorth); //基于当前加速度的阻尼项

    //联合限制速度和加速度的大小，而不是分别限制，这样进出场路径不易弯折，末端定位也不易漂移很大。
        float spd_dif_mod= sqrt(spdx_dif*spdx_dif+spdy_dif*spdy_dif);

        //不同的距离不同的限制，距离远可以抑制加速度，距离近可以稳定位置。
        float spd_lmt, acc_lmt;
        get_spd_acc_lmt_by_dist(dist, spd_angle0, spd_lmt, acc_lmt);

        if(spd_dif_mod > spd_lmt) {
            spdx_dif *= spd_lmt/spd_dif_mod;
            spdy_dif *= spd_lmt/spd_dif_mod;
        }

        float acc_dif_mod= sqrt(accx_dif*accx_dif+ accy_dif*accy_dif);
        if(acc_dif_mod > acc_lmt) {
            accx_dif *= acc_lmt/acc_dif_mod;
            accy_dif *= acc_lmt/acc_dif_mod;
        }

        float want_acc_mod= sqrt(want_hacc_east*want_hacc_east+want_hacc_north*want_hacc_north);
        if(want_acc_mod > acc_lmt) {
            want_hacc_east *= acc_lmt/want_acc_mod;
            want_hacc_north *= acc_lmt/want_acc_mod;
        }
        //这样统一限制过后，路径应该不易弯折。20240416
        //增强速度及加速度的调节力度。之前0.5，现在0.8
        //实验中，由侧后方来风，使得路线朝右偏离，路线形成反时针弧线，这样导致方向角的判断增大。
        //一个原因就是侧向位移没控制好，力度不够大。本应该及时调节roll角来抵抗侧风的，由于不及时
        //形成了路线的弧线，导致最后调节方向角去处理。
        //对于持续侧风如果调节力度太软，容易导致方向角持续偏离。
        {
            new_castx += spdx_dif*0.8*RATIO_PIN; //速度调节项.
            new_castx += accx_dif*0.8*RATIO_PIN; //加速度调节.
            new_castx -= accx_time_dif*0.35*RATIO_PIN; //为了加速度更平滑，20240713+
            new_castx -= accx_trans*RATIO_PIN*dist_acc_damping_ratio;  //近处阻尼强。

            //这里发现错误，new_casty当为new_castx, 20240705！需要检查在4/18测试时是否用的这个代码！
            //经过检查，4/18日的飞行也是这里有错误，导致突然的路线偏离无法纠正。
            //new_casty -= spdx_trans*RATIO_PIN*dist_damping_ratio*gps_quality; 
            new_castx -= spdx_trans*RATIO_PIN*dist_spd_damping_ratio; //20240705纠正。
            new_castx -= (1.2*imu_waccx - 0.2*want_hacc_east)*RATIO_PIN;
        }

        {
            new_casty += spdy_dif*0.8*RATIO_PIN;
            new_casty += accy_dif*0.8*RATIO_PIN; //加速度调节项
            new_casty -= accy_time_dif*0.35*RATIO_PIN; //为了加速度更平滑，20240713+ 
            new_casty -= accy_trans*RATIO_PIN*dist_acc_damping_ratio; //加速度阻尼
            new_casty -= spdy_trans*RATIO_PIN*dist_spd_damping_ratio; 
            new_casty -= (1.2*imu_waccy - 0.2*want_hacc_north)*RATIO_PIN;
        }

#undef RATIO_PIN

        //debug
        // if(spd_heading_target_new<4.0) {
        //     char dbg[128];
        //     sprintf(dbg,"lowspd,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f\n",
        //         g0.speed,g0.direction,spd_heading_target_new, ref_castx, ref_casty,new_castx,new_casty);
        //     logmessage(dbg);
        // }
    //当飞行路线偏离航线时，执行推力方向旋转调节。20240721+ 
    //修正推力方向可能影响到方向的判断，因为路线的偏移被这个修正所掩盖。
    //如果路线偏向目标的右侧，那么可能是我们的方向识别就偏右侧了，此时本可以用这个信息来纠正方向，
    //但被舵向调节纠正了路线偏移，这个纠正方向的信息就被遮掩了。
    //不妨这样来考虑，这个舵向修正需要考虑是否会增大roll的绝对值，如果是，则调节力度放缓。
    //如果当前roll已经很大，继续调节舵向会继续增大roll，则调节力度放缓。腾出的空间可以去调节方向角。
    //新增spd_heading_target_new > 1.0，保证正向飞行，反向飞行时不调节舵向。
    if(gps_major_bias_queue.last(6).inuse==3 && spd_heading_target_new > 1.0)
    {

        float lmt_ang0= spd_angle0; //>0 目标点在速度方向的右侧， <0 目标点在速度方向的左侧。
        FLOAT_LIMIT(lmt_ang0, -15.0, 15.0);

        float lmt_ang1= spd_angle1; //>0 目标点在速度方向的右侧， <0 目标点在速度方向的左侧。
        FLOAT_LIMIT(lmt_ang1, -15.0, 15.0);

        //float lmt_ang2= spd_angle2; //>0 目标点在速度方向的右侧， <0 目标点在速度方向的左侧。
        //FLOAT_LIMIT(lmt_ang2, -20.0, 20.0);


        float dr2=1.0; //距离很近时，方向角偏差大，可靠性不高，调节力度减弱。
        if(dist<5) dr2=0.2;
        else if(dist<10) dr2=0.3;
        else if(dist<15) dr2=0.5;
        else if(dist<20) dr2=0.7;
        else if(dist<20) dr2=0.8;
        else if(dist<30) dr2=0.9;

        float sr2=1.0;
        if(spd1 <0.2) sr2=0.05;
        else if(spd1 < 0.5) sr2=0.1;
        else if(spd1 < 1.0) sr2=0.2;
        else if(spd1 < 2.0) sr2=0.5;
        else if(spd1 < 4.0) sr2=0.7;
        else if(spd1 < 5.0) sr2=0.9;

        //要考虑角度变化的加速度问题。20250402
        //20250403测试表面舵向调节力度偏低，增强，持续5度偏差会调节1.0度。过强的调节可能会形成S型曲线。
        //float steer1= 0.05*(lmt_ang0+lmt_ang1+lmt_ang2); //I调节
        //float steer2= 0.2*lmt_ang0; //P调节
        //float steer3= 0.05*(2.0*lmt_ang0-lmt_ang1-lmt_ang2); //D调节, 差分，反应了当前的变化率。

        //两个部分，前者是差异度调节，后者是加速度调节
        //float steer= ((0.6*lmt_ang0+0.3*lmt_ang1+0.15*lmt_ang2) + 0.6*(lmt_ang0-lmt_ang1))*sr2*dr2;
        float steer= (1.2*lmt_ang0 -0.3*lmt_ang1)*sr2*dr2;
        //float steer = (0.02*lmt_ang0+0.01*lmt_ang1)*sr2*dr2; //如果这个数大于零，即推力会顺时针旋转。

        //推力方向是世界系的，要判断当前的roll需要另外的办法。cur_roll, cur_pitch可参考。
        if(micro_task.heading_forward)
        {
            //如果当前飞行是处在头部指向目标的模式下，可以适度调节舵量，避免roll越来越大。
            //舵向调节的力度减弱会使路线更弯曲，但方向可能判断更准确。
            //20250331修正理解，这里早期的理解是相反的
            //舵向右旋会使roll朝正值移动
            //舵向左旋会使roll朝负值移动
            //steer>0 是右旋调节舵向
            //steer<0 是左旋调节舵向

            //考虑当前姿态，适度减小舵向调节以避免过大的roll，多数时候侧风不会很大。
            
            if(steer>0 && cur_roll>0.0 && cur_pitch<0.0) 
            {
                //准备右旋，roll朝正向移动，需要减弱调节
                if(cur_roll>13.0) steer*=0.6;
                else if(cur_roll>11.0) steer*=0.7;
                else if(cur_roll>9.0) steer*=0.8;
                else if(cur_roll>7.0) steer*=0.9;
            }
            else if(steer<0 && cur_roll<0.0 && cur_pitch<0.0) 
            {
                //准备左旋，roll朝负向移动，需要减弱调节
                if(cur_roll<-13.0) steer*=0.6;
                else if(cur_roll<-11.0) steer*=0.7;
                else if(cur_roll<-9.0) steer*=0.8;
                else if(cur_roll<-7.0) steer*=0.9;
            }


        }

        //推力上执行顺时针旋转，相当于坐标系逆时针旋转。
        float nx,ny;
        rotate_ccw(new_castx, new_casty, steer, nx, ny); 
        new_castx= nx;
        new_casty= ny;
    }

    float new_xy= sqrt(new_castx*new_castx+new_casty*new_casty); 

    //现在电流大小直接影响最大侧倾角度
    if(mod_limit_by_power >0 && new_xy > mod_limit_by_power)
    {
        float r= mod_limit_by_power/new_xy; //<1.0
        new_castx*=r;
        new_casty*=r;
        new_xy*=r; //后面有强制限制，如果这里调节过了，后面就不太可能被强制再次调节。相当于一个标记。
        
        // if(spd_heading_target_new<4.0) {
        //     char dbg[128];
        //     sprintf(dbg,"lowspd, mod limit, %3.2f,%3.2f,%3.2f,%3.2f\n",
        //         mod_limit_by_power, r, new_castx, new_casty);
        //     logmessage(dbg);
        // }
    }

    if( mod_limit_by_height > 0 && new_xy > mod_limit_by_height)
    {
        float r= mod_limit_by_height/new_xy; //<1.0
        new_castx*=r;
        new_casty*=r;
        new_xy*=r; 

        // if(spd_heading_target_new<4.0) {
        //     char dbg[128];
        //     sprintf(dbg,"lowspd, mod limit by height, %3.2f,%3.2f,%3.2f,%3.2f\n",
        //         mod_limit_by_height, r, new_castx, new_casty);
        //     logmessage(dbg);
        // }
    }

    //30度可能超过了死锁角度，这个角度和硬件有关。
    //但考虑水平速度时也要考虑风速，如果迎面吹来10m/s的风，而速度是6m/s，则相对于风力是16m/s的风压，此时30度倾角
    //很可能会死锁，无论怎么增加电机功率都无法提高机身。现在上面有代码防范这种掉高度的情况，就是限制倾角增加。
    //速度较大时，这个强制限制可以较小，以主动避免发生角度死锁。速度低时则可以放松些。
    //这里之前用g0.speed来测度并限制倾角，如果考虑定点情况被大风吹远，产生负速度，则会限制倾角导致无法定住，所以这里需要修改
    //可以使用spd_heading_target_new替代，一旦为负值则不限制倾角大小。20250404修正。
    float angle_lmt= 0.6248; //32度,如果速度很低或退行，最大32度角。
    if(spd_heading_target_new > 18.0) {
        angle_lmt=0.384; //21度
    }else if(spd_heading_target_new > 16.0) {
        angle_lmt=0.404; //22度
    }else if(spd_heading_target_new > 14.0) {
        angle_lmt=0.4245; //23度
    }else if(spd_heading_target_new > 12.0) {
        angle_lmt=0.4452; //24度
    }else if(spd_heading_target_new > 10.0) {
        angle_lmt=0.4663; //25度
    }else if(spd_heading_target_new > 8.0) {
        angle_lmt=0.4877; //26度
    }else if(spd_heading_target_new > 6.0) {
        angle_lmt=0.5095; //27度
    }else if(spd_heading_target_new > 4.0) {
        angle_lmt=0.5317; //28度
    }else if(spd_heading_target_new > 2.0) {
        angle_lmt=0.5773; //30度
    }

    if(new_xy > angle_lmt) { //强制限制动态最大角度 
        float r= angle_lmt/new_xy;
        new_castx*=r;
        new_casty*=r;

        // if(spd_heading_target_new<4.0) {
        //     char dbg[128];
        //     sprintf(dbg,"lowspd, mod limit by spd, %3.2f,%3.2f,%3.2f,%3.2f,%3.2f\n",
        //         angle_lmt, r, spd_heading_target_new, new_castx, new_casty);
        //     logmessage(dbg);
        // }
    }

    //世界坐标转到机身坐标的 x,y

    float castx_flight, casty_flight;
    World2Flight(cosheading, sinheading, new_castx, new_casty, castx_flight, casty_flight);

    //20231023修改
	float roll= atan(castx_flight)*57.2958; //atan算出的角度比asin略小点
	float pitch= -atan(casty_flight)*57.2958;

    //机头指向目标点。
    //如果这里不调节，那么机头的指向是最初的方向决定的。随着imu的yaw积分漂移，会发生变化。
    //如果这里调节，那么就会根据_heading_yaw的修正去修正。可能总体上会更稳妥。
    //但是取消这个调节是因为，最后的曲线可能不够直，以至于最后机身会明显旋转。
    //可以考虑在150米外连续调节，进入150米内就不再调节。
    #if 1

    if(micro_task.heading_forward && dist > 100.0f)
    {
        float want_yaw= _heading_yaw - angle; //这里angle(0,360), _heading_yaw(-180,180)
        if(want_yaw > 180.0) want_yaw-=360.0;
        if(want_yaw < -180.0) want_yaw+=360.0;
        FLOAT_LIMIT(want_yaw, -180.0, 180.0);
        micro_task.yaw=want_yaw;
    }
    
    #endif
    //调节记录
    gps_major_bias majorb;
    majorb.gps_tmark=dtime;
    majorb.world_castx = new_castx;
    majorb.world_casty = new_casty;
    majorb.roll_bias=roll;
    majorb.pitch_bias=pitch;
    majorb.gps_spd= g0.speed; //20240418+
    majorb.gps_spdDir = g0.direction;
    majorb.gps_spdEast =g0.speed_east;
    majorb.gps_spdNorth=g0.speed_north;
    majorb.gps_dist= dist;
    majorb.gps_distEast=distEast;
    majorb.gps_distNorth=distNorth;
    majorb.flight_real_x=flight_cur_castx; //20240706+
    majorb.flight_real_y=flight_cur_casty;
    majorb.world_real_x = world_real_x;
    majorb.world_real_y = world_real_y;
    majorb.current= system_power_current; //20240308+

    majorb.inuse=3; //标记是平飞过程记录数据。
    gps_major_bias_queue.push(majorb);

    newdata=true;
    return true;

#else
    return false;
#endif

}

//新函数，检查当前是否已经锚定，如果有锚定则使用本函数。如果没有锚定，检查是否符合锚定条件，
//如果不符合锚定条件，则调用gps_position_stable()抑制所有速度。

//三种_heading_yaw的调节状况。20240302 总结。
//1. 围绕目标点旋转的，顺时针旋转，目标点在速度方向右侧，则_heading_yaw也要顺时针旋转调节，即调小，反之同理。
//2. 朝向目标点飞行的，突遇横风吹偏，如果形成了顺时针偏转，即航向角增大，目标点在速度方向的左侧，_heading_yaw需要增大。反之同理。
//3. 朝向目标点飞行在减速阶段的，如果形成了顺时针偏转，即航向角增大，目标点在速度方向的左侧，_heading_yaw需要减小。反之同理。
//1/2不矛盾，2/3两者是有矛盾的，大风天气下的减速较难处理。 
//1/2合并起来可以仅区分目标点在速度方向的左侧或右侧来统一处理。不用考虑运动轨迹是否包裹了目标点。
//3的处理现在考虑新的方法，即判断内部水平推力的变化，去匹配速度的变化，比较两者的角度差去纠正偏角。
//在准备修改_heading_yaw的调节前做了备份，
//目标点较近的，不易区分目标点在路线内还是在外，可以主要使用速度的方向变化来判断。
//目标点较远的，容易区分目标点在路线内还是外，可以使用目标点的左右侧来判断。
//


bool CMicroTaskExec::gps_position_adjust()
{
#ifdef USE_GPS

    if(fabs(micro_task.roll)>0.01 || fabs(micro_task.pitch)>0.01)
    {
        //取消锚点，冲刷历史记录。
        micro_task.gps_anchor=false;
        return false;
    }


    if(!micro_task.gps_anchor) {
        return gps_position_brake();
    }

    //现在运行到这里就是有锚点状态。

    uint32_t now=get_time_mark();
    uint32_t dtime;
    gps_info g0,g1,g2,g3,g4;

    critical_section_enter_blocking(&gps_queue_section);
    g0=gps_queue.last(0);
    g1=gps_queue.last(1);
    g2=gps_queue.last(2);
    g3=gps_queue.last(3);
    g4=gps_queue.last(4);
    critical_section_exit(&gps_queue_section);

    dtime=g0.tmark;

    if(now - dtime > 700) {
        gps_major_bias majorb;
        majorb.gps_tmark=dtime;
        gps_major_bias_queue.push(majorb);
        //如果有这个，则受到间歇性干扰时，系统行为是发现干扰即取消定点，恢复后再刹车然后定点，这样会有缓慢位置漂移。
        //如果注销这个，则对于短时间的干扰，可以保持位置不动。
        //micro_task.gps_anchor=false; 

        if(!micro_task.gps_bad_mark)
        {
            logmessage("lost gps in gps-pos-adjust\n");
            micro_task.gps_bad_mark=true; //标记GPS信号问题。
        }
        return false;
    }
    else if(micro_task.gps_bad_mark)
    {
        micro_task.gps_bad_mark=false;
        logmessage("gps resume in gps-pos-adjust\n");
    }

    if(dtime == gps_major_bias_queue.last().gps_tmark) {
        //信号间微调位置。似乎没有也还行。
        return true;
    }


    //这里取不同的间隔是为了更准确的加速度
    uint32_t tgap2 = g0.tmark - g2.tmark;
    float t2_inverse= 1000.0/float(tgap2);
    uint32_t tgap3 = g0.tmark - g3.tmark;
    float t3_inverse= 1000.0/float(tgap3);
    uint32_t tgap4 = g0.tmark - g4.tmark;
    float t4_inverse= 1000.0/float(tgap4);

    //当前位置，用来求距离。现在坐标有滤波 20230806+
    //因为取消了滤波，所以这里增加平均值采样。20240105
    double lati0= (g0.latitude+g1.latitude+g2.latitude+g3.latitude)/4.0;
    double logi0= (g0.longitude+g1.longitude+g2.longitude+g3.longitude)/4.0;

    //最近速度。通过位置变化计算出来的速度不如GPS给出的数据准确，算出来的偏大。所以还是使用GPS的速度
    //20240106修改为3组数据的平均值。
    float spdEast1= (g0.speed_east+g1.speed_east+g2.speed_east)/3.0; //标准单位
    float spdNorth1= (g0.speed_north+g1.speed_north+g2.speed_north)/3.0;
    float spd1= (g0.speed+g1.speed+g2.speed)/3.0; //模量速度 m/s。


    //加速度
    float accEast2 = (g0.speed_east- g2.speed_east)*t2_inverse; //标准单位,m/s/s
    float accNorth2= (g0.speed_north-g2.speed_north)*t2_inverse;
    float accEast3 = (g0.speed_east- g3.speed_east)*t3_inverse; //标准单位,m/s/s
    float accNorth3= (g0.speed_north-g3.speed_north)*t3_inverse;
    float accEast4 = (g0.speed_east- g4.speed_east)*t4_inverse; //标准单位,m/s/s
    float accNorth4= (g0.speed_north-g4.speed_north)*t4_inverse;

    //2个加速度等权重处理
    float accEast= (accEast2+accEast3)/2.0;
    float accNorth= (accNorth2+accNorth3)/2.0;

    //稍早的加速度。用于差分调节。
    float accEast_long= (accEast3+accEast4)/2.0;
    float accNorth_long= (accNorth3+accNorth4)/2.0;


    //因为速度没有参与滤波，所以加速度噪音大，适当限制幅度。
    //因为现在是4个加速度平均，稍放开范围到2.0m/s
    FLOAT_LIMIT(accEast, -5.0, 5.0);
    FLOAT_LIMIT(accNorth, -5.0, 5.0); 
    FLOAT_LIMIT(accEast_long, -5.0, 5.0);
    FLOAT_LIMIT(accNorth_long, -5.0, 5.0); 

    //加速度算完以后再考虑速度的调节。
    //在低速情况下，噪音的概率较大，可以降低数值以稳定位置。
    //float sr= gps_speed_reliable(spd1);

    //accEast*=sr;
    //accNorth*=sr; 20231224 取消加速度信度调节
    //spdEast1*=sr;
    //spdNorth1*=sr;

    //到锚定点的两个距离。
    float dist=0;
    float angle=0;


    global_gps_dist_angle(lati0, logi0, micro_task.target_gps_latitude, micro_task.target_gps_logitude, dist, angle);


    //三个夹角，间隔2个数据，可以更准确判断弧线内卷或外卷
    float spd_angle0=0; //当前速度和到目标连线的夹角
    float spd_angle1=0; //速度夹角
    float spd_angle2=0; //速度夹角

    //现在是当前速度方向和期望方向的夹角。区分顺逆旋转方向。
    spd_angle0= angle - g0.direction; //>0 目标点在速度方向的右侧， <0 目标点在速度方向的左侧。
    if(spd_angle0>180) spd_angle0-=360;
    if(spd_angle0<-180) spd_angle0+=360;

    spd_angle1 = angle - g2.direction;
    if(spd_angle1>180) spd_angle1-=360;
    if(spd_angle1<-180) spd_angle1+=360;

    spd_angle2 = angle - g4.direction;
    if(spd_angle2>180) spd_angle2-=360;
    if(spd_angle2<-180) spd_angle2+=360;


    //距离信度调节
    //float dr= gps_dist_reliable(dist, g0.hacc);

    float distEast= dist*sin(angle/57.2958);
    float distNorth= dist*cos(angle/57.2958);

    //imu水平加速度噪音可能较大，取多个数据平均。做阻尼用。
    float imu_accx=0;
    float imu_accy=0; //y方向和光流方向相反
    float cur_yaw;
    float cur_pitch;
    float cur_roll;

    //取消了机身加速度和GPS加速度的方向匹配。
    //考虑新的方法，即当有锚点存在时，机身加速度方向大体指向锚点。绕圈时应该也是。
    //持续观察加速度方向，在比较明显的加速度上进行持续调节。微弱的加速度不算。
    //由于gps是10hz,所以持续观察50个imu的加速度数据。就利用已有的imu_accx, imu_accy,正好50个数据。
    //20240305 改回50个数据
    critical_section_enter_blocking(&imu_queue_section);
    cur_pitch=imu_queue.last().angle[0];
    cur_roll=imu_queue.last().angle[1];
    cur_yaw=imu_queue.last().angle[2];
    for(size_t i=0; i<50; i++)
    {
        imu_accx+= imu_queue.last(i).acc_gnd[0];
        imu_accy+= imu_queue.last(i).acc_gnd[1];
    }
    critical_section_exit(&imu_queue_section);

    imu_accx/=5.0; //求均值，转标准单位
    imu_accy/=5.0;

    //考虑由于yaw漂移，cur_yaw存在从180突变到-180的可能。
    //如果_heading_yaw= -45, cur_yaw 从 180突变到-180，也不会引起系统行为突变。

    float heading= _heading_yaw - cur_yaw;
    //这里可能并不需要限制范围。
    if(heading<0.0) heading+=360.0;
    if(heading>=360.0) heading-=360.0;
    heading/=57.2958;

    float cosheading= cos(heading);
    float sinheading= sin(heading);

    //机身坐标的加速度信息转到世界坐标
    float imu_waccx, imu_waccy;
    Flight2World(cosheading, sinheading, imu_accx, imu_accy, imu_waccx, imu_waccy);

    FLOAT_LIMIT(imu_waccx, -5.0, 5.0); //同GPS加速度限制范围
    FLOAT_LIMIT(imu_waccy, -5.0, 5.0); //20231224 放松到3.0

    float flight_cur_castx= tan(cur_roll/57.2958); //20240706+
    float flight_cur_casty= -tan(cur_pitch/57.2958);

    // //三均值 20231103改动，测试没有表现显著差异，似乎倾角调节更平滑。
    //最近发现，如果改为2个均值则机身摆动明显，还是3个均值更平滑。
    float ref_castx=0;
    float ref_casty=0;

    for(size_t i=0;i<3;i++)
    {
        ref_castx+= gps_major_bias_queue.last(i).world_castx;
        ref_casty+= gps_major_bias_queue.last(i).world_casty;
    }

    ref_castx/=3.0;
    ref_casty/=3.0;

    float gps_quality=GpsQuality(g0.ns, g0.hacc); //信号质量调节系数。20240109+


    //_heading_yaw更新。

    
        //统一的调节 20240304，1，航向调节，2，弧线调节，3.加速度方向和速度方向的夹角，4.加速度匹配机身姿态
        //在近距离环绕时，由于震荡，机器远离目标点，那么这个夹角会很大，比如说-180， 180，而不是一个很小的锐角。
        //此时的调节应该朝着180度方向去调整。
        float dr2=1.0; //方向角度的信度，距离很近时，方向角偏差大，可靠性不高。
        if(dist<5*g0.hacc) dr2=0.1;
        else if(dist<10*g0.hacc) dr2=0.2;
        else if(dist<15*g0.hacc) dr2=0.35;
        else if(dist<20*g0.hacc) dr2=0.5;
        else if(dist<30*g0.hacc) dr2=0.65;
        else if(dist<40*g0.hacc) dr2=0.8;
        else if(dist<50*g0.hacc) dr2=0.95;

        float sr2=1.0;
        if(spd1 <0.2) sr2=0.02;
        else if(spd1 < 0.3) sr2=0.04;
        else if(spd1 < 0.4) sr2=0.06;
        else if(spd1 < 0.6) sr2=0.08;
        else if(spd1 < 1.2) sr2=0.1;
        else if(spd1 < 2.4) sr2=0.3;
        else if(spd1 < 4.8) sr2=0.5;
        else if(spd1 < 9.6) sr2=0.7;
        else if(spd1 < 12.0) sr2=0.9;

    //加速度方向和速度方向的夹角，越是接近0的越不可靠。可靠的时候加速度方向应和速度方向相反。
    //这里利用了天然的定点特性，即抑制一切速度。所以在平飞时不能用这个方法分辨方向。
    //20240404+
    //20240717 这个目前纠偏力度占比最大。
    //如果距离目标点很远，这个调节就很可疑，因为加速度可能会指向目标点，和速度同向。
    //20240717 增加随距离衰减dr3, 近距离作用大，远距离作用小。
    if(gps_major_bias_queue.last(6).inuse==1)
    {
        //现在在定点中，速度方向和加速度方向夹角调节的作用仅次于最后一种。风中似乎无偏。

        //两种方法，一种是速度方向和目标方向的夹角调节。远距离作用大，近距离作用小
        //另一种是速度方向和加速度方向夹角调节。远距离作用小，近距离作用大

        //加速度方向和目标方向的夹角，如果不限制一个偏差范围，则在侧风波动下可能是不对称的调节。
        //比如右前侧风加强，则加速度是-135度方向，右前侧风减弱，加速度是45度方向。多次反复后总体调节量就偏离了0.
        //所以限制每次调节在一个小角度是必要的。
        //如何利用好加速度方向和目标方向的夹角是个难题。
        //在最初加速段，其匹配目标方向可以很好的校准方向。但路途中的风力很难处理。风力可以推动加速度朝向任何方向，
        //也可能是某个方向及其反向，比如某方向的恒风，大小改变，则造成的加速度是180度对称的。

        float cur_acc_ang= atan2(accEast, accNorth)*57.2958; //20240712+ 忘记了这个。
        float angle_trans=angle; //angle (0-360) angle_trans(-180, 180) 把范围调节成相同的。
        if(angle>=180.0) angle_trans=angle - 360.0;

        float acc_dir = angle_trans - cur_acc_ang; //目标角和加速度角的夹角。
        if(acc_dir>180.0) acc_dir-=360.0;
        if(acc_dir<-180.0) acc_dir+=360.0;

        float accnorm= sqrt(accEast*accEast+accNorth*accNorth);
        float accr=gps_speed_reliable(accnorm); //加速度信度，采用速度信度
        float acc_vert= sin(acc_dir/57.2958)*accnorm; //垂直于路径方向的加速度分量。>0 指向左侧，<0 指向右侧。
        //float acc_para= cos(acc_dir/57.2958)*accnorm; //平行路径方向的加速度分量。 >0 指向目标点。
        //沿着目标方向的加速度可能指向目标，也可能反向，不调节。

        //指向左侧，_heading_yaw调小，反之调大。
        //FLOAT_LIMIT(acc_dir, -5.0, 5.0); //限制每次的调节幅度。

//20240727 将曲线旋转调节移动到这里。不区分加速减速
        float dc1= g0.direction - g1.direction; //(-180,180) >0曲线右旋。

        if(dc1>180.0) dc1-= 360.0;
        if(dc1<-180.0) dc1+= 360.0;
        FLOAT_LIMIT(dc1, -10.0, 10.0);

//新增距离调节项 20241124+，距离越远调节越强。
        float distadj=1.0; 
        if(dist>500.0) distadj=1.0;
        else if(dist>100.0) distadj=0.9;
        else if(dist>30.0) distadj=0.8;
        else if(dist>15.0) distadj=0.7;
        else if(dist>6.0) distadj=0.6;
        else distadj=0.5; 

        float adj0= 0.10*dc1*distadj; 
        _heading_yaw -= adj0; 

        //不限制方向角度的调节，防止偏调。但希望180度对称的加速度在调节上互相抵消。
        //如果希望所有180度对称反向的加速度在调节上都互相抵消，则可能不能使用目标角。
        //可以分解加速度，一个沿指向目标点的连线的投影，一个是垂直投影，分别调节。那么对称的加速度可以互相抵消。
        //20240718 新思路，将加速度沿着目标指向方向分解为平行和垂直分量。
        //目前认为，垂直加速度的调节是加减速一致的，所以20米外不限制使用，20米内线性递减。
        //之前一直避免在减速段使用这个调节 20241126
        //5m内递减主要是当前到目标点的连线可能存在较大误差，导致加速度在垂直方向的投影误差。
        float distadj2=1.0;
        if(dist<5.0) {
            distadj2= dist/5.0;
        }

        FLOAT_LIMIT(acc_vert, -0.5, 0.5);
        float adj=0.15*acc_vert*accr*distadj2;
        _heading_yaw += adj;  

    }


    //20240706+ 考虑新的加速度方向匹配办法。基于机身侧倾角度的变化。去对比GPS数据的变化，跨度12个历史gps数据.
    //较大的历史数据跨度或许可以更准确的匹配GPS加速度。

    //有角度限制可能造成单向调节问题。
    {

        float spd_east_chg= gps_major_bias_queue.last(1).gps_spdEast - gps_major_bias_queue.last(6).gps_spdEast;
        float spd_north_chg= gps_major_bias_queue.last(1).gps_spdNorth - gps_major_bias_queue.last(6).gps_spdNorth;

        //在有锚点的时候，主动调节产生的速度变化总是指向锚点，要么就是在刹车时反向指向锚点。垂直方向的调节应该都是风力影响产生的。
        //刹车时的调节难度较大，这种逆向加速也可能是顶风风力产生的，也可能是近距离刹车，也可能是电流限制刹车。
        //有可能会发生调节错误，所以暂时过滤掉逆向加速的调节，即如果速度变化方向大体指向锚点的就调节，偏差较大的不调节。
        float gps_spd_chg_ang= atan2(spd_east_chg, spd_north_chg)*57.2958; //以世界坐标为准的方位角，（-180，180）

//之前这里没有考虑旋转问题，实际在掉头时，旋转可以达到每秒40度，如果忽视会有很大误差。
        float world_real_xa = gps_major_bias_queue.last(3).world_real_x + gps_major_bias_queue.last(4).world_real_x + gps_major_bias_queue.last(5).world_real_x + gps_major_bias_queue.last(6).world_real_x + gps_major_bias_queue.last(7).world_real_x;
        float world_real_ya = gps_major_bias_queue.last(3).world_real_y + gps_major_bias_queue.last(4).world_real_y + gps_major_bias_queue.last(5).world_real_y + gps_major_bias_queue.last(6).world_real_y + gps_major_bias_queue.last(7).world_real_y;
        float world_real_xb = gps_major_bias_queue.last(8).world_real_x + gps_major_bias_queue.last(9).world_real_x + gps_major_bias_queue.last(10).world_real_x + gps_major_bias_queue.last(11).world_real_x + gps_major_bias_queue.last(12).world_real_x;
        float world_real_yb = gps_major_bias_queue.last(8).world_real_y + gps_major_bias_queue.last(9).world_real_y + gps_major_bias_queue.last(10).world_real_y + gps_major_bias_queue.last(11).world_real_y + gps_major_bias_queue.last(12).world_real_y;
        //基于历史数据计算当时的世界推力

        float world_chg_x = world_real_xa - world_real_xb;
        float world_chg_y = world_real_ya - world_real_yb;

        float world_chg_ang= atan2(world_chg_x, world_chg_y)*57.2958; //推力变化方向
        float force_chg_norm= sqrt(world_chg_x*world_chg_x+ world_chg_y*world_chg_y); //推力变化强度

        float acc_ang_dif = gps_spd_chg_ang - world_chg_ang; //角度差，负值是机身推力方向在GPS方向的左旋方向。
        float acc_norm= sqrt(spd_east_chg*spd_east_chg+spd_north_chg*spd_north_chg);
        //也转为（-180，180）范围
        if(acc_ang_dif > 180.0) acc_ang_dif-=360.0;
        if(acc_ang_dif < -180.0)  acc_ang_dif+=360.0;


        FLOAT_LIMIT(acc_ang_dif, -3.0, 3.0); //限制一次性调节范围，避免误判抖动。
        FLOAT_LIMIT(acc_norm, 0.0, 1.2); 
        FLOAT_LIMIT(force_chg_norm, 0.0, 1.2); //5个数据差的和，可能会较大。
        //机身倾角变化量也应该考虑，如果变化很小，很可能是风力引起的速度变化。
        float hyadj = 0.3*acc_norm*force_chg_norm*force_chg_norm*acc_ang_dif; //太大似乎不稳定。

        //因为经过了机身转世界坐标，利用了方向角，所以这里可能是加法。
        _heading_yaw += hyadj; 

    }


    if(_heading_yaw >= 180.0) _heading_yaw -= 360;
    if(_heading_yaw < -180.0) _heading_yaw += 360;

    float world_real_x, world_real_y;
    Flight2World(cosheading, sinheading, flight_cur_castx, flight_cur_casty, world_real_x, world_real_y);

    //20230802修改了平衡代码里的阻尼，测试后GPS定点响应速度很快，侧倾角变化很快。
    //为了适应这种情况，适度降低调节系数。原来是0.15，修改为0.1

#define RATIO_POS_HOLD (0.1)

    //20231014修改，因为现在运行到这里的状态都是有锚点的，所以考虑去除纯粹的速度阻尼。速度调节本身可以替代速度阻尼
    //放开倾角限制到20度。这需要做定点测试
    //测试表明，没有纯速度阻尼下也一样的定点，只是定点的漂移速度较快。
    //包括pin_to_location，在没有速度阻尼下，到达目标点更迅速，不拖泥带水。
    //为了在定点时更稳定，还是需要有适度的速度阻尼。需要其在高速下作用不大，在低速下作用明显。

    //加速度阻尼项依旧维持原样，只是速度阻尼项按距离调节。
    //100米外速度阻尼为0, 加速度阻尼0.333
    float dist_spd_damping_ratio;
    float dist_acc_damping_ratio;
    if(dist<=5.0) {
        dist_spd_damping_ratio=1.0;
        dist_acc_damping_ratio=1.0;
    }
    else if(dist>=105.0) {
        dist_spd_damping_ratio=0.0;
        dist_acc_damping_ratio=0.333;
    }
    else {
        dist_spd_damping_ratio= 1.0- (dist-5.0)/100;
        dist_acc_damping_ratio= 1.0- (dist-5.0)/150;
    }

    //水平速度
    float want_hspd = get_proper_hspd(dist, gps_quality); //20240714+ 统一和pin那边一样。
    if(want_hspd > HORI_SPD_LMT) want_hspd= HORI_SPD_LMT;

    //当前的调节参数在卫星定位数据很好时，表现良好，但是
    //在卫星数据受到干扰产生较大误差时，摆动很大。
    //这是因为定位数据摆动较大造成的。可能是定位数据也可能是速度数据的波动。
    //为了减小这种摆动幅度，分析数据质量，然后降低调节力度并限制最大调节幅度。
    
    //后面的调节项都乘以这个数，但imu不变。这样防止太大的摆动。
    //另外调节的上下限都受这个影响，信号不好时，上下限更小。

    //东向调节

        float want_hspd_east = want_hspd*sin(angle/57.2958); 
        float spdx_dif = want_hspd_east - spdEast1;
        float want_hacc_east = (want_hspd_east - spdEast1)/3.0;
        float accx_dif = want_hacc_east - accEast;
        float accx_time_dif= accEast - accEast_long; //加速度差分
        float spdx_trans= SigmoidTrans(spdEast1); //纯速度阻尼。20231014修改，希望低速下作用明显但高速下影响小。*3.0高速下迅速饱和。
        float accx_damping= SigmoidTrans(accEast); //加速度阻尼。

    //北向调节。
 
        float want_hspd_north = want_hspd*cos(angle/57.2958); //用乘法避免/0
        float spdy_dif = want_hspd_north - spdNorth1;
        float want_hacc_north= (want_hspd_north - spdNorth1)/3.0;
        float accy_dif = want_hacc_north - accNorth;
        float accy_time_dif= accNorth - accNorth_long; //加速度差分
        float spdy_trans=SigmoidTrans(spdNorth1); //纯速度阻尼。20231014修改，希望低速下作用明显但高速下影响小。*3.0高速下迅速饱和。
        float accy_damping=SigmoidTrans(accNorth);

        //联合限制速度和加速度的大小，而不是分别限制，这样进出场路径不易弯折，末端定位也不易漂移很大。
        float spd_dif_mod= sqrt(spdx_dif*spdx_dif+spdy_dif*spdy_dif);

        //不同的距离不同的限制，距离远可以抑制加速度，距离近可以稳定位置。
        float spd_lmt, acc_lmt;
        get_spd_acc_lmt_by_dist(dist, spd_angle0, spd_lmt, acc_lmt);

        if(spd_dif_mod > spd_lmt) {
            spdx_dif *= spd_lmt/spd_dif_mod;
            spdy_dif *= spd_lmt/spd_dif_mod;
        }

        //之前两方向限制个1.0，现在统一限制0.5，加减速更柔和
        float acc_dif_mod= sqrt(accx_dif*accx_dif+ accy_dif*accy_dif);
        if(acc_dif_mod > acc_lmt) {
            accx_dif *= acc_lmt/acc_dif_mod;
            accy_dif *= acc_lmt/acc_dif_mod;
        }

        float want_acc_mod= sqrt(want_hacc_east*want_hacc_east+want_hacc_north*want_hacc_north);
        if(want_acc_mod > acc_lmt) {
            want_hacc_east *= acc_lmt/want_acc_mod;
            want_hacc_north *= acc_lmt/want_acc_mod;
        }

        //20230925，提高阻尼抑制高速刹车的运动范围，减小调节项。
        ref_castx += spdx_dif*0.8*RATIO_POS_HOLD;   //速度差异调节项
        ref_castx += accx_dif*0.8*RATIO_POS_HOLD; //加速度调节
        ref_castx -= accx_time_dif*0.35*RATIO_POS_HOLD; 
        ref_castx -= accx_damping*RATIO_POS_HOLD*dist_acc_damping_ratio;  //近处阻尼强。
        ref_castx -= spdx_trans*RATIO_POS_HOLD*dist_spd_damping_ratio;  //速度阻尼项，*0.1希望高速下影响很小，低速影响大。
        //掺杂一个调节项在这里，会使得机身位置更加不稳定，会围绕锚点运动。
        //如果没有调节项，定点看起来更稳，即使方向角偏差很大，似乎也很稳。但是运动量太小就对方向识别造成困难，使系统不能及时调节方向。
        //考虑对方向的识别，还是少量增加一点调节项在这里。
        ref_castx -= (1.2*imu_waccx - 0.2*want_hacc_east)*RATIO_POS_HOLD; //加速度阻尼。
    
        //20230925，提高阻尼抑制高速刹车的运动范围，减小调节项。
        ref_casty += spdy_dif*0.8*RATIO_POS_HOLD;  //速度差异调节项。
        ref_casty += accy_dif*0.8*RATIO_POS_HOLD; //加速度调节项
        ref_casty -= accy_time_dif*0.35*RATIO_POS_HOLD;
        ref_casty -= accy_damping*RATIO_POS_HOLD*dist_acc_damping_ratio; //加速度阻尼
        ref_casty -= spdy_trans*RATIO_POS_HOLD*dist_spd_damping_ratio;  //速度阻尼项，*0.1希望高速下影响很小，低速影响大。
        ref_casty -= (1.2*imu_waccy - 0.2*want_hacc_north)*RATIO_POS_HOLD; //加速度阻尼。
    

    //执行方向旋转调节。20240721+ 
    if(gps_major_bias_queue.last(6).inuse==1)
    {

        float lmt_ang0= spd_angle0;
        FLOAT_LIMIT(lmt_ang0, -15.0, 15.0);

        float lmt_ang1= spd_angle1;
        FLOAT_LIMIT(lmt_ang1, -15.0, 15.0);

        float steer = (0.02*lmt_ang0+0.01*lmt_ang1)*sr2*dr2; //如果这个数大于零，之前希望方向角减小，即顺时针旋转，即希望推力会顺时针旋转。

        //推力上执行顺时针旋转，相当于坐标系逆时针旋转。
        float nx,ny;
        rotate_ccw(ref_castx, ref_casty, steer, nx, ny); 
        ref_castx= nx;
        ref_casty= ny;
    }

    float ref_xy= sqrt(ref_castx*ref_castx+ref_casty*ref_casty);

    //实测27度角在7级风下可能定不住，还是放松到35度，如果有持续大风，可能会导致高度无法维持住。
    if(ref_xy>0.6248) { //放松到30度 0.6248=tan(32)
        float r= 0.6248/ref_xy;
        ref_castx*=r;
        ref_casty*=r;
    }

    //世界坐标转回机身坐标的 x,y
    float castx_flight, casty_flight;
    World2Flight(cosheading, sinheading, ref_castx, ref_casty, castx_flight, casty_flight);

#undef RATIO_POS_HOLD
    //再通过这个点计算roll/pitch.
	float roll= atan(castx_flight)*57.2958; //atan算出的角度比asin略小点
	float pitch= -atan(casty_flight)*57.2958;


    //记录调节记录
    gps_major_bias majorb;
    majorb.gps_tmark=dtime;

    majorb.world_castx = ref_castx;
    majorb.world_casty = ref_casty;
    majorb.gps_spdDir = g0.direction;
    majorb.gps_spdEast =g0.speed_east;
    majorb.gps_spdNorth=g0.speed_north;
    majorb.gps_dist= dist;
    majorb.gps_distEast=distEast;
    majorb.gps_distNorth=distNorth;
    majorb.gps_spd=g0.speed;
    majorb.roll_bias=roll;
    majorb.pitch_bias=pitch;
    majorb.flight_real_x=flight_cur_castx;
    majorb.flight_real_y=flight_cur_casty;
    majorb.world_real_x = world_real_x;
    majorb.world_real_y = world_real_y;
    majorb.current= system_power_current;
    majorb.inuse=1; //标记是定点过程记录数据。

    gps_major_bias_queue.push(majorb);

    return true;

#undef GPS_TIME_DELAY_MS

#else
    return false;
#endif
}

//20240313+
//get a good acc limit for current spd; spd>=0
float CMicroTaskExec::gps_brake_limit_acc(float spd)
{
    float acc = fabs(spd)*2.0;
    FLOAT_LIMIT(acc, 1.0, 6.0);
    return acc;
}
//20230927新增，无锚点下，抑制一切速度，使速度降低到0，可以理解为提供一个刹车效果。
//当稳定达到一定时间后，再转用带锚点的函数，因为没有锚点，所以没有速度矩可用，只使用速度方向去调节指南。

//20240121大风中起飞，不能定点，最后飞丢，主要原因是无法抵抗大风，起初存在高度上升的问题，可能是摇杆漂移。
//注意到高速刹车时，有大幅摆动现象，速度杀停后不能及时稳住，而是反向增速。这估计是GPS数据延迟引起，暂无解决办法。

//最好是在这个函数里下锚点，而不是另一个函数抢夺控制权下锚点。
//在这里可以不断监测GPS速度和位移，在速度很低时，根据imu的加速度，迅速回正机身，然后观察到加速度较低时下锚点。
//在高速刹车时，常常是速度下降到0时，加速度却最大。这种情况下下锚点很可能回摆严重。此时迅速回正机身就很有必要。


//新的刹车处理，用来替换老的
//目前90度偏角不会有大圈环绕。
bool CMicroTaskExec::gps_position_brake()
{
#ifdef USE_GPS

    uint32_t now=get_time_mark();
    uint32_t dtime;
    gps_info g0,g1,g2,g3,g4; //5个历史数据做平均处理。

    //float mean_spd_mod=0; //最后5个速度的均值，不考虑方向。

    //当前速度用g0,g1均值。加速度用g2,g3均值去减，位置用g0-g4均值
    critical_section_enter_blocking(&gps_queue_section);
    g0=gps_queue.last(0);
    g1=gps_queue.last(1);
    g2=gps_queue.last(2);
    g3=gps_queue.last(3);
    g4=gps_queue.last(4);
    // for(size_t i=0;i<3;i++) {
    //     mean_spd_mod+= gps_queue.last(i).speed;
    // }
    critical_section_exit(&gps_queue_section);

    //mean_spd_mod/=3.0;

    dtime=g0.tmark;

    if(now - dtime > 700) {
        gps_major_bias majorb;
        majorb.gps_tmark=dtime;
        gps_major_bias_queue.push(majorb);

        if(!micro_task.gps_bad_mark)
        {
            logmessage("lost gps in gps-pos-stable\n");
            micro_task.gps_bad_mark=true; //标记GPS信号问题。
        }
        return false;
    }
    else if(micro_task.gps_bad_mark)
    {
        micro_task.gps_bad_mark=false;
        logmessage("gps resume in gps-pos-stable\n");
    }

    //当前的调节参数在卫星定位数据很好时，表现良好，但是
    //在卫星数据受到干扰产生较大误差时，摆动很大。
    //这是因为定位数据摆动较大造成的。可能是定位数据也可能是速度数据的波动。
    //为了减小这种摆动幅度，分析数据质量，然后降低调节力度并限制最大调节幅度。

    //微调也用到这个，提前上来。
    float gps_quality=GpsQuality(g0.ns, g0.hacc); //信号质量调节系数。20240109+

    if(dtime == gps_major_bias_queue.last().gps_tmark) {

        return true; //不使用微调，目前微调的作用不大，看不出差别。而且似乎带来稳定问题。20240308

    }


    //这里取不同的间隔是为了更准确的加速度
    uint32_t tgap2 = g0.tmark - g2.tmark;
    float t2_inverse= 1000.0/float(tgap2);
    uint32_t tgap3 = g0.tmark - g3.tmark;
    float t3_inverse= 1000.0/float(tgap3);
    uint32_t tgap4 = g0.tmark - g4.tmark;
    float t4_inverse= 1000.0/float(tgap4);

    //最近速度。通过位置变化计算出来的速度不如GPS给出的数据准确，算出来的偏大。所以还是使用GPS的速度

    //20240106修改为3组数据的平均值。
    float spdEast1= (g0.speed_east+g1.speed_east+g2.speed_east)/3.0; //标准单位
    float spdNorth1= (g0.speed_north+g1.speed_north+g2.speed_north)/3.0;
    //如果采用这个方法，模量不一致，比如g0.speed_east=-0.3， g1.speed_east=0.3，那么算出来的spdEast1=0
    //float spd1= (g0.speed+g1.speed)/2.0; //模量速度 m/s。 
    float spd1 = sqrt(spdEast1*spdEast1+spdNorth1*spdNorth1); //这里重新计算模量。

    //加速度 去掉最近的一个差值，时间太短了，误差波动太大。
    float accEast2 = (g0.speed_east- g2.speed_east)*t2_inverse; //标准单位,m/s/s
    float accNorth2= (g0.speed_north-g2.speed_north)*t2_inverse;
    float accEast3 = (g0.speed_east- g3.speed_east)*t3_inverse; //标准单位,m/s/s
    float accNorth3= (g0.speed_north-g3.speed_north)*t3_inverse;
    float accEast4 = (g0.speed_east- g4.speed_east)*t4_inverse; //标准单位,m/s/s
    float accNorth4= (g0.speed_north-g4.speed_north)*t4_inverse;

    //2个加速度等权重处理
    float accEast= (accEast2+accEast3)/2.0;
    float accNorth= (accNorth2+accNorth3)/2.0;

    //稍早的加速度。用于差分调节。
    float accEast_long= (accEast3+accEast4)/2.0;
    float accNorth_long= (accNorth3+accNorth4)/2.0;
   

    //因为速度没有参与滤波，所以加速度噪音大，适当限制幅度。
    //因为现在是4个加速度平均，稍放开范围到2.0m/s
    FLOAT_LIMIT(accEast, -5.0, 5.0); //20231224 放大范围
    FLOAT_LIMIT(accNorth, -5.0, 5.0);
    FLOAT_LIMIT(accEast_long, -5.0, 5.0);
    FLOAT_LIMIT(accNorth_long, -5.0, 5.0);

    //float gps_acc_norm= sqrt(accEast*accEast+accNorth*accNorth);

//2均值 20231103改动，测试没有表现显著差异，似乎倾角调节更平滑。

    float ref_castx=0;
    float ref_casty=0;

    for(size_t i=0;i<2;i++)
    {
        ref_castx+= gps_major_bias_queue.last(i).world_castx;
        ref_casty+= gps_major_bias_queue.last(i).world_casty;
    }

    ref_castx/=2.0;
    ref_casty/=2.0;

    float sr= gps_speed_reliable(spd1);

    //spdEast1*=sr;
    //spdNorth1*=sr;

    //将imu数据读取前置，避免2次读取。
    //imu水平加速度噪音可能较大，取多个数据平均。做阻尼用。
    float imu_accx=0;
    float imu_accy=0;
    float cur_yaw;
    float cur_pitch;
    float cur_roll;
    //取消了机身加速度和GPS加速度的方向匹配。
    //考虑新的方法，即当有锚点存在时，机身加速度方向大体指向锚点。绕圈时应该也是。
    //持续观察加速度方向，在比较明显的加速度上进行持续调节。微弱的加速度不算。
    //由于gps是10hz,所以持续观察50个imu的加速度数据。就利用已有的imu_accx, imu_accy,正好50个数据。
    critical_section_enter_blocking(&imu_queue_section);
    cur_pitch=imu_queue.last().angle[0];
    cur_roll=imu_queue.last().angle[1];
    cur_yaw=imu_queue.last().angle[2];
    for(size_t i=0; i<50; i++)
    {
        imu_accx+= imu_queue.last(i).acc_gnd[0];
        imu_accy+= imu_queue.last(i).acc_gnd[1];
    }
    critical_section_exit(&imu_queue_section);

    imu_accx/=5.0; //求均值，转标准单位
    imu_accy/=5.0;

    //float imu_acc_mod=sqrt(imu_accx*imu_accx+imu_accy*imu_accy);
    float gps_acc_mod=sqrt(accEast*accEast+accNorth*accNorth);

    float heading= _heading_yaw - cur_yaw;
    if(heading<0.0) heading+=360.0;
    if(heading>=360.0) heading-=360.0;
    heading/=57.2958;

    float cosheading= cos(heading);
    float sinheading= sin(heading);

    //这里增加判断，如果是初次进入刹车，不立即锚定，因历史数据可能是上次降落留下的。
    //避免再次起飞时引用上次落地时的倾角数据，会导致起飞侧飞严重。
    //if(mean_spd_mod<0.25 && fabs(imu_accx) < 0.15 && fabs(imu_accy) < 0.15)
    //这里放松点标准，防止因为震动而不能定住。imu_acc其实不重要，即使是较大的数值进入稳定定点，也可以被定点函数后期抵消掉。

    if(spd1 < 0.3 && gps_acc_mod < 0.4 /*&& imu_acc_mod <0.7*/) //20240714修改，mean_spd_mod只采用最近5个数据。
    {

            //较低的速度和加速度，可锚定。
            gps_major_bias majorb=gps_major_bias_queue.last(); //沿用上一个roll/pitch.
            majorb.gps_tmark=dtime;
            majorb.gps_spdDir = g0.direction;
            majorb.gps_spdEast = g0.speed_east;
            majorb.gps_spdNorth= g0.speed_north;
            majorb.gps_dist= 0;
            majorb.gps_distEast=0;
            majorb.gps_distNorth=0;
            majorb.gps_spd=g0.speed;
            gps_major_bias_queue.push(majorb);

            //处理锚定，随后控制权交给gps_position_adjust.
            micro_task.gps_anchor=true;
            //只要GPS锚定，光流就不能锚定，目前会冲突，为防范可能的冲突意外这里这么设置。
            micro_task.oflow_anchor_x=false;
            micro_task.oflow_anchor_y=false;
            micro_task.target_gps_latitude= g0.latitude;
            micro_task.target_gps_logitude= g0.longitude;

            // Send_remote_message(45);

            // char dbg[168];
            // sprintf(dbg, "anchor, %4.2f,%4.2f,%4.2f,%4.2f,%4.2f\n", spd1, imu_acc_mod, gps_acc_mod, accEast, accNorth);
            // logmessage(dbg);

            // logmessage("mark anchor\n");

            return true;

    }


    //机身坐标的加速度信息转到世界坐标
    //20240113引入新的转换函数。
    float imu_waccx, imu_waccy;
    Flight2World(cosheading, sinheading, imu_accx, imu_accy, imu_waccx, imu_waccy);

    FLOAT_LIMIT(imu_waccx, -5.0, 5.0); //同GPS加速度限制范围
    FLOAT_LIMIT(imu_waccy, -5.0, 5.0);

    float flight_cur_castx= tan(cur_roll/57.2958); //20240706+
    float flight_cur_casty= -tan(cur_pitch/57.2958);

    //以当前姿态倾角转世界倾角，做参考值，也可记录为参考值
    //差别不大。
    //float ref_castx, ref_casty;
    //Flight2World(cosheading, sinheading, flight_cur_castx, flight_cur_casty, ref_castx, ref_casty);


        float sr2=1.0;
        if(spd1 <0.2) sr2=0.05;
        else if(spd1 < 0.3) sr2=0.1;
        else if(spd1 < 0.4) sr2=0.15;
        else if(spd1 < 0.6) sr2=0.25;
        else if(spd1 < 1.2) sr2=0.5;
        else if(spd1 < 2.4) sr2=0.7;
        else if(spd1 < 4.8) sr2=0.9;


    //用加速度方向和速度方向的夹角调节方向，减速阶段应该完全相反。
    //20240404+
    

    //如果高速下被接管，方向正好搞反，由于倾角限制，可能没有明显的加速度。此时要考察是否有弧线弯曲。
    //如果减速不明显，有没有弧线弯曲，也可以推断为方向有问题。

    //加速度和速度的夹角。夹角<90速度会增加，>90速度减小。
    //有加速倾向，加速度和速度夹角小于90度。是方向严重有问题。
    //<36.0的角度差使得旋转一圈的时间是大约5秒，如果能在5秒内旋转一圈，则可以较快的自然纠正角度差，否则纠正的速度可能偏低。

    //由于控制的同步性，减速过程往往中间会杀到接近停下，但是随后发现减速度过高，又放松，于是速度又加速了
    //此时就产生了方向误判，认为方向相反，然后大幅调节方向。

    if(gps_major_bias_queue.last(5).inuse==2)
    {
        //可以开始匹配加速度和速度的角度差。
        float cur_spd_ang= atan2(spdEast1, spdNorth1)*57.2958; //方向角，和Y轴的夹角。
        float cur_acc_ang= atan2(accEast, accNorth)*57.2958;

        float ang_dif= cur_spd_ang - cur_acc_ang; //-180,180范围。
        if(ang_dif > 180) ang_dif -= 360;
        if(ang_dif < -180) ang_dif += 360;

        float accnorm= sqrt(accEast*accEast+accNorth*accNorth);
        float accr=gps_speed_reliable(accnorm); //加速度信度，采用速度信度

        if(ang_dif<0)
        {
            float ad= -180.0 - ang_dif; //负数
            float adj= 0.02*ad*sr*accr*gps_quality;
            _heading_yaw+= adj;
        }
        else
        {
            float ad= 180.0 - ang_dif; //正数
            float adj= 0.02*ad*sr*accr*gps_quality; 
            _heading_yaw+= adj;
        }

    }
    
   
    //曲线旋转调节
    {
    
        //小于90度偏差时，速度会减速，旋转方向还是一样，如果_heading_yaw指向北偏西，就是顺时针旋转。
        //速度绝对值减小，有收敛趋势。
        float dc1= g0.direction - g1.direction; //(-180,180) >0曲线右旋。
        if(dc1>180.0) dc1-= 360.0;
        if(dc1<-180.0) dc1+= 360.0;
        FLOAT_LIMIT(dc1, -15.0, 15.0); 

        _heading_yaw -= 0.1*dc1*sr2; //这个没有计算到目标的夹角，只看速度角差分，适合绕锚点盘旋。

        //20240324+ 曲线旋转的差分，避免超调。先修改上面的差值，从g0-g2改为g0-g1
        float dc2= g1.direction - g2.direction; 
        if(dc2>180.0) dc2-= 360.0;
        if(dc2<-180.0) dc2+= 360.0;
        FLOAT_LIMIT(dc2, -15.0, 15.0); 
        _heading_yaw -= 0.02*(dc1-dc2)*sr2; //这里取变化的速度差
    }


    //20240706+ 加速度匹配机身姿态变化
    float spd_east_chg= gps_major_bias_queue.last(1).gps_spdEast - gps_major_bias_queue.last(6).gps_spdEast;
    float spd_north_chg= gps_major_bias_queue.last(1).gps_spdNorth - gps_major_bias_queue.last(6).gps_spdNorth;
    float gps_spd_chg_ang= atan2(spd_east_chg, spd_north_chg)*57.2958; //以世界坐标为准的方位角，（-180，180）

//之前这里没有考虑旋转问题，实际在掉头时，旋转可以达到每秒40度，如果忽视会有很大误差。
        float world_real_xa = gps_major_bias_queue.last(3).world_real_x + gps_major_bias_queue.last(4).world_real_x + gps_major_bias_queue.last(5).world_real_x + gps_major_bias_queue.last(6).world_real_x + gps_major_bias_queue.last(7).world_real_x;
        float world_real_ya = gps_major_bias_queue.last(3).world_real_y + gps_major_bias_queue.last(4).world_real_y + gps_major_bias_queue.last(5).world_real_y + gps_major_bias_queue.last(6).world_real_y + gps_major_bias_queue.last(7).world_real_y;
        float world_real_xb = gps_major_bias_queue.last(8).world_real_x + gps_major_bias_queue.last(9).world_real_x + gps_major_bias_queue.last(10).world_real_x + gps_major_bias_queue.last(11).world_real_x + gps_major_bias_queue.last(12).world_real_x;
        float world_real_yb = gps_major_bias_queue.last(8).world_real_y + gps_major_bias_queue.last(9).world_real_y + gps_major_bias_queue.last(10).world_real_y + gps_major_bias_queue.last(11).world_real_y + gps_major_bias_queue.last(12).world_real_y;
        //基于历史数据计算当时的世界推力

        float world_chg_x = world_real_xa - world_real_xb;
        float world_chg_y = world_real_ya - world_real_yb;

        float world_chg_ang= atan2(world_chg_x, world_chg_y)*57.2958; //推力变化方向
        float force_chg_norm= sqrt(world_chg_x*world_chg_x+ world_chg_y*world_chg_y); //推力变化强度

        float acc_ang_dif = gps_spd_chg_ang - world_chg_ang; //角度差，负值是机身推力方向在GPS方向的左旋方向。
        float acc_norm= sqrt(spd_east_chg*spd_east_chg+spd_north_chg*spd_north_chg);
        //也转为（-180，180）范围
        if(acc_ang_dif > 180.0) acc_ang_dif-=360.0;
        if(acc_ang_dif < -180.0)  acc_ang_dif+=360.0;

        FLOAT_LIMIT(acc_ang_dif, -3.0, 3.0); //限制一次性调节范围，避免误判抖动。
        FLOAT_LIMIT(acc_norm, 0.0, 1.2); 
        FLOAT_LIMIT(force_chg_norm, 0.0, 0.8); //5个数据差的和，可能会较大。

        //机身倾角变化量也应该考虑，如果变化很小，很可能是风力引起的速度变化。
        float hyadj = 0.05*acc_norm*force_chg_norm*acc_ang_dif; //太大似乎不稳定。

        _heading_yaw -= hyadj; //应该是减去


    if(_heading_yaw >= 180) _heading_yaw -= 360;
    if(_heading_yaw < -180) _heading_yaw += 360;

    float world_real_x, world_real_y;
    Flight2World(cosheading, sinheading, flight_cur_castx, flight_cur_casty, world_real_x, world_real_y);

    // char dbg[64];
    // sprintf(dbg,"bk hy=%4.2f\n",_heading_yaw);
    // logmessage(dbg);

    // char dbg[128];
    // sprintf(dbg,"in stable hy=%4.2f,spd=%4.2f, acc=%4.2f, sr=%4.2f, accr=%4.2f, accad=%4.2f, hr=%4.2f\n", 
    // _heading_yaw, spd1, accnorm, sr, accr, ang_dif, _heading_reliable);
    // logmessage(dbg);

    //20230802修改了平衡代码里的阻尼，测试后GPS定点响应速度很快，侧倾角变化很快。
    //为了适应这种情况，适度降低调节系数。原来是0.15，修改为0.1

#define RATIO_GPS (0.1)

    //20231014，取消速度调节项，因为没有锚点，所以也没有速度调节，之前的调节也就是阻尼项。
    //调大相应的速度阻尼。取消速度阻尼封顶限制。放开倾角限制到20度。


    //后面的调节项都乘以这个数，但imu不变。这样防止太大的摆动。
    //另外调节的上下限都受这个影响，信号不好时，上下限更小。

    //20240313 关于刹车，目前还有明显震荡，因刹车导致的倾角不能及时恢复，或是，GPS速度数据慢了一些。
    //所以要考虑，在高速时，刹车的加速度允许较大，而低速时，刹车的加速度不能太大，
    //加速度的上限需要根据速度去调节，这样速度降低后，加速度也降低，即使GPS数据有延时，导致的震荡也不大。

    // char dbg[128];
    // sprintf(dbg, "%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f\n", spdEast1, spdNorth1, spd1, gps_acc_mod, imu_accx, imu_accy, _heading_yaw);
    // logmessage(dbg);

    //float want_acc_limit= gps_brake_limit_acc(spd1); //总加速度模量限制。20250407+ 这样比分向限制应该更好。

    //当前一个问题是这里的调节速度超过了机架的执行速度很多。调节力度主要体现在期望加速度上，对于速度阻尼的调节偏弱了。
    //现在把加速度调节参数降低，把速度阻尼参数提高上去。20250407
    //东向调节
    {
        //20240107 上下限的设置改为放在调节参数上
        float want_accx= -spdEast1*2.0;
        float want_accx_lmt= gps_brake_limit_acc(spdEast1); //20240313 加速度上限根据速度调节，防止震荡。
        FLOAT_LIMIT(want_accx, -want_accx_lmt, want_accx_lmt);
        float spdx_trans= SigmoidTrans(spdEast1*2.0); //纯速度阻尼。
        float accx_dif = (want_accx - accEast);
        float accx_time_dif= accEast - accEast_long; //加速度差分阻尼。 >0表示加速度在增强。

        //20230925，提高阻尼抑制高速刹车的运动范围，减小调节项。
        float accx_damping= SigmoidTrans(accEast); //加速度阻尼。

        ref_castx += accx_dif*0.5*RATIO_GPS*gps_quality; //gps加速度调节项，20240106修改
        ref_castx -= accx_time_dif*0.3*RATIO_GPS*gps_quality; //如加速度随时间增强，则减少x推力分量，可能更平滑。
        ref_castx -= spdx_trans*RATIO_GPS*gps_quality;  //速度阻尼项，
        ref_castx -= accx_damping*0.5*RATIO_GPS*gps_quality;
        ref_castx -= 1.3*imu_waccx*RATIO_GPS; //降低依赖避免因机身振动造成定不住的情况。

        //debug
        // float adj0= accx_dif*0.5*RATIO_GPS*gps_quality; //之前0.7，改0.5
        // float adj1= -accx_time_dif*0.2*RATIO_GPS*gps_quality;
        // float adj2= -spdx_trans*RATIO_GPS*gps_quality;
        // float adj3= -accx_damping*0.1*RATIO_GPS*gps_quality;
        // float adj4= -imu_waccx*RATIO_GPS;

        // //ref_castx += adj0+adj1+adj2+adj3+adj4;
        // char dbg[128];
        // sprintf(dbg,"bx,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f\n", 
        //     ref_castx, want_accx,spdx_trans,accEast,accEast_long,accx_dif, accx_damping, imu_waccx,gps_quality);
        // logmessage(dbg);
        // ref_castx += adj0+adj1+adj2+adj3+adj4;
        // sprintf(dbg,"ax,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f\n", ref_castx, adj0,adj1,adj2,adj3,adj4);
        // logmessage(dbg);

    }

    //北向调节。
    {

        float want_accy= -spdNorth1*2.0;
        float want_accy_lmt= gps_brake_limit_acc(spdNorth1); 
        FLOAT_LIMIT(want_accy, -want_accy_lmt, want_accy_lmt);
        float spdy_trans= SigmoidTrans(spdNorth1*2.0); //纯速度阻尼。这里和有锚点的处理不同，因为全靠速度阻尼，所以比较大，而且不限幅。
        float accy_dif = (want_accy - accNorth);
        float accy_time_dif= accNorth - accNorth_long; //加速度差分阻尼。 >0表示加速度在增强。
        float accy_damping=SigmoidTrans(accNorth); //基于当前加速度的阻尼项

        ref_casty += accy_dif*0.5*RATIO_GPS*gps_quality; //20240106 修改
        ref_casty -= accy_time_dif*0.3*RATIO_GPS*gps_quality; //20240713+ 如果y方向加速度增强，则减小推力分量，可能更平滑。
        ref_casty -= spdy_trans*RATIO_GPS*gps_quality;  //速度阻尼项，
        ref_casty -= accy_damping*0.5*RATIO_GPS*gps_quality;
        ref_casty -= 1.3*imu_waccy*RATIO_GPS; //降低依赖避免因机身振动造成定不住的情况。

                //debug
        // float adj0= accy_dif*0.5*RATIO_GPS*gps_quality; //之前0.7，改0.5
        // float adj1= -accy_time_dif*0.2*RATIO_GPS*gps_quality;
        // float adj2= -spdy_trans*RATIO_GPS*gps_quality; 
        // float adj3= -accy_damping*0.1*RATIO_GPS*gps_quality;
        // float adj4= -imu_waccy*RATIO_GPS;

        // char dbg[128];
        // sprintf(dbg,"by,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f\n", 
        //     ref_casty, want_accy,spdy_trans,accNorth,accNorth_long,accy_dif, accy_damping, imu_waccy, gps_quality);
        // logmessage(dbg);
        // ref_casty += adj0+adj1+adj2+adj3+adj4;
        // sprintf(dbg,"ay,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f\n", ref_casty, adj0,adj1,adj2,adj3,adj4);
        // logmessage(dbg);

    }
#undef RATIO_GPS

    //27度限制很难抵抗7级风力，可能在大风下长时间刹不住导致位置漂移。还是放松到30度限制
    float ref_xy= sqrt(ref_castx*ref_castx+ref_casty*ref_casty); //0.6248=tan(32), 
    if(ref_xy>0.6248) {
        float r= 0.6248/ref_xy;
        ref_castx*=r;
        ref_casty*=r;
    }

    //世界坐标转回机身坐标的 x,y
    float castx_flight, casty_flight;
    World2Flight(cosheading, sinheading, ref_castx, ref_casty, castx_flight, casty_flight);

    //记录机身速度给微调处理。
    //float spdx_flight, spdy_flight;
    //World2Flight(cosheading, sinheading, spdEast1, spdNorth1, spdx_flight, spdy_flight);
    //由于spdEast1,spdNorth1是三个综合值，延迟较大，易引起震荡，这里采用最后一个值。似乎无差别
    //World2Flight(cosheading, sinheading, g0.speed_east, g0.speed_north, spdx_flight, spdy_flight);


    //再通过这个点计算roll/pitch.
    //20231023修改
	float roll= atan(castx_flight)*57.2958; //atan算出的角度比asin略小点
	float pitch= -atan(casty_flight)*57.2958;


    //调节记录
    gps_major_bias majorb;
    majorb.gps_tmark=dtime;
    majorb.world_castx = ref_castx;
    majorb.world_casty = ref_casty;
    majorb.gps_spdDir = g0.direction;
    majorb.gps_spdEast = g0.speed_east;
    majorb.gps_spdNorth = g0.speed_north;
    majorb.gps_dist= 0;
    majorb.gps_distEast=0;
    majorb.gps_distNorth=0;
    majorb.gps_spd=g0.speed;
    majorb.roll_bias=roll;
    majorb.pitch_bias=pitch;
    majorb.flight_real_x=flight_cur_castx;
    majorb.flight_real_y=flight_cur_casty;
    majorb.world_real_x = world_real_x;
    majorb.world_real_y = world_real_y;
    majorb.inuse=2; //标记是刹车过程记录数据。
    gps_major_bias_queue.push(majorb);


    return true;

#undef GPS_TIME_DELAY_MS

#else
    return false;
#endif   

}

//机身坐标转世界坐标
void CMicroTaskExec::Flight2World(float cur_yaw, float fx, float fy, float& wx, float& wy)
{
    float heading= _heading_yaw - cur_yaw;
    if(heading<0.0) heading+=360.0;
    if(heading>=360.0) heading-=360.0;
    heading/=57.2958;

    float cosheading= cos(heading);
    float sinheading= sin(heading);

    wx= fx*cosheading + fy*sinheading;
    wy= fy*cosheading - fx*sinheading;
}

void CMicroTaskExec::Flight2World(float cosheading, float sinheading, float fx, float fy, float& wx, float& wy)
{
    wx= fx*cosheading + fy*sinheading;
    wy= fy*cosheading - fx*sinheading;
}



//世界坐标转机身坐标
void CMicroTaskExec::World2Flight(float cur_yaw, float wx, float wy, float& fx, float& fy)
{
    float heading= _heading_yaw - cur_yaw;
    if(heading<0.0) heading+=360.0;
    if(heading>=360.0) heading-=360.0;
    heading/=57.2958;

    float cosheading= cos(heading);
    float sinheading= sin(heading);

    //世界坐标转回机身坐标的 x,y
    fx= wx*cosheading - wy*sinheading;
    fy= wx*sinheading + wy*cosheading;
}


// //20240720+ 扩展版，附加航线方向的旋转量作用，使得世界转机身的推力指向再旋转一个量，而这个量并不参与_heading_yaw的调节，是独立的控制飞行方向的调节量。
// void CMicroTaskExec::World2Flight_Ext(float cur_yaw, float wx, float wy, float& fx, float& fy)
// {
//     //pin_rot是独立的调节飞行方向的量。用来旋转推力方向使方向稳定不偏。
//     float heading= _heading_yaw + pin_rot - cur_yaw;
//     if(heading<0.0) heading+=360.0;
//     if(heading>=360.0) heading-=360.0;
//     heading/=57.2958;

//     float cosheading= cos(heading);
//     float sinheading= sin(heading);

//     //世界坐标转回机身坐标的 x,y
//     fx= wx*cosheading - wy*sinheading;
//     fy= wx*sinheading + wy*cosheading;
// }

void CMicroTaskExec::World2Flight(float cosheading, float sinheading, float wx, float wy, float& fx, float& fy)
{

    //世界坐标转回机身坐标的 x,y
    fx= wx*cosheading - wy*sinheading;
    fy= wx*sinheading + wy*cosheading;
}

//oflow_position_adjust极微小调节,已适应旋转。
//思路是针对旋转，去调节锚点的坐标，即修改micro_task.oflow_sumx, sumy，这个点记录了锚定点的坐标值。
//把最初的针对yaw角记录下的锚点，转换为当前yaw角下的坐标值，这样或许能行。采用连续修改法。
//在tof基础数据处理结构里，新增一个Yaw记录。distH是测距高度,0=无效值。
//20230709修改，采用oflow_bias里的offset_sumx,sumy来记录总偏差，不使用micro_task里的总偏差记录
//这样方便进行历史偏差的坐标转换，可以往前追溯数个历史偏差进行转换，而不是只能盯住上一个历史偏差
//盯住上一个历史偏差的转换角度极小，累积起来可能误差较大。
//20230721 目前这个函数的光流控制比较好。取消不必要的消息和日志。
bool CMicroTaskExec::oflow_position_adjust_minor_change()
{

#if defined USE_OFLOW
    //光流的数据处理。
    //目前有60个数据，够用。
    float sum_a_x=0; //0-20
    float sum_a_y=0; //0-20
    float sum_b_x=0; //10-30
    float sum_b_y=0; //10-30
 
    size_t num_a=0;
    float quali_a=0;
    //float tofh=-1.0;    //测距高，在很低高度限制偏角，负数表示无效测距。
    tof_oflow_info last_tof;
    //tof_oflow_info secd_tof;
    tof_oflow_info head_tof;
    //通过计算两段数据，可以知道最近的速度和稍早的速度，可以计算出加速度。
    critical_section_enter_blocking(&tof_oflow_queue_section);
    last_tof= tof_oflow_queue.last();
    head_tof= tof_oflow_queue.last(30);

    for(size_t i=0;i<20;i++) {
        tof_oflow_info& ref= tof_oflow_queue.last(i);
        if(ref.oflow_valid) {
            sum_a_x+= ref.fixed_oflow_spdx;
            sum_a_y+= ref.fixed_oflow_spdy;
            num_a++;
            quali_a+= ref.oflow_quality;
        }
    }

    for(size_t i=10;i<30;i++) {
        tof_oflow_info& ref= tof_oflow_queue.last(i);
        if(ref.oflow_valid) {
            sum_b_x+= ref.fixed_oflow_spdx;
            sum_b_y+= ref.fixed_oflow_spdy;
        }
    }

    critical_section_exit(&tof_oflow_queue_section);

    //如果oflow数据中断了，自动调节就失效。
    //为了防范这种小概率失效，这里先判断数据流是否已经中断。
    uint32_t now = get_time_mark();
    if(now - last_tof.tmark > 20) {
        //oflow数据流中断了，不能自动调节了。
        micro_task.oflow_anchor_x=false;
        micro_task.oflow_anchor_y=false;
        oflow_bias ob;
        ob.tmark=last_tof.tmark;
        oflow_bias_queue.push(ob);
        return false;
    }
    else if(last_tof.tmark - head_tof.tmark > 300 + 30)
    {
        //这不是一个连续的数据流，中间肯定断了。正常30个数据相差300毫秒，这个大了太多，无效。
        micro_task.oflow_anchor_x=false;
        micro_task.oflow_anchor_y=false;
        oflow_bias ob;
        ob.tmark=last_tof.tmark;
        oflow_bias_queue.push(ob);
        return false;
    }
    else if(last_tof.tmark == oflow_bias_queue.last().tmark) {
        //重复上次计算出的偏角
        return true;
    }

    //tof测距失效 光流失效，已不合适用光流定点了。用GPS.
    if(!last_tof.tof_valid || !last_tof.oflow_valid) {
        micro_task.oflow_anchor_x=false;
        micro_task.oflow_anchor_y=false;

        oflow_bias ob;
        ob.tmark=last_tof.tmark;
        oflow_bias_queue.push(ob);
        return false;
    }


    if(num_a!=0) quali_a/=num_a;

    if(quali_a < 10.0)
    {
        //光流质量低。
        micro_task.oflow_anchor_x=false;
        micro_task.oflow_anchor_y=false;

        oflow_bias record;
        record.tmark= last_tof.tmark;
        oflow_bias_queue.push(record);
        return false;
    }

//锚点的打开和关闭控制。
    //摇杆居中，锚点未打开，速度很低，则打开锚点设置。 sum_seg2_x是30个数据的和，每数据最小值是0.2, 30个数据最小值是6.0
    //如果当前速度是1m/s，则光流的数值应该是1.0，sum_seg2_x应该是30.0
    bool adjust_x=true;
    bool adjust_y=true;

    oflow_bias record;
    
    if(fabs(micro_task.roll)<0.01 && !micro_task.oflow_anchor_x && fabs(sum_a_x) < 0.2 && fabs(sum_b_x) < 0.2) {

        micro_task.oflow_anchor_x=true;
        //清理部分历史记录，这样不会有残余的参考值，20230822+
        for(size_t i=0;i<10;i++) {
            oflow_bias_queue.last(i).offset_sumx=0;
            oflow_bias_queue.last(i).roll_bias=0;
        }
        //目前光流和GPS定点冲突，只要光流锚定起作用，就不能启用GPS锚定。
        micro_task.gps_anchor=false;
        //micro_task.oflow_sumx=last_tof.sumx;
        record.offset_sumx=0; //总偏记录现在改在这里。因为不使用光流处理的累计量，所以总偏需要自己累加。
    }
    else if(fabs(micro_task.roll)>=0.01 && micro_task.oflow_anchor_x) {
        micro_task.oflow_anchor_x=false;
        record.offset_sumx=0; //为了在打开锚点的瞬间取历史数据时避免取到非0数据，产生初始位移。
        adjust_x=false;
    }
    else if(fabs(micro_task.roll)>=0.01) {
        record.offset_sumx=0; //为了在打开锚点的瞬间取历史数据时避免取到非0数据，产生初始位移。
        micro_task.oflow_anchor_x=false;
        adjust_x=false;
    }

    if(fabs(micro_task.pitch)<0.01 && !micro_task.oflow_anchor_y && fabs(sum_a_y) < 0.2 && fabs(sum_b_y)<0.2) {

        micro_task.oflow_anchor_y=true;
        //清理部分历史记录，这样不会有残余的参考值，20230822+
        for(size_t i=0;i<10;i++) {
            oflow_bias_queue.last(i).offset_sumy=0;
            oflow_bias_queue.last(i).pitch_bias=0;
        }
        //目前光流和GPS定点冲突，只要光流锚定起作用，就不能启用GPS锚定。
        micro_task.gps_anchor=false;
        //micro_task.oflow_sumy=last_tof.sumy;
        record.offset_sumy=0; //总偏记录现在改在这里。因为不使用光流处理的累计量，所以总偏需要自己累加。
    }
    else if(fabs(micro_task.pitch)>=0.01 && micro_task.oflow_anchor_y) {
        micro_task.oflow_anchor_y=false;
        record.offset_sumy=0; //为了在打开锚点的瞬间取历史数据时避免取到非0数据，产生初始位移。
        adjust_y=false;
    }
    else if(fabs(micro_task.pitch)>=0.01) {
        record.offset_sumy=0; //为了在打开锚点的瞬间取历史数据时避免取到非0数据，产生初始位移。
        adjust_y=false;
        micro_task.oflow_anchor_y=false;
    }


    //这里的处理可以避免对imu不必要的访问。
    if(!adjust_x && !adjust_y) {
        //两个方向均有摇杆控制，不调节。
        micro_task.oflow_anchor_x=false;
        micro_task.oflow_anchor_y=false;
        //记录调节偏角的历史数据
        oflow_bias record;
        record.tmark= last_tof.tmark;
        oflow_bias_queue.push(record);
        return true;
    }


    // //imu水平加速度噪音可能较大，取多个数据平均。
    float imu_accx=0;
    float imu_accy=0; //y方向和光流方向相反
    critical_section_enter_blocking(&imu_queue_section);
    record.yaw=imu_queue.last().angle[2]; //填写当前的yaw角度。
    for(size_t i=0; i<15; i++)
    {
        imu_accx+= imu_queue.last(i).acc_gnd[0];
        imu_accy+= imu_queue.last(i).acc_gnd[1];
    }
    critical_section_exit(&imu_queue_section);


    float roll_ref=0;
    float pitch_ref=0;
    float offsetx_ref=0;
    float offsety_ref=0;

    //20230721测试了短周期70毫秒的参考范围，摆动比较大。不太合适。
    //再测试稍长周期的参考，200毫秒, 太长也不好。
    //uint32_t ts= now - 60;  //参考值应该可以缩短，太大了有记忆效应。原120，现改70，20230720+
    //因为轻微摇摆，调节做测试，原来60毫秒，改为110毫秒，20230808
    uint32_t ts= now - 80; //原110，缩短到80， 20230822
    uint32_t ref_cnt=0;
    for(size_t i=0;i<10;i++)
    {
        oflow_bias& ref= oflow_bias_queue.last(i);
        if(ref.tmark < ts) break;
        coordinate_trans_struct cts;
        cts.pitchA= ref.pitch_bias;
        cts.rollA=ref.roll_bias;
        cts.xA=ref.offset_sumx;
        cts.yA=ref.offset_sumy;
        cts.yawA=ref.yaw;
        cts.yawB=record.yaw; //目标角度及当前角
        coordinate_trans(cts); //将角度和总偏都转换到当前角度。

        roll_ref+= cts.rollB;
        pitch_ref+= cts.pitchB;
        offsetx_ref += cts.xB;
        offsety_ref += cts.yB;

        ref_cnt++;
    }

    if(ref_cnt) {
        roll_ref/=ref_cnt;
        pitch_ref/=ref_cnt;
        offsetx_ref/=ref_cnt;
        offsety_ref/=ref_cnt;
    }

    record.offset_sumx=offsetx_ref;
    record.offset_sumy=offsety_ref;

    FLOAT_LIMIT(record.offset_sumx, -10.0, 10.0);
    FLOAT_LIMIT(record.offset_sumy, -10.0, 10.0);

    record.pitch_bias=pitch_ref;
    record.roll_bias=roll_ref;

    //处理完历史总偏的旋转，累加当前的偏离。
    if(last_tof.oflow_valid)
    {
        if(micro_task.oflow_anchor_x) record.offset_sumx+= last_tof.fixed_oflow_spdx*0.01;
        if(micro_task.oflow_anchor_y) record.offset_sumy+= last_tof.fixed_oflow_spdy*0.01;
    }

//目前这个参数感觉偏软了点。0.2->0.3   20230710调整。
#define RATIO_OFLOW (0.3)
    if(adjust_x)
    {
        //首先控制速度，降低速度，如果有锚点，调节锚点距离。
        //测量时间较短，速度和加速度都有很大噪音。现在增加了初级滤波，可能会好点。
        float accx= (sum_a_x*5.0 - sum_b_x*5.0)*10.0; //当前加速度，每秒变化量，>0加速度朝x轴正方向。
        float spdx= sum_a_x*5.0; //当前秒速度。厘米/秒，通过Imu校准，摇摆产生的噪音速度控制在10左右。
        float want_spd = 0; //期望速度。没有锚点时期望为0
        
        if(micro_task.oflow_anchor_x)
        {
            //有锚点设置，需要有回归速度。
            //float dist = last_tof.sumx - micro_task.oflow_sumx;  //总距偏差，>0 机身偏向右侧，需要左移
            float dist = last_tof.sumx; //20230725+ 这里应该和micro_task.oflow_sumx没有关系了，总偏就是记录在调节序列里。
            FLOAT_LIMIT(dist, -1.5, 1.5);
            want_spd = -dist/2.0; //2秒回归原位的速度。
        }

        float want_accx= want_spd - spdx; //期望一秒变化到期望速度的加速度。

        float acc_difx= 2.0/(1+exp(accx - want_accx)) -1.0; //期望加速度和当前加速度的差，如果是负值，则调高右侧降低左侧
        //增加速度阻尼后似乎更好。
        float spdx_trans= 2.0/(1+exp(-spdx/20.0)) -1.0; 
        //这里写错了，但表现并不差。 本意是 float acc_trans= 2.0/(1+exp(-accx/20.0)) -1.0; 
        //这玩意总是返回正数，介于0-1之间，和本意完全不相关。
        //明天先取消此项调节，后改为 2.0/(1+exp(-accx)) -1.0; 
        //float acc_trans= 2.0/(1+exp(-accx)/20.0) -1.0; 
        float acc_trans= 2.0/(1+exp(-accx/3.0)) -1.0; 

        //期望加速度调节项，可以小一点慢慢调节。
        record.roll_bias += acc_difx*0.3*RATIO_OFLOW; 
        //单纯光流检测的加速度阻尼。
        //机身摇摆时，因为校准误差，会引起测量误差，可能导致加速度波动较大，引起水平摆动
        record.roll_bias -=  acc_trans*0.5*RATIO_OFLOW;//0.1
        //单纯速度阻尼。
        record.roll_bias -= spdx_trans*RATIO_OFLOW;
        //单纯imu加速度阻尼。
        record.roll_bias -= imu_accx*RATIO_OFLOW; //抬高右侧，roll减小。
    }

    if(adjust_y)
    {
        //首先控制速度，降低速度，如果有锚点，调节锚点距离。
        float accy= (sum_a_y*5.0 - sum_b_y*5.0)*10.0; //每秒变化量，
        float spdy= sum_a_y*5.0; //当前秒速度。
        float want_spd = 0; //期望速度。

        if(micro_task.oflow_anchor_y)
        {
            //有锚点设置，需要有回归速度。
            //float dist = last_tof.sumy - micro_task.oflow_sumy;  //总距偏差，>0 机身偏向后侧，需要前移
            float dist = last_tof.sumy; //20230725+ 这里应该和micro_task.oflow_sumx没有关系了，总偏就是记录在调节序列里。
            FLOAT_LIMIT(dist, -1.5, 1.5);
            want_spd = -dist/2.0; //2秒回归原位的速度。

        }
        
        float want_accy= want_spd - spdy; //期望一秒变化到期望速度的加速度。

        float acc_dify= 2.0/(1+exp(accy - want_accy)) -1.0; //期望加速度和当前加速度的差，如果是负值，则调高后侧降低前侧
        //增加速度阻尼后似乎更好。
        float spdy_trans= 2.0/(1+exp(-spdy/20.0)) - 1.0;
        //这里写错了，但表现并不差。 本意是 float acc_trans= 2.0/(1+exp(-accx/20.0)) -1.0; 
        //这玩意总是返回正数，介于0-1之间，和本意完全不相关。
        //float acc_trans= 2.0/(1+exp(-accy)/20.0) -1.0;
        float acc_trans= 2.0/(1+exp(-accy/3.0)) -1.0;

        //20230702，现在由于光流数据处理把机身向前移动的读数修改为正数，原先是负数。所以关于光流测量的y方向这里都要反向。

        //加速度调节项，可以小一点慢慢调节。
        record.pitch_bias -= acc_dify*0.3*RATIO_OFLOW; //反向

        //单纯光流检测的加速度阻尼。
        //机身摇摆时，因为校准误差，会引起测量误差，可能导致加速度波动较大，引起水平摆动
        record.pitch_bias += acc_trans*0.5*RATIO_OFLOW;//0.1  反向
        //单纯速度阻尼。
        record.pitch_bias += spdy_trans*RATIO_OFLOW; // 反向
        //单纯imu加速度阻尼。
        record.pitch_bias += imu_accy*RATIO_OFLOW;   //抬高前面，Pitch增加。
    }

#undef RATIO_OFLOW

    //限幅
    if(quali_a<20.0)
    {
        FLOAT_LIMIT(record.roll_bias, -3.0, 3.0);
        FLOAT_LIMIT(record.pitch_bias, -3.0, 3.0);
    }
    else if(quali_a<40.0)
    {
        FLOAT_LIMIT(record.roll_bias, -5.0, 5.0);
        FLOAT_LIMIT(record.pitch_bias, -5.0, 5.0);
    }
    else if(quali_a<80.0)
    {
        FLOAT_LIMIT(record.roll_bias, -8.0, 8.0);
        FLOAT_LIMIT(record.pitch_bias, -8.0, 8.0);
    }
    else if(quali_a<120.0)
    {
        FLOAT_LIMIT(record.roll_bias, -10.0, 10.0);
        FLOAT_LIMIT(record.pitch_bias, -10.0, 10.0);
    }
    else
    {
        FLOAT_LIMIT(record.roll_bias, -12.0, 12.0);
        FLOAT_LIMIT(record.pitch_bias, -12.0, 12.0);
    }

    //关于高度的倾角限制, 近地面光流数据质量不好。 20230625+
    if(last_tof.tof_valid)
    {
        if(last_tof.fixed_tof_distance < 0.2)
        {
            FLOAT_LIMIT(record.roll_bias, -3.0, 3.0);
            FLOAT_LIMIT(record.pitch_bias, -3.0, 3.0);            
        }
        else if(last_tof.fixed_tof_distance < 0.4)
        {
            FLOAT_LIMIT(record.roll_bias, -6.0, 6.0);
            FLOAT_LIMIT(record.pitch_bias, -6.0, 6.0);  
        }

    }


    //现在里面记录了Yaw, offsetx,y
    record.tmark= last_tof.tmark;
    oflow_bias_queue.push(record);

    return true;

#else //if defined USE_OFLOW
    return false;
#endif    
}



int CMicroTaskExec::switch_fixpoint_mod()
{
    micro_task.position_keep_switch= (micro_task.position_keep_switch+1)%3;
    //锚点修改权输微任务所有，外部的宏任务不干涉。
    // //发生了定点切换，所有锚点要清空
    // micro_task.gps_anchor=false;
    // micro_task.oflow_anchor_x=false;
    // micro_task.oflow_anchor_y=false;
 
    return micro_task.position_keep_switch;
}

//完全遥控指令下的执行。高度盯住气压，姿态盯住设置。最后还有总力的限制。
void CMicroTaskExec::micro_task_on_remote()
{

    if(micro_task.status == _micro_task_status_shutdown)
    {
        //这个状态下，飞机已经落地停机。
        return; 
    }

    micro_task.status=_micro_task_status_inprogress;

    // if(micro_task.status==_micro_task_status_none)
    // {
    //     micro_task.help_takeoff=false; //20231215+
    //     micro_task.status=_micro_task_status_inprogress;
    // }
    // else if(micro_task.status == _micro_task_status_remote_done)
    // {
    //     //这个状态下，飞机已经落地停机。
    //     return; 
    // }


    //现在使用以气压为主，tof辅助的控高，应该不用担心tof数据错误导致的冲高。
   // height_adjust_by_baro_with_tof(1.2, INAIR_TOPTHRUST);

    //bool new_height_thrust;

    bool landed=false;
    // if(micro_task.height_keep_switch)
    // {
    //     //height_adjust_by_baro(INAIR_BOTTHRUST, INAIR_TOPTHRUST); //高空控制型，只用气压
    //     hyper_height_adjust_by_baro(INAIR_BOTTHRUST, INAIR_TOPTHRUST);
    // }
    // else
    // {
    //     height_adjust(INAIR_BOTTHRUST, INAIR_TOPTHRUST);   //通用型，先测距后气压
    // }

    //uint64_t tk1=get_time_us_mark(); //for debug

    bool hadj=height_adjust_with_landing_detect(INAIR_BOTTHRUST, INAIR_TOPTHRUST, landed);
    if(hadj && landed) {
        motor_ctrl.SetBurdens(0,0,0,0,0);
        micro_task.status=_micro_task_status_shutdown;
        _inair=false; //全局空中标记
        return;
    }

    float pb=0;
    float rb=0;

#if defined USE_GPS && defined USE_OFLOW
    if(micro_task.position_keep_switch==1)
    {
        bool b0=gps_position_adjust(); 

        if(b0){
            pb= gps_major_bias_queue.last().pitch_bias;
            rb= gps_major_bias_queue.last().roll_bias;

        }else{
            Send_remote_message(41);
        }
    }
    else if(micro_task.position_keep_switch==0)
    {
        bool b0=oflow_position_adjust_minor_change(); 

        if(b0){
            pb= oflow_bias_queue.last().pitch_bias;
            rb= oflow_bias_queue.last().roll_bias;
        }
        
    }
#elif defined USE_GPS && !defined USE_OFLOW
        //仅GPS水平定位。
    //old 
    bool b0=gps_position_adjust(); 
    if(b0){
        pb= gps_major_bias_queue.last().pitch_bias;
        rb= gps_major_bias_queue.last().roll_bias;
    }

    //考虑修改合并代码，取消gps_adjust函数。
    // if(fabs(micro_task.roll)>0.01 || fabs(micro_task.pitch)>0.01)
    // {
    //     //取消锚点，冲刷历史记录。
    //     micro_task.gps_anchor=false;
    //     pb=0;
    //     rb=0;
    // }


    // if(!micro_task.gps_anchor) {
    //    bool b0=gps_position_brake();
    //    if(b0) {
    //         pb= gps_major_bias_queue.last().pitch_bias;
    //         rb= gps_major_bias_queue.last().roll_bias;
    //    }else{
    //         pb=0;
    //         rb=0;
    //    }
    // }
    // else
    // {
    //     float cur_dist, cur_spd, cur_dir, cur_height;
    //     bool newdata;
    //     pin_to_location(cur_dist, cur_spd, cur_dir, cur_height, newdata);
    //     pb= gps_major_bias_queue.last().pitch_bias;
    //     rb= gps_major_bias_queue.last().roll_bias;
    // }

#elif defined USE_OFLOW && !defined USE_GPS
        //未使用GPS，但使用光流
        bool b0=oflow_position_adjust_minor_change(); 
        if(b0){
            pb= oflow_bias_queue.last().pitch_bias;
            rb= oflow_bias_queue.last().roll_bias;
        }
#endif

    //uint64_t tk2=get_time_us_mark(); //for debug

    float new_burdens[4];
    float new_thrust;
    if(fabs(micro_task.pitch)>0.01||fabs(micro_task.roll)>0.01)
    {
        balance(new_burdens, new_thrust, micro_task.pitch, micro_task.roll);
    }
    else
    {
        balance(new_burdens, new_thrust,  pb, rb);
    }

    //uint64_t tk3=get_time_us_mark(); //for debug
    //printf("%u,%u\n", tk3-tk1, tk3-tk2);  //for debug
    motor_ctrl.SetBurdens(new_burdens, new_thrust);

}


//t:任务代码及任务参数
//gi:姿态数据
void CMicroTaskExec::do_micro_task()
{
    
    switch (micro_task.taskid)
    {
    case _micro_task_id_nop:
        break;
    case _micro_task_id_ground_up:
        micro_task_ground_up();
        break;
    case _micro_task_id_vertical_landing:
        micro_task_vertical_landing();
        break;
    case _micro_task_id_keep_stable_withyaw:
        micro_task_keep_stable_with_yaw();
        break;
    case _micro_task_id_move_ahead_withyaw:
        micro_task_move_ahead_with_yaw();
        break;
    case _micro_task_id_move_around:
        micro_task_move_around();
        break;
    case _micro_task_id_idle:
        micro_task_idle();
        break;
    case _micro_task_id_on_remote:
        {
            //uint64_t s=get_time_us_mark(); // for debug
            micro_task_on_remote();
            // uint64_t e=get_time_us_mark(); // for debug
            // //记录执行时间
            // time_debug_queue.push_back(float(e-s)/1000.0f);

            // if(time_debug_queue.size()>=250) {
            //     //输出信息，平均耗时及最大耗时。
            //     auto minv = std::min_element(time_debug_queue.begin(), time_debug_queue.end());
            //     auto maxv = std::max_element(time_debug_queue.begin(), time_debug_queue.end());
            //     float m= std::accumulate(time_debug_queue.begin(), time_debug_queue.end(), 0.0)/time_debug_queue.size();
            //     printf("minv= %f, maxv=%f, mean=%f\n", *minv, *maxv, m);
            //     time_debug_queue.clear();
            // }
        }
        break;
    case _micro_task_id_vertical_landing_location:
        micro_task_vertical_landing_target_location();
        break;
    case _micro_task_id_pin_to_location:
        micro_task_pin_to_location();
        break;
    default:
        break;
    }
}



//坐标转换。用于光流的旋转稳定。
//yawB是要转到的坐标系，yawA是基础系
void CMicroTaskExec::coordinate_trans(coordinate_trans_struct& trans)
{
    //yawA逆时针旋转delta得到yawB, 从yawA->yawB的转换应该是
    //x'=x*cos(delta) + y*sin(delta)
    //y'=y*cos(delta) - x*sin(delta)

    float delta= trans.yawB - trans.yawA;
    if(delta<0) delta+=360;
    delta/=57.2958;

    float cosdelta=cos(delta);
    float sindelta=sin(delta);

    trans.xB = trans.xA*cosdelta + trans.yA*sindelta; 
    trans.yB = trans.yA*cosdelta - trans.xA*sindelta;

    //如果把roll, -pitch 分别看作是坐标点，则直接用坐标转换，在小角度时应该误差不大。

    trans.rollB = trans.rollA*cosdelta - trans.pitchA*sindelta;
    trans.pitchB =  trans.pitchA*cosdelta + trans.rollA*sindelta;
}




//GPS速度可信度，0.1-1.0
float CMicroTaskExec::gps_speed_reliable(float gps_spd)
{
    
    float sr=1.0; //速度信度。
    if(gps_spd<0.08) {
        sr=0.1;
    }else if(gps_spd<0.12) {
        sr=0.15;
    }else if(gps_spd<0.18) {
        sr=0.2;
    }else if(gps_spd<0.26) {
        sr=0.3;
    }else if(gps_spd<0.36) {
        sr=0.4;
    }else if(gps_spd<0.48) {
        sr=0.5;
    }else if(gps_spd<0.62) {
        sr=0.6;
    }else if(gps_spd<0.78) {
        sr=0.7;
    }else if(gps_spd<0.96) {
        sr=0.8;
    }else if(gps_spd<1.2) {
        sr=0.9;
    }
    return sr;
}

float CMicroTaskExec::gps_dist_reliable(float gps_dist, float hacc)
{

    float x= gps_dist/hacc;
    if(x < 0.4) {
        return 0.2;
    }else if(x < 0.5) {
        return 0.25;
    }else if(x < 0.6) {
        return 0.3;
    }else if(x < 0.7) {
        return 0.35;
    }else if(x < 0.8) {
        return 0.4;
    }else if(x < 0.9) {
        return 0.5;
    }else if(x < 1.0) {
        return 0.6;
    }else if(x<1.1) {
        return 0.7;
    }else if(x<1.2) {
        return 0.8;
    }else if(x<1.3) {
        return 0.85;
    }else if(x<1.4) {
        return 0.9;
    }else if(x<1.5) {
        return 0.95;
    }else {
        return 1.0;
    }

}

// 计算实时垂直速度（滑动窗口最小二乘法）
//这是一个deepseek提供的计算垂直速度的方法，有改动。
//50hz采样，取最近5个数据计算，返回正值是向上速度m/s
//经过比较，这样算出来的速度波动反而比较大，还是老办法好。
float CMicroTaskExec::calculate_baro_vertical_speed_50hz_n5() 
{

    // 提取最近5个点（约0.1秒窗口）
    float time[5]={0.0,0.02,0.04,0.06,0.08};
    float press[5];
    critical_section_enter_blocking(&press_queue_section);
    for(size_t i=0;i<5;i++)
    {
        press[i]=press_queue.last(i).press;
    }
    critical_section_exit(&press_queue_section);
    

    // 最小二乘拟合
    float sum_t = 0.2, sum_t2 = 0.012, sum_at = 0.0, sum_a = 0.0;
    for (size_t i=0; i<5; ++i) {
        //sum_t += time[i];
        //sum_t2 += time[i] * time[i];
        sum_at += press[i] * time[i];
        sum_a += press[i];
    }
    
    //float denominator = n * sum_t2 - sum_t * sum_t;
    //if (denominator == 0.0) return 0.0;
    
    return (5 * sum_at - sum_t * sum_a) / 0.02 /11.7;  // 斜率即速度
}

//气压降落，在检测到异常震动时开启降落过程，开启持续减力到0（需要速度持续向下），最后标记降落。
//暂时永不返回降落标记。因为气压检测落地较难，如果是缓降，可能检测不到震动。如果从垂直速度着手，需要一定时间
//判断，而这段时间可能GPS水平定位发生变化，导致机身倾角变化，则落地后机身倾斜。
//20240228+ 新增侧翻保护，侧翻激活自动降落过程，3秒停机。
//返回true, 函数给出新的推力调节，并给出是否降落的标记信号。
//返回false, 没有推力调节数据，降落标记不可信。
bool CMicroTaskExec::height_adjust_by_baro_with_landing_detect(float botlmt, float toplmt, bool& landed)
{

    press_info pr0, pr1, pr2, pr3;//, pr4, pr5; //采样2阶段气压数据，可求出气压速度。
    press_info praccu; //用于积分
    uint32_t dtime; //数据时间
    uint32_t tgap; //20230619+ 一个气压数据时间间隔。毫秒 >0
    critical_section_enter_blocking(&press_queue_section);
    dtime=press_queue.last().tmark;
    //此函数调用频率同imu数据频率，但气压数据频率很低，多数个调用只需要一次处理，处理过的气压数据就无需继续处理。
    //在此过滤处理过的气压数据节约后面的计算时间。20250227+
    if(_height_adjust_queue.last().tmark==dtime) {
        critical_section_exit(&press_queue_section);
        return false;
    }
    tgap= dtime - press_queue.last(5).tmark; //修改为5个数值间隔。
    pr0= press_queue.history_mean(0, 5);  //当前气压值，5均值。
    pr1= press_queue.history_mean(5, 5); //0.1s
    pr2= press_queue.history_mean(10, 5); //0.2s
    pr3= press_queue.history_mean(25, 5); //0.5s
    praccu= press_queue.history_mean(0, 40); //40个数据积分 
    critical_section_exit(&press_queue_section);



    // if(_height_adjust_queue.last().tmark==dtime) {
    //     //如有必要，这期间可以穿插处理。可以观察imu, tof.
    //     landed=false;
    //     return false;
    // }

    //如果是起飞，先搞到最低发力。
    if(micro_task.help_takeoff && _height_adjust_queue.last().thrust < botlmt)
    {
        height_adjust_record har;
        har.tmark=dtime;

        har.thrust=_height_adjust_queue.last().thrust + 0.01;

        har.hd= (pr0.press - micro_task.press)/11.719;
        har.current= system_power_current;
        har.spd=0;

        _height_adjust_queue.push(har);

        //起飞阶段不修改气压，是设定气压。
        micro_task.press= micro_task.help_takeoff_target_press;
        micro_task.tof_dominate=false;
        landed=false;
        return true;
    }

#if defined USE_TOF
    //控高模式切换检查。
    if(micro_task.tof_dominate)
    {
        //这是由tof切换到气压控高来。先前设定的气压高度作废，重新设置。
        micro_task.press=pr0.press;
        critical_section_enter_blocking(&gps_queue_section);
        micro_task.gps_ref_height=gps_queue.last(0).height;
        critical_section_exit(&gps_queue_section);
        //micro_task.gps_ref_height= gps_queue.last(0).height; //记录气压时，同时记录GPS高度。
        micro_task.gps_height_press_bias=0;

        if(micro_task.help_takeoff) 
        {
            micro_task.press= micro_task.help_takeoff_target_press; //起飞阶段盯住设定气压。
        }

        micro_task.tof_dominate=false;

        //刚从tof控高切换过来，时间间隔基于历史记录的时间。影响非常的小，可以忽略，如果是忽略，则以气压数据间隔调节高度。
        tgap= dtime - _height_adjust_queue.last().tmark; 

        //TOFSense-F2 Mini 可能确实有稳定问题，在高空常有这个信号出现，这不正常。
        // char log[128];
        // sprintf(log, "t2b, p=%6.0f, gps_h=%5.1f\n", micro_task.press, micro_task.gps_ref_height);
        // logmessage(log);
    }
#endif



    float zaxis, vib_z;
    float imuvacc= get_refer_imu_vacc_for_height_control(zaxis, vib_z); //20240201+

    //三段式，检查是否取消平滑降落，执行平滑降落，是否进入平滑降落。
    if(micro_task.smooth_landing && micro_task.vspeed < 0)
    {
        micro_task.smooth_landing=0; //关闭降落过程因请求速度向上。
        micro_task.smooth_landing_triger_reason=0;
        logmessage("cancel smoothlanding for vspeed <0\n");
    }
    else if(micro_task.smooth_landing && micro_task.smooth_landing_triger_reason==3 && zaxis >0)
    {
        //因机身侧翻被纠正，之前因侧翻激活的降落被取消。
        //20250406 侧翻落地，保持肚皮向上，激活smooth_landing后会立即从这里退出
        micro_task.smooth_landing=0;
        micro_task.smooth_landing_triger_reason=0;
        char dbg[128];
        sprintf(dbg, "cancel upside down smoothlanding, zaxis=%f, triger_reason==3\n", zaxis);
        logmessage(dbg);
    }
    else if(micro_task.smooth_landing && micro_task.smooth_landing_triger_reason==1)
    {
        //tof测距打开了平滑降落，控制权却转移到了气压控高，这不合理。小距离降落必须全程在tof测距下完成。
        //一旦到了气压控高一定不对。
        //20240229发生的十几米高的平滑降落也许是一个tof误测距激发的，随后交给气压控高执行，而这里没有防备这种情况导致高空停机。
        micro_task.smooth_landing=0;
        micro_task.smooth_landing_triger_reason=0;
        logmessage("cancel smoothlanding set by tof\n");
    }

    //气压速度和气压加速度都非常不准，尤其是在电机发力变化很大的情况下，空气被扰动影响气压值，形成负反馈，所以sigmoid变化不可少。
    //修改了tgap，现在是5个数据间隔 20240223
    float pspd0 = (pr0.press - pr1.press)*(1000.0/tgap)/dynamic_air_press_height_ratio; //m/s 下为正
    float pspd1 = (pr1.press - pr2.press)*(1000.0/tgap)/dynamic_air_press_height_ratio; //m/s 下为正 (pr1.press - pr2.press)才对
    float pspd2 = (pr0.press - pr3.press)*(200.0/tgap)/dynamic_air_press_height_ratio; //m/s 下为正 25个数据时差，0.5秒, 用来设定速度的调节


    if(micro_task.smooth_landing)
    {

        //执行平滑降落并返回。
        float ref_thrust=_height_adjust_queue.last().thrust; //这里不使用过去平均推力了。

        //已经减力了多久？减力持续了1.5秒，直接停机。
        uint32_t tt= dtime - micro_task.smooth_landing;

        if(tt>=1500 || ref_thrust <= 0.5) {

            landed=true;
            _inair=false;
            char logmsg[128];
            sprintf(logmsg, "smooth landing done, reason=%d\n", micro_task.smooth_landing_triger_reason);
            logmessage(logmsg);

            height_adjust_record har;
            har.tmark= dtime;
            har.thrust = 0;
            har.hd = 0;
            har.spd = pspd2;
            har.current= system_power_current;
            _height_adjust_queue.push(har);
            return true;

        }else{

            //还有多少时间可以用来减力？目标是在余下的时间减小到0.5;
            uint32_t tl= 1500 - tt; //毫秒，剩余的时间。 >0
            float td = ref_thrust - 0.5; //要减少的力度  >0
            uint32_t gp= dtime - _height_adjust_queue.last().tmark; //本次调节与上次调节的时间间隔 >0 
            float adj = td * gp/tl; //本次调节。
            FLOAT_LIMIT(adj, 0.001, 0.2); //限制最小调节量。

            landed=false;
            height_adjust_record har;
            har.tmark= dtime;
            har.thrust = ref_thrust - adj;
            har.hd = 0;
            har.spd = pspd2;
            har.current= system_power_current;
            _height_adjust_queue.push(har);
            return true;
        }   
    }




    //如果设定了垂直速度，这里首先调节设定高度。
    //目前高度方向的运动主要靠速度匹配，不靠高差拉动。
    float flex=0;

    //由非零速度进入到0速度，当前实际没有做任何处理，会导致高度摆动。
    //如果是检测到归0的速度，可以进入刹车状态，在速度降低后再定高，则可减少摆动。
    if(micro_task.vspeed!=0 && !micro_task.help_takeoff)  //vspeed现在有向，>0速度向下。
    {
        //在有速度时，设定高度追随当前气压。附加一定的速度因子防止突然停止导致的震荡。
        //在有速度时，调节高差设置为0，只盯住速度去调节。
        micro_task.press = pr0.press + micro_task.vspeed*1.2; //现在新增刹车标记处理，不需要跟随设置稳定气压，但为防止意外而保留。
        flex = SigmoidTrans(fabs(micro_task.vspeed*0.1));
    }



    //gps调节静态高差，部分参与高度控制，但不完全信任GPS高度数据，可避免受干扰。
    if(micro_task.vspeed==0 && micro_task.height_break_tmark==0 && dtime - micro_task.gps_height_press_bias_last_update > 400 )
    {

        float gps_h;
        float gps_vacc;
        uint32_t gps_tmark;
        critical_section_enter_blocking(&gps_queue_section);
        gps_h= (gps_queue.last().height + gps_queue.last(1).height + gps_queue.last(2).height )/3.0;
        gps_vacc=gps_queue.last().vacc;
        gps_tmark=gps_queue.last().tmark;
        critical_section_exit(&gps_queue_section);

        if(gps_vacc < 1.2 && gps_tmark + 1000 > dtime) //限制高度精度在较小的范围内
        {
            //真高，如果真高很小，调节不适合太大。之前没考虑到这个问题，在高海拔起飞时会有这个问题。
            //应当限制在真高3%以内。
            float realh= gps_h - initial_gps_info.height; 
            if(realh<1.0) realh=1.0;
            float plmt= realh*dynamic_air_press_height_ratio*0.03; //气压调节范围

            float gps_press_bias= (gps_h - micro_task.gps_ref_height)*dynamic_air_press_height_ratio;//这里差值放大2.0倍可以调节范围更大。
            

            //这种调节能力受到一定限制，比如GPS显示目前高于设定高度0.5米，给出气压调节量5.6
            //但当前气压值却大于设定气压，大的量超出了1.5米高度，这两者就矛盾了，然后偏移量调节过后
            //GPS的高度依旧不能匹配到。即受调节量大小的限制，不能进一步调节高度。
            //如果是调节量本身不受限制，发现当前GPS高度高于设定或低于设定就累加累减调节量，那就可以
            //完全调节气压偏差。但可能受GPS数据影响较大。

            //避免一次调节过大，降低波动性
            if(gps_press_bias - micro_task.gps_height_press_bias > 3.0)
            {
                micro_task.gps_height_press_bias+=3.0;
            }
            else if(gps_press_bias - micro_task.gps_height_press_bias < -3.0)
            {
                micro_task.gps_height_press_bias-=3.0;
            }
            else 
            {
                micro_task.gps_height_press_bias= gps_press_bias;
            }

            //总后限制调节总值。一个基于GPS数据，一个基于气压数据。
            FLOAT_LIMIT(micro_task.gps_height_press_bias, -plmt, plmt); 

            float pdif= fabs(pr0.press - initial_press_info.press); //距离起飞位置的大致高度差。
            float adjlmt= 10.0 + pdif*0.04; //gps可调节量是气压差的4%+10.0
            FLOAT_LIMIT(micro_task.gps_height_press_bias, -adjlmt, adjlmt); //用于调节的量在高差大时变大，高差小时变小。

        }
        micro_task.gps_height_press_bias_last_update=dtime;
    }

    //刷新高度和气压的关系系数。这个系数影响垂直速度的计算。
    //不同的海拔高度下，每米气压变化不同。
    if(dtime - micro_task.air_press_height_relation_last_update > 3000)
    {
        float gps_h;
        float gps_vacc;
        uint32_t gps_tmark;
        critical_section_enter_blocking(&gps_queue_section);
        gps_h= gps_queue.last().height;
        gps_vacc=gps_queue.last().vacc;
        gps_tmark=gps_queue.last().tmark;
        critical_section_exit(&gps_queue_section);

        //是最近的数据且精度可以。
        if(gps_vacc < 5.0 && gps_tmark + 1000 > dtime) //限制高度精度在较小的范围内
        {
            dynamic_air_press_height_ratio=air_press_height_relation(gps_h);
            micro_task.air_press_height_relation_last_update=dtime;
        }
    }

    //高差
    float hd; 
    //高差积分
    float accuhd;

    if(micro_task.vspeed!=0) {
        hd=0; //有速度时，高差为0，不使用高差去调节。
        accuhd=0;
        micro_task.height_break_tmark=0;
        micro_task.gps_height_press_bias=0;
    }else{
        hd = (pr0.press - micro_task.press - micro_task.gps_height_press_bias)/11.719; //气压指示的距离设定高度的高度差。>0 表示低于设定高度.
        accuhd = (praccu.press - micro_task.press - micro_task.gps_height_press_bias)/11.719;
        //之前将高差限制的极小，当Imu加速度有偏时，高度就不能匹配，因Imu阻尼设置的较大。
        FLOAT_LIMIT(accuhd, -5.0, 5.0);
        FLOAT_LIMIT(hd, -12.0, 12.0); //高差不能限制的太小，否则长期下来容易掉高，太小的调节可能被其他调节抵消，导致长期高度失配。比如imu加速度的微小偏差。之前限制在0.5
    }

    //气压指示的加速度。这个可以去和imu加速度比较，如果是近似的就相信，如果差别很大，就否定。
    float pacc =  (pspd0 - pspd1)*(1000.0/tgap); //m/s^2 下为正


    if(micro_task.height_break_tmark)
    {
        //在刹车过程中，检查是否刹车过程结束，如果结束就设定目标气压。
        //也许此过程会延续到tof测距控高，所以那边也需要处理。
        //刹车阶段取消高差调节，即不使用稳定高度。有imu阻尼，气压阻尼，速度调节。最终速度会回到0.
        hd=0;
        accuhd=0;

        //这么处理主要是防止机身抖动剧烈，imuvacc有持续加速度无法降低，然后无法锚定高度。
        //无法锚定高度时，则无法限制机身抖动对高度的影响，可能持续飞高。
        float thres=0.2;
        if(dtime < micro_task.height_break_tmark + 3000) thres = 0.15;
        else if(dtime < micro_task.height_break_tmark + 6000) thres=0.20;
        else if(dtime < micro_task.height_break_tmark + 9000) thres=0.25;
        else thres=0.3;
        //刹车标记。vspeed一定为0，此时调节倾向于抑制速度加速度。也有临时的稳定气压用来高差调节，然后速度和加速度降低到一定程度时，设置稳定气压。
        if(fabs(pspd2)<thres && fabs(imuvacc) < thres && fabs(pacc) < thres)
        {
            micro_task.press=pr0.press; //设置当前稳定气压
            micro_task.height_break_tmark=0; //取消刹车标记。
            //记录气压的同时记录GPS高度。
            critical_section_enter_blocking(&gps_queue_section);
            micro_task.gps_ref_height=gps_queue.last(0).height;
            critical_section_exit(&gps_queue_section);

            micro_task.gps_height_press_bias=0;
        }
        
        if(micro_task.vspeed) micro_task.height_break_tmark=0; //20240702+ 可以没有但有更保险。
    }

    //限制气压速度减少气压波动干扰。
    //20230726 适度放开限制，原来5，现在7
    //20240508 放开限制，原来7，现在8，新增pspd2限制
    FLOAT_LIMIT(pspd0, -10.0, 11.5); //下为正，降落速度较快，放松点。
    FLOAT_LIMIT(pspd1, -10.0, 11.5);
    FLOAT_LIMIT(pspd2, -10.0, 11.5);

    FLOAT_LIMIT(pacc, -3.0, 3.0); //限制范围同imu acc

    float takeoff_const_thrust=0; //如果是起飞阶段，增加一个常量给推力。

    if(micro_task.help_takeoff)
    {

        //在离地后4个气压内都保持持续加力，避免受地面效应干扰。随后进入正常调节模式。
        //设定起飞高度应该至少大于4个气压高度。这里也可以强制利用tof来判断持续加力范围。
        if(pr0.press - micro_task.help_takeoff_target_press > 11.719 )
        {
            //距离目标值超过1米，
            takeoff_const_thrust = 0.02;
        }
        else if(pr0.press - micro_task.help_takeoff_target_press > 8.0 )
        {
            takeoff_const_thrust = 0.016;
        }
        else if(pr0.press - micro_task.help_takeoff_target_press > 6.0 )
        {
            takeoff_const_thrust = 0.012;
        }
        else if(pr0.press <= micro_task.help_takeoff_target_press + 0.1)
        {
            micro_task.help_takeoff=false; //已经辅助增力到位，关闭辅助推力，以后是遥控器控制高低。
            _inair=true; //标记已在空中。
        }

    }


//原0.15，因为amax电机的起飞油门太低，大概在1.0左右，所以调小这个比例，以降低高度的抖动性。
//对于新西达kv1100电机，参数小了上下波动偏大。kv值小的电机，这里要稍大才能使高度更稳定。
//#define RATIO (0.15) 
//#define RATIO (0.12) 
//不能有太大的加速度和速度，向上和向下的加速度和速度都不能大。所以速度和加速度的调节参数较大。
//单独的kp导致上下跳跃更大。
//20230725 kp原来0.12，改0.13，希望对高差更敏感点，侧飞目前掉高明显。
//20240909 由0.16改0.1，2806电机太强
#define Kp (0.06*BARO_HEIGHT_RATIO) //高差调节系数，高差来源于气压的直接测量
#define Ki (0.05*BARO_HEIGHT_RATIO) //高差积分系数
//取消这个微分调节，那么第二层微分调节要加强。
//下一步尝试增强这个阻尼，看效果。基础量 0.2，适度增强是可行的方案，0.8似乎偏强了，高度变化突兀，考虑0.4-0.5左右
//似乎调大了这个值导致无法降落？需要研究！20230401晚上出现过一次无法降落，20230402早起复现没有成功。
//这个左右阻尼作用大不大，有没有必要设置很大的参数，需要对比测试。
//大的kd需要大的kx来压制，否则容易过头
#define Kd (0.08*BARO_HEIGHT_RATIO) //气压微分调节，阻尼性质。在气压控制情况下可能是有自反馈现象，所以这个值不能大。
//降低这个参数显著增加高度的波动性。0.5在冲顶时容易吸附。0.3能工作，稍显偏小。0.4似乎合适。
//有时0.3就偏大了，吸顶
//在添加了胶垫后，感觉应该可以大一点。
//20230708，通过风中测试，为了缓解风中气压不稳定导致的高度变化，增大这个imu阻尼值。
#define Kx (0.40*BARO_HEIGHT_RATIO)  //imu加速度，做阻尼。
#define Kpa (0.05*BARO_HEIGHT_RATIO) //气压加速度调节，
#define Kpx (0.07*BARO_HEIGHT_RATIO) //气压加速度阻尼
#define Ks (0.14*BARO_HEIGHT_RATIO)  //速度调节，增大后易引起高度震荡。但减小后则速度追踪缓慢。

    //限制范围削弱了加速度的阻尼，但可以避免部分噪音。
    //相当于限制 加速度在1.0m/s^2以内，超过的无能为力。
    //如果安装减震脚，则可以避免大噪音，这里可以处理更大的加速度。
    //20230503放大限制范围到0.15以内，机身有减震，代码有噪音防备。

    FLOAT_LIMIT(imuvacc, -3.0, 3.0);


//可能和气压计的iir有关，现在iir是7，估计还要往前找。
//#define trim (15) //延迟参数，15似乎比10好点，7似乎比10差点。18不行，起飞上冲严重
//10/30 高处减力过快， 15/30 合理小幅波动， 17/30冲顶不落。13/30高处减力过快坠落
//15/30确实在当下比较合适，目前是气压计100hz,iir=7，0-10，10-20，20-30三段采样计算速度和加速度
//似乎是trim对准三段的中间位置，span对准三段的跨度。
//如果三段的选择是0-5，5-10，10-15，则trim=7/8,span=15
//如果三段选择是0-6，6-12，12-18，则trim=9,span=18, 试试这个。不理想，可能是trim没对准，也可能是三段太短噪音大
//试试三段选0-7，7-14，14-21，则trim=12,span=21，这里似乎是trim没有对准，发力时而过头时而不够，15比12好，14/21较好，13/21不好
//因为trim=14最好，但还不完美，调节span, 14/28, 上力较慢，不达设定高度，14/25，相当稳加减力似乎偏慢，14/24似乎不如14/25
//7/12配合50hz挺好，试试5/15，似乎比7/12更好。

    
    float ref_thrust=get_refer_thrust_for_height_control();

    //在没有测距时，才允许气压控高执行平滑降落
#if !defined USE_TOF
    if(micro_task.smooth_landing)
    {
        if(micro_task.vspeed<=0) {
            micro_task.smooth_landing=false;
        }
        else{
            ref_thrust-=0.01;
            if(ref_thrust<0.6)
            {
                height_adjust_record har;
                har.tmark= dtime;
                har.thrust = 0; 
                har.hd= hd;
                har.spd = pspd2; //下为正。
                _height_adjust_queue.push(har);
                landed=true;
                return true;
            }
            else
            {
                height_adjust_record har;
                har.tmark= dtime;
                har.thrust = ref_thrust; 
                har.hd= hd;
                har.spd = pspd2; //下为正。
                _height_adjust_queue.push(har);
                landed=false;
                return true;
            }
        }
    }
#endif
    
    
    //新的能量控制法。自适应最优的爬升速度。
    float want_vspd= float(micro_task.vspeed)/10.0; //m/s 向上为负值

    float spd_dif = pspd2 - want_vspd; //速度调节项，向下为正。

    if(want_vspd < -2.0 ) //期望向上的速度大于2m/s 功耗控制才起作用
    {
        //超过2m/s的爬高速度，需要能量管理。
        //提取参考电流和垂直速度。

        float far_cur=0.0;
        float far_vspd=0.0;
        for(size_t i=4;i<8;i++)
        {
            far_cur+=_height_adjust_queue.last(i).current;
            far_vspd+= _height_adjust_queue.last(i).spd;
        }
        far_cur/=4.0;
        far_vspd/=4.0;

        //近端没有使用最后一个电流和速度数据。
        float near_cur=0.0;
        float near_vspd=0.0;
        for(size_t i=0;i<4;i++)
        {
            near_cur+=_height_adjust_queue.last(i).current;
            near_vspd+= _height_adjust_queue.last(i).spd;
        }
        near_cur/=4.0;
        near_vspd/=4.0;


        if(near_vspd < -2.0 && far_vspd < -2.0)
        {
            //两者同时有大于2m/s的向上速度，才可以比较效率。
            float effi_old= -far_vspd/far_cur; //效率，>0，越大越好
            float effi_new= -near_vspd/near_cur; //效率，>0，越大越好
            float r1= effi_new/effi_old;
            float r2= fabs(near_vspd/far_vspd); //最近速度大于早期速度则>1.0

            if(r1<1.0 && r2 > 1.0 && r1*r2 < 1.0) 
            {
                want_vspd= (far_vspd*0.2 + near_vspd*0.8);
            }

            //如果是最近效率高就不用理会和调节。
            float vlmt=PowerCostForClimbing(power_control_solution, far_cur, far_vspd);
            //返回值越接近1.0，速度提升应该越慢，然后限制最大速度。
            if(want_vspd < vlmt) {
                want_vspd= vlmt; //极限速度限制。
            }
            if(want_vspd < float(micro_task.vspeed)/10.0) want_vspd=float(micro_task.vspeed)/10.0;

            spd_dif= pspd2 - want_vspd; //正值导致向上增力。

        }
    }



    float acc_want; // = -hd/3.0 - pspd2; //期望加速度。hd>0是低于设定高度。pspd2>0是向下速度，负值是期望加速度向上。

    if(micro_task.vspeed!=0)
    {
        acc_want= -spd_dif; //动态无高差，根据是速度差调节。
    }
    else
    {
        acc_want = -hd/3.0 - pspd2; //期望加速度。hd>0是低于设定高度。pspd2>0是向下速度，负值是期望加速度向上。
    }

    //加速度不对称限制，以防减力过大过猛。20230804+
    FLOAT_LIMIT(acc_want, -2.0, 0.8); //20231217改  20240913 改向上加速度限制1.5, 20241203 放松到-2.0， 0.8
    float acc_dif= pacc - acc_want; //当前加速度-期望加速度 >0 加力。pacc, acc_want都是向下为正。


    //20240714，里面/3.0，外面*3.0，加强高差调节速度，之前在高差上易饱和，调节力度弱，侧飞高度会降低且不能及时纠正。
    float st10= Kp*3.0*SigmoidTrans(hd/3.0); //高差调节，hd有限制，避免一次的加减力过大。
    float st11= Ki*3.0*SigmoidTrans(accuhd/3.0); //高差积分项
    float st12= Kx*imuvacc*(1.2-flex); //imu加速度阻尼，不影响最终速度，除非有偏差。
    
    //气压微分阻尼，能力限制在6m/s的速度上。基本够用。
    float pdif0= (pr0.press - pr1.press)/2.0; //其实也是速度。
    float st13= Kd*SigmoidTrans(pdif0)*(1.0-flex); //由于近地气压大增，会引发加力，如果调节的很大，在纯气压控制下可能无法降落。

    //加速度调节项
    float st14 = Kpa*SigmoidTrans(acc_dif);
    //气压加速度阻尼项 20240913+ 这里不会像imuvacc一样有偏，这个是无偏的。
    float st15 = Kpx*pacc*(1.2-flex); //气压加速度阻尼


    //速度调节项
    float st16= Ks*SigmoidTrans(spd_dif); //如果大了，说明下落速度超过了设定，需要增力
    //速度阻尼项
    float st17= Kd*SigmoidTrans(pspd2)*(1.0-flex); 

    float change= st10+st11+st12+st13+st14+st15+st16+st17; 
    //20231212+ 防止掉高太快。无论是在设定高度的上方或下方，在设定为稳定高度时，
    //有下行速度超过1m/s的减力都被限制，以防止速度降落太快。辅助调节项。
    //速度匹配方面上面和下面都做了修改，有可能可以不用这个调节了。
    if(micro_task.vspeed==0)
    {
        //在无速度设定时，限制上下行速度，避免震荡加大。
        if(pspd2>0.5 && change<0.0)
            change+= Ks*SigmoidTrans(pspd2 - 0.5);
        else if(pspd2<-0.5 && change>0.0) //向上速度超过0.5m/s, 并且在增力，减小力度。
            change+= Ks*SigmoidTrans(pspd2 + 0.5); //是个负值。相当于减力。
    }

    //二次调节，检查过去0.4秒的高度差距，如果始终不能校准设定气压，那么附加上一个调节。
    //缩小区间无法改变震荡。似乎还更差，所以扩大区间到0.6秒，差距保持不变
    //似乎区间大小无法矫正震荡。5/20会冲顶，0/20也冲顶，5/15对付，但坠落反应不及时。
    //10/25对坠落反应较快，容易高出设置。10/20类似，波动性稍大。8/20似乎好于前面所有参数，8/16抖动变大了，不好
    //8/22不必 8/20好。8/18几乎和8/20表现一样好。


//如果以20个数据平均，10个数据为间隔，三个采样点的中心点分别是10个，20个，30个数据之前。
//三个高差的中心点是20个数据之前。算出来的加速度的中点也对应20个数据之前。要找对应的力，应该也是20个数据之前的点。

    //10个平均5个错位，对应加速度中心点在10个数据前。
    //平均范围和错位范围大点虽然可以计算的更精确，但是延迟大了。
    //可以考虑缩小平均长度和错位长度，会有噪音，但延迟较小。
    //实际不需要重叠段。
    float sumhd_near=0; 
    for(size_t i=0;i<4;i++) {
        sumhd_near+= _height_adjust_queue.last(i).hd;
    }
    sumhd_near/=4;

    float sumhd_mid=0; //更早一点的高度差，
    for(size_t i=4;i<8;i++) {
        sumhd_mid+= _height_adjust_queue.last(i).hd;
    }
    sumhd_mid/=4;

    float sumhd_far=0; //更早一点的高度差，
    for(size_t i=8;i<12;i++) {
        sumhd_far+= _height_adjust_queue.last(i).hd;
    }
    sumhd_far/=4;

    //统一的处理方式，可能要再考虑一个积分。即历史高差序列。
    //新法考虑不同的调节系数在不同的距离上。
    float sspd1 = (sumhd_near - sumhd_mid)*12.5; //速度向下为正。
    float sspd2 = (sumhd_mid - sumhd_far)*12.5; //上一个速度
    float sspd3 = (sumhd_near - sumhd_far)*6.25; //粗速度。时间跨度大，应该更精确。

    float sacc = (sspd1 - sspd2)*12.5; //加速度, 下为正，因采样时间缩短，sacc含有较大噪音，可能需要滤波


    float wt_spd = -sumhd_near; //期望速度，向下为正，如果当前高于设定，期望速度为正，将它减小到1/5
    if(wt_spd>0) wt_spd/=2.0; //抑制向下的速度不要太快。20230727

    float wt_acc = wt_spd - sspd1; //向上为负
    
    //20230804 修改向下的加速度限制，最大0.2，最小1.0，这和一次调节一样。
    FLOAT_LIMIT(wt_acc, -1.0, 0.5);  //同样，这里不对称抑制向下的加速度，向下加速应该较小。20230727

    //近处精细，远端钝化，开根号
    //增大参数导致波动稍大
    float t_hd, t_spd, t_acc;

    t_hd=SigmoidTrans(sumhd_near);

    //FLOAT_LIMIT(t_hd, -1.0, 1.0); //这里放开到2.0，同一次调节范围 20230727

    t_spd= SigmoidTrans(sspd1 - wt_spd);
    //FLOAT_LIMIT(t_spd, -1.0, 1.0); //这里放开到3.0，原来1.0， 20230726

    t_acc = SigmoidTrans(sacc-wt_acc);


    //在偏离设定高度时，逐步去掉由加速度带来的影响，防止由于机身抖动带来的持续加速上升意外。
    //越是偏高越是去除的多。
    //20230727 在高速上升突然停止时，不可避免要冲高，然后再回落，回落需要imu的阻尼来控制速度。
    //适当放开这里的限制，允许在冲高15米以后才开始减弱imu的作用。
    //20231011 修改限制范围更小
    //20231215 修改，适度放宽限制范围，上打杆是容易过头的，稍一过头就砍掉imu的作用容易造成掉高过快
    //20240223 继续放松。同步稳定版也放松
    if(sumhd_near < -20.0 ) {
        change -= st12;
    }
    else if(sumhd_near < -18.0 ) {
        change -= st12*0.9;
    }
    else if(sumhd_near < -16.0 ) {
        change -= st12*0.8;
    }
    else if(sumhd_near < -14.0 ) {
        change -= st12*0.7;
    }
    else if(sumhd_near < -12.0 ) {
        change -= st12*0.6;
    }
    else if(sumhd_near < -10.0 ) {
        change -= st12*0.5;
    }
    else if(sumhd_near < -8.0 ) {
        change -= st12*0.4;
    }
    else if(sumhd_near < -6.0 ) {
        change -= st12*0.3;
    }
    else if(sumhd_near < -4.0) {
        change -= st12*0.2;
    }
    else if(sumhd_near < -2.0 ) {
        change -= st12*0.1;
    }

    //未校准的imu,如果垂直方向加速度向上偏离，则推力会减弱，可能导致高度低于设定。此时适当降低imu阻尼。
    //前面那段只考虑了向上失控的情况。
    if(sumhd_near > 12.0) {
        change -= st12*0.4;
    }else if(sumhd_near > 10.0) {
        change -= st12*0.3;
    }else if(sumhd_near > 7.0) {
        change -= st12*0.2;
    }else if(sumhd_near > 4.0) {
        change -= st12*0.1;
    }

//单纯的微分调节，阻尼性质。用当前的数据

//高差调节
    float adj5 = t_hd*0.05*BARO_HEIGHT_RATIO*(1.2-flex); //增大系数反而增加波动。
    change += adj5;

//速度调节
    float adj6 = t_spd*0.05*BARO_HEIGHT_RATIO*(1.2-flex); //0.005
    change += adj6;

//加速度调节
    float adj7 = t_acc*0.05*BARO_HEIGHT_RATIO*(1.2-flex);  //0.005
    change += adj7;

//两个纯阻尼用当前的气压速度和加速度。
    float t_pacc, t_pspd;

    t_pacc = SigmoidTrans(sacc);

    //这里把设定的垂直速度考虑进去, 可能用最后的速度比较好，大跨度速度有延迟。
    t_pspd = SigmoidTrans(sspd3);

    //纯加速度阻尼。速度和加速度都是向下为正。
    //这个值大了似乎不能到达设定高度？
    float adj8= t_pacc*0.1*BARO_HEIGHT_RATIO*(1.2 - flex);  //0.06，0.1
    change += adj8;

    //存粹速度阻尼。这个值大了似乎不能到达设定高度？
    float adj9= t_pspd*0.1*BARO_HEIGHT_RATIO*(1.2 - flex);  //0.08，0.1
    change += adj9;



    change += takeoff_const_thrust; //起飞的持续增力。


    //单次调节限制。太大了震荡大。
    //气压控高的力度变化不宜太大。易引起气压的负反馈现象。发力气压增大，飞机以为是跌落，则继续追加发力，于是发力过大。
    //反之，减力下降时，气压降低，飞机以为是升高，继续减力，结果减力过大过快跌落。

    float onetime= ref_thrust*0.07;
    FLOAT_LIMIT(change, -0.12, 0.12); //这里比气压那边放的更大，近地需要迅速控制速度。
    FLOAT_LIMIT(change, -onetime, onetime);
    float newthrust = ref_thrust + change; 

#if defined USE_TOF
    //系统有测距时，气压控高不设置平滑降落
    FLOAT_LIMIT(newthrust, botlmt, toplmt);
#else
    //20231120+ 允许在只使用气压控高的情况下停机。
    if(micro_task.vspeed >0 && ref_thrust < BOTDUTY) 
    {
        // FLOAT_LIMIT(newthrust, botlmt, toplmt);
        // micro_task.help_landing=false;
        micro_task.smooth_landing=dtime;
    }
    else
    {
        //在设定向下速度>5时，且当前向下的速度较小时，打开辅助降落，这才允许降落。
        //不限制最低值，允许停机。
        //micro_task.help_landing=true;
        micro_task.smooth_landing=0;
    }
#endif

    if(zaxis<0 && !micro_task.smooth_landing)
    {
        if(!micro_task.smooth_landing)
        {
            FLOAT_LIMIT(newthrust, botlmt, 1.0); //颠倒后，限制最大推力1.0
            micro_task.smooth_landing=dtime;
            micro_task.smooth_landing_triger_reason=3; //因机身侧翻激活自动降落，以保护电机电调。
            logmessage("set smooth landing in baro for upside down\n");
            // //最好做个详细记录。

            // char dbg[64];
            // sprintf(dbg, "mark ud in hc, %u", dtime);
            // Send_remote_message(dbg);
        }
        else if(dtime - micro_task.smooth_landing > 1000)
        {
            logmessage("upside down, 1s landed\n");
            landed=true;
            char dbg[64];
            sprintf(dbg, "mark landed in hc, %u", dtime);
            Send_remote_message(dbg);
            return true;
        }
    }
    

    height_adjust_record har;
    har.tmark= dtime;
    har.thrust = newthrust;
    har.hd= hd;
    har.spd = pspd2; //下为正。
    har.current=system_power_current; //20240625+
    _height_adjust_queue.push(har);
    landed=false;
    return true;


#undef Kp
#undef Ki
#undef Kd
#undef Kx
#undef Kpa
#undef Kpx
#undef Ks

}

//返回true, 函数给出新的推力调节，并给出是否降落的标记信号。
//返回false, 没有推力调节数据，降落标记不可信。
bool CMicroTaskExec::height_adjust_by_baro_with_landing_detect_stable(float botlmt, float toplmt, bool& landed)
{

    press_info pr0, pr1, pr2, pr3;//, pr4, pr5; //采样2阶段气压数据，可求出气压速度。
    press_info praccu; //用于积分
    uint32_t dtime; //数据时间
    uint32_t tgap; //20230619+ 一个气压数据时间间隔。毫秒 >0
    critical_section_enter_blocking(&press_queue_section);
    dtime=press_queue.last().tmark;
    //此函数调用频率同imu数据频率，但气压数据频率很低，多数个调用只需要一次处理，处理过的气压数据就无需继续处理。
    //在此过滤处理过的气压数据节约后面的计算时间。20250227+
    if(_height_adjust_queue.last().tmark==dtime) {
        critical_section_exit(&press_queue_section);
        return false;
    }
    tgap= dtime - press_queue.last(5).tmark; //修改为5个数值间隔。
    pr0= press_queue.history_mean(0, 5);  //当前气压值，5均值。
    pr1= press_queue.history_mean(5, 5); //0.1s
    pr2= press_queue.history_mean(10, 5); //0.2s
    pr3= press_queue.history_mean(25, 5); //0.5s
    praccu= press_queue.history_mean(0, 40); //40个数据积分 
    critical_section_exit(&press_queue_section);



    // if(_height_adjust_queue.last().tmark==dtime) {
    //     //如有必要，这期间可以穿插处理。可以观察imu, tof.
    //     landed=false;
    //     return false;
    // }

    //如果是起飞，先搞到最低发力。
    if(micro_task.help_takeoff && _height_adjust_queue.last().thrust < botlmt)
    {
        height_adjust_record har;
        har.tmark=dtime;

        har.thrust=_height_adjust_queue.last().thrust + 0.01;

        har.hd= (pr0.press - micro_task.press)/11.719;
        har.current= system_power_current;
        har.spd=0;

        _height_adjust_queue.push(har);

        //起飞阶段不修改气压，是设定气压。
        micro_task.press= micro_task.help_takeoff_target_press;
        micro_task.tof_dominate=false;
        landed=false;
        return true;
    }

#if defined USE_TOF
    //控高模式切换检查。
    if(micro_task.tof_dominate)
    {
        //这是由tof切换到气压控高来。先前设定的气压高度作废，重新设置。
        micro_task.press=pr0.press;
        critical_section_enter_blocking(&gps_queue_section);
        micro_task.gps_ref_height=gps_queue.last(0).height;
        critical_section_exit(&gps_queue_section);
        //micro_task.gps_ref_height= gps_queue.last(0).height; //记录气压时，同时记录GPS高度。
        micro_task.gps_height_press_bias=0;

        if(micro_task.help_takeoff) 
        {
            micro_task.press= micro_task.help_takeoff_target_press; //起飞阶段盯住设定气压。
        }

        micro_task.tof_dominate=false;

        //刚从tof控高切换过来，时间间隔基于历史记录的时间。影响非常的小，可以忽略，如果是忽略，则以气压数据间隔调节高度。
        tgap= dtime - _height_adjust_queue.last().tmark; 

        //TOFSense-F2 Mini 可能确实有稳定问题，在高空常有这个信号出现，这不正常。
        char log[128];
        sprintf(log, "t2b, p=%6.0f, gps_h=%5.1f\n", micro_task.press, micro_task.gps_ref_height);
        logmessage(log);
    }
#endif



    float zaxis, vib_z;
    float imuvacc= get_refer_imu_vacc_for_height_control(zaxis, vib_z); //20240201+

    //三段式，检查是否取消平滑降落，执行平滑降落，是否进入平滑降落。
    //修改micro_task.vspeed < -5.0，之前是0为标准，可能受摇杆漂移影响。-5.0不算大，是0.5m/s向上的意思。
    if(micro_task.smooth_landing && micro_task.vspeed < -5.0) 
    {
        micro_task.smooth_landing=0; //关闭降落过程因请求速度向上。
        micro_task.smooth_landing_triger_reason=0;
        logmessage("cancel smoothlanding for vspeed < -5.0\n");
    }
    else if(micro_task.smooth_landing && micro_task.smooth_landing_triger_reason==3 && zaxis >0)
    {
        //因机身侧翻被纠正，之前因侧翻激活的降落被取消。
        micro_task.smooth_landing=0;
        micro_task.smooth_landing_triger_reason=0;
        logmessage("cancel upside down smoothlanding\n");
    }
    else if(micro_task.smooth_landing && micro_task.smooth_landing_triger_reason==1)
    {
        //tof测距打开了平滑降落，控制权却转移到了气压控高，这不合理。小距离降落必须全程在tof测距下完成。
        //一旦到了气压控高一定不对。
        //20240229发生的十几米高的平滑降落也许是一个tof误测距激发的，随后交给气压控高执行，而这里没有防备这种情况导致高空停机。
        micro_task.smooth_landing=0;
        micro_task.smooth_landing_triger_reason=0;
        logmessage("cancel smoothlanding set by tof\n");
    }

    //气压速度和气压加速度都非常不准，尤其是在电机发力变化很大的情况下，空气被扰动影响气压值，形成负反馈，所以sigmoid变化不可少。
    //修改了tgap，现在是5个数据间隔 20240223
    float pspd0 = (pr0.press - pr1.press)*(1000.0/tgap)/dynamic_air_press_height_ratio; //m/s 下为正
    float pspd1 = (pr1.press - pr2.press)*(1000.0/tgap)/dynamic_air_press_height_ratio; //m/s 下为正 (pr1.press - pr2.press)才对
    float pspd2 = (pr0.press - pr3.press)*(200.0/tgap)/dynamic_air_press_height_ratio; //m/s 下为正 25个数据时差，0.5秒, 用来设定速度的调节


    if(micro_task.smooth_landing)
    {

        //执行平滑降落并返回。
        float ref_thrust=_height_adjust_queue.last().thrust; //这里不使用过去平均推力了。

        //已经减力了多久？减力持续了1.5秒，直接停机。
        uint32_t tt= dtime - micro_task.smooth_landing;

        if(tt>=1500 || ref_thrust <= 0.5) {

            landed=true;
            _inair=false;
            char logmsg[128];
            sprintf(logmsg, "smooth landing done, reason=%d\n", micro_task.smooth_landing_triger_reason);
            logmessage(logmsg);

            height_adjust_record har;
            har.tmark= dtime;
            har.thrust = 0;
            har.hd = 0;
            har.spd = pspd2;
            har.current= system_power_current;
            _height_adjust_queue.push(har);
            return true;

        }else{

            //还有多少时间可以用来减力？目标是在余下的时间减小到0.5;
            uint32_t tl= 1500 - tt; //毫秒，剩余的时间。 >0
            float td = ref_thrust - 0.5; //要减少的力度  >0
            uint32_t gp= dtime - _height_adjust_queue.last().tmark; //本次调节与上次调节的时间间隔 >0 
            float adj = td * gp/tl; //本次调节。
            FLOAT_LIMIT(adj, 0.001, 0.2); //限制最小调节量。

            landed=false;
            height_adjust_record har;
            har.tmark= dtime;
            har.thrust = ref_thrust - adj;
            har.hd = 0;
            har.spd = pspd2;
            har.current= system_power_current;
            _height_adjust_queue.push(har);
            return true;
        }   
    }




    //如果设定了垂直速度，这里首先调节设定高度。
    //目前高度方向的运动主要靠速度匹配，不靠高差拉动。
    float flex=0;

    //由非零速度进入到0速度，当前实际没有做任何处理，会导致高度摆动。
    //如果是检测到归0的速度，可以进入刹车状态，在速度降低后再定高，则可减少摆动。
    if(micro_task.vspeed!=0 && !micro_task.help_takeoff)  //vspeed现在有向，>0速度向下。
    {
        //在有速度时，设定高度追随当前气压。附加一定的速度因子防止突然停止导致的震荡。
        //在有速度时，调节高差设置为0，只盯住速度去调节。
        micro_task.press = pr0.press + micro_task.vspeed*1.2; //现在新增刹车标记处理，不需要跟随设置稳定气压，但为防止意外而保留。
        flex = SigmoidTrans(fabs(micro_task.vspeed*0.1));
    }



    //gps调节静态高差，部分参与高度控制，但不完全信任GPS高度数据，可避免受干扰。
    if(micro_task.vspeed==0 && micro_task.height_break_tmark==0 && dtime - micro_task.gps_height_press_bias_last_update > 400 )
    {

        float gps_h;
        float gps_vacc;
        uint32_t gps_tmark;
        critical_section_enter_blocking(&gps_queue_section);
        gps_h= (gps_queue.last().height + gps_queue.last(1).height + gps_queue.last(2).height )/3.0;
        gps_vacc=gps_queue.last().vacc;
        gps_tmark=gps_queue.last().tmark;
        critical_section_exit(&gps_queue_section);

        if(gps_vacc < 1.2 && gps_tmark + 1000 > dtime) //限制高度精度在较小的范围内
        {
            //真高，如果真高很小，调节不适合太大。之前没考虑到这个问题，在高海拔起飞时会有这个问题。
            //应当限制在真高3%以内。
            float realh= gps_h - initial_gps_info.height; 
            if(realh<1.0) realh=1.0;
            float plmt= realh*dynamic_air_press_height_ratio*0.03; //气压调节范围

            float gps_press_bias= (gps_h - micro_task.gps_ref_height)*dynamic_air_press_height_ratio;//这里差值放大2.0倍可以调节范围更大。
            

            //这种调节能力受到一定限制，比如GPS显示目前高于设定高度0.5米，给出气压调节量5.6
            //但当前气压值却大于设定气压，大的量超出了1.5米高度，这两者就矛盾了，然后偏移量调节过后
            //GPS的高度依旧不能匹配到。即受调节量大小的限制，不能进一步调节高度。
            //如果是调节量本身不受限制，发现当前GPS高度高于设定或低于设定就累加累减调节量，那就可以
            //完全调节气压偏差。但可能受GPS数据影响较大。

            //避免一次调节过大，降低波动性
            if(gps_press_bias - micro_task.gps_height_press_bias > 3.0)
            {
                micro_task.gps_height_press_bias+=3.0;
            }
            else if(gps_press_bias - micro_task.gps_height_press_bias < -3.0)
            {
                micro_task.gps_height_press_bias-=3.0;
            }
            else 
            {
                micro_task.gps_height_press_bias= gps_press_bias;
            }

            //总后限制调节总值。一个基于GPS数据，一个基于气压数据。
            FLOAT_LIMIT(micro_task.gps_height_press_bias, -plmt, plmt); 

            float pdif= fabs(pr0.press - initial_press_info.press); //距离起飞位置的大致高度差。
            float adjlmt= 10.0 + pdif*0.04; //gps可调节量是气压差的4%+10.0
            FLOAT_LIMIT(micro_task.gps_height_press_bias, -adjlmt, adjlmt); //用于调节的量在高差大时变大，高差小时变小。

        }
        micro_task.gps_height_press_bias_last_update=dtime;
    }

    //刷新高度和气压的关系系数。这个系数影响垂直速度的计算。
    //不同的海拔高度下，每米气压变化不同。
    if(dtime - micro_task.air_press_height_relation_last_update > 3000)
    {
        float gps_h;
        float gps_vacc;
        uint32_t gps_tmark;
        critical_section_enter_blocking(&gps_queue_section);
        gps_h= gps_queue.last().height;
        gps_vacc=gps_queue.last().vacc;
        gps_tmark=gps_queue.last().tmark;
        critical_section_exit(&gps_queue_section);

        //是最近的数据且精度可以。
        if(gps_vacc < 5.0 && gps_tmark + 1000 > dtime) //限制高度精度在较小的范围内
        {
            dynamic_air_press_height_ratio=air_press_height_relation(gps_h);
            micro_task.air_press_height_relation_last_update=dtime;
        }
    }

    //高差
    float hd; 
    //高差积分
    float accuhd;

    if(micro_task.vspeed!=0) {
        hd=0; //有速度时，高差为0，不使用高差去调节。
        accuhd=0;
        micro_task.height_break_tmark=0;
        micro_task.gps_height_press_bias=0;
    }else{
        hd = (pr0.press - micro_task.press - micro_task.gps_height_press_bias)/11.719; //气压指示的距离设定高度的高度差。>0 表示低于设定高度.
        accuhd = (praccu.press - micro_task.press - micro_task.gps_height_press_bias)/11.719;
        //之前将高差限制的极小，当Imu加速度有偏时，高度就不能匹配，因Imu阻尼设置的较大。
        FLOAT_LIMIT(accuhd, -5.0, 5.0);
        FLOAT_LIMIT(hd, -12.0, 12.0); //高差不能限制的太小，否则长期下来容易掉高，太小的调节可能被其他调节抵消，导致长期高度失配。比如imu加速度的微小偏差。之前限制在0.5
    }

    //气压指示的加速度。这个可以去和imu加速度比较，如果是近似的就相信，如果差别很大，就否定。
    float pacc =  (pspd0 - pspd1)*(1000.0/tgap); //m/s^2 下为正


    if(micro_task.height_break_tmark)
    {
        //在刹车过程中，检查是否刹车过程结束，如果结束就设定目标气压。
        //也许此过程会延续到tof测距控高，所以那边也需要处理。
        //刹车阶段取消高差调节，即不使用稳定高度。有imu阻尼，气压阻尼，速度调节。最终速度会回到0.
        hd=0;
        accuhd=0;

        //这么处理主要是防止机身抖动剧烈，imuvacc有持续加速度无法降低，然后无法锚定高度。
        //无法锚定高度时，则无法限制机身抖动对高度的影响，可能持续飞高。
        float thres=0.2;
        if(dtime < micro_task.height_break_tmark + 3000) thres = 0.15;
        else if(dtime < micro_task.height_break_tmark + 6000) thres=0.20;
        else if(dtime < micro_task.height_break_tmark + 9000) thres=0.25;
        else thres=0.3;
        //刹车标记。vspeed一定为0，此时调节倾向于抑制速度加速度。也有临时的稳定气压用来高差调节，然后速度和加速度降低到一定程度时，设置稳定气压。
        if(fabs(pspd2)<thres && fabs(imuvacc) < thres && fabs(pacc) < thres)
        {
            micro_task.press=pr0.press; //设置当前稳定气压
            micro_task.height_break_tmark=0; //取消刹车标记。
            //记录气压的同时记录GPS高度。
            critical_section_enter_blocking(&gps_queue_section);
            micro_task.gps_ref_height=gps_queue.last(0).height;
            critical_section_exit(&gps_queue_section);

            micro_task.gps_height_press_bias=0;
        }
        
        if(micro_task.vspeed) micro_task.height_break_tmark=0; //20240702+ 可以没有但有更保险。
    }

    //限制气压速度减少气压波动干扰。
    //20230726 适度放开限制，原来5，现在7
    //20240508 放开限制，原来7，现在8，新增pspd2限制
    FLOAT_LIMIT(pspd0, -10.0, 11.5); //下为正，降落速度较快，放松点。
    FLOAT_LIMIT(pspd1, -10.0, 11.5);
    FLOAT_LIMIT(pspd2, -10.0, 11.5);

    FLOAT_LIMIT(pacc, -3.0, 3.0); //限制范围同imu acc

    float takeoff_const_thrust=0; //如果是起飞阶段，增加一个常量给推力。

    if(micro_task.help_takeoff)
    {

        //在离地后4个气压内都保持持续加力，避免受地面效应干扰。随后进入正常调节模式。
        //设定起飞高度应该至少大于4个气压高度。这里也可以强制利用tof来判断持续加力范围。
        if(pr0.press - micro_task.help_takeoff_target_press > 11.719 )
        {
            //距离目标值超过1米，
            takeoff_const_thrust = 0.02;
        }
        else if(pr0.press - micro_task.help_takeoff_target_press > 8.0 )
        {
            takeoff_const_thrust = 0.016;
        }
        else if(pr0.press - micro_task.help_takeoff_target_press > 6.0 )
        {
            takeoff_const_thrust = 0.012;
        }
        else if(pr0.press <= micro_task.help_takeoff_target_press + 0.1)
        {
            micro_task.help_takeoff=false; //已经辅助增力到位，关闭辅助推力，以后是遥控器控制高低。
            _inair=true; //标记已在空中。
        }

    }


//原0.15，因为amax电机的起飞油门太低，大概在1.0左右，所以调小这个比例，以降低高度的抖动性。
//对于新西达kv1100电机，参数小了上下波动偏大。kv值小的电机，这里要稍大才能使高度更稳定。
//#define RATIO (0.15) 
//#define RATIO (0.12) 
//不能有太大的加速度和速度，向上和向下的加速度和速度都不能大。所以速度和加速度的调节参数较大。
//单独的kp导致上下跳跃更大。
//20230725 kp原来0.12，改0.13，希望对高差更敏感点，侧飞目前掉高明显。
//20240909 由0.16改0.1，2806电机太强
#define Kp (0.06*BARO_HEIGHT_RATIO) //高差调节系数，高差来源于气压的直接测量
#define Ki (0.05*BARO_HEIGHT_RATIO) //高差积分系数
//取消这个微分调节，那么第二层微分调节要加强。
//下一步尝试增强这个阻尼，看效果。基础量 0.2，适度增强是可行的方案，0.8似乎偏强了，高度变化突兀，考虑0.4-0.5左右
//似乎调大了这个值导致无法降落？需要研究！20230401晚上出现过一次无法降落，20230402早起复现没有成功。
//这个左右阻尼作用大不大，有没有必要设置很大的参数，需要对比测试。
//大的kd需要大的kx来压制，否则容易过头
#define Kd (0.08*BARO_HEIGHT_RATIO) //气压微分调节，阻尼性质。在气压控制情况下可能是有自反馈现象，所以这个值不能大。
//降低这个参数显著增加高度的波动性。0.5在冲顶时容易吸附。0.3能工作，稍显偏小。0.4似乎合适。
//有时0.3就偏大了，吸顶
//在添加了胶垫后，感觉应该可以大一点。
//20230708，通过风中测试，为了缓解风中气压不稳定导致的高度变化，增大这个imu阻尼值。
#define Kx (0.40*BARO_HEIGHT_RATIO)  //imu加速度，做阻尼。
#define Kpa (0.05*BARO_HEIGHT_RATIO) //气压加速度调节，
#define Kpx (0.07*BARO_HEIGHT_RATIO) //气压加速度阻尼
#define Ks (0.14*BARO_HEIGHT_RATIO)  //速度调节，增大后易引起高度震荡。但减小后则速度追踪缓慢。

    //限制范围削弱了加速度的阻尼，但可以避免部分噪音。
    //相当于限制 加速度在1.0m/s^2以内，超过的无能为力。
    //如果安装减震脚，则可以避免大噪音，这里可以处理更大的加速度。
    //20230503放大限制范围到0.15以内，机身有减震，代码有噪音防备。

    FLOAT_LIMIT(imuvacc, -3.0, 3.0);


//可能和气压计的iir有关，现在iir是7，估计还要往前找。
//#define trim (15) //延迟参数，15似乎比10好点，7似乎比10差点。18不行，起飞上冲严重
//10/30 高处减力过快， 15/30 合理小幅波动， 17/30冲顶不落。13/30高处减力过快坠落
//15/30确实在当下比较合适，目前是气压计100hz,iir=7，0-10，10-20，20-30三段采样计算速度和加速度
//似乎是trim对准三段的中间位置，span对准三段的跨度。
//如果三段的选择是0-5，5-10，10-15，则trim=7/8,span=15
//如果三段选择是0-6，6-12，12-18，则trim=9,span=18, 试试这个。不理想，可能是trim没对准，也可能是三段太短噪音大
//试试三段选0-7，7-14，14-21，则trim=12,span=21，这里似乎是trim没有对准，发力时而过头时而不够，15比12好，14/21较好，13/21不好
//因为trim=14最好，但还不完美，调节span, 14/28, 上力较慢，不达设定高度，14/25，相当稳加减力似乎偏慢，14/24似乎不如14/25
//7/12配合50hz挺好，试试5/15，似乎比7/12更好。

    
    float ref_thrust=get_refer_thrust_for_height_control();

    //在没有测距时，才允许气压控高执行平滑降落
#if !defined USE_TOF
    if(micro_task.smooth_landing)
    {
        if(micro_task.vspeed<=0) {
            micro_task.smooth_landing=false;
        }
        else{
            ref_thrust-=0.01;
            if(ref_thrust<0.6)
            {
                height_adjust_record har;
                har.tmark= dtime;
                har.thrust = 0; 
                har.hd= hd;
                har.spd = pspd2; //下为正。
                _height_adjust_queue.push(har);
                landed=true;
                return true;
            }
            else
            {
                height_adjust_record har;
                har.tmark= dtime;
                har.thrust = ref_thrust; 
                har.hd= hd;
                har.spd = pspd2; //下为正。
                _height_adjust_queue.push(har);
                landed=false;
                return true;
            }
        }
    }
#endif
    
    
    //新的能量控制法。自适应最优的爬升速度。
    float want_vspd= float(micro_task.vspeed)/10.0; //m/s 向上为负值

    float spd_dif = pspd2 - want_vspd; //速度调节项，向下为正。

    if(want_vspd < -2.0 ) //期望向上的速度大于2m/s 功耗控制才起作用
    {
        //超过2m/s的爬高速度，需要能量管理。
        //提取参考电流和垂直速度。

        float far_cur=0.0;
        float far_vspd=0.0;
        for(size_t i=4;i<8;i++)
        {
            far_cur+=_height_adjust_queue.last(i).current;
            far_vspd+= _height_adjust_queue.last(i).spd;
        }
        far_cur/=4.0;
        far_vspd/=4.0;

        //近端没有使用最后一个电流和速度数据。
        float near_cur=0.0;
        float near_vspd=0.0;
        for(size_t i=0;i<4;i++)
        {
            near_cur+=_height_adjust_queue.last(i).current;
            near_vspd+= _height_adjust_queue.last(i).spd;
        }
        near_cur/=4.0;
        near_vspd/=4.0;


        if(near_vspd < -2.0 && far_vspd < -2.0)
        {
            //两者同时有大于2m/s的向上速度，才可以比较效率。
            float effi_old= -far_vspd/far_cur; //效率，>0，越大越好
            float effi_new= -near_vspd/near_cur; //效率，>0，越大越好
            float r1= effi_new/effi_old;
            float r2= fabs(near_vspd/far_vspd); //最近速度大于早期速度则>1.0

            if(r1<1.0 && r2 > 1.0 && r1*r2 < 1.0) 
            {
                want_vspd= (far_vspd*0.2 + near_vspd*0.8);
            }

            //如果是最近效率高就不用理会和调节。
            float vlmt=PowerCostForClimbing(power_control_solution, far_cur, far_vspd);
            //返回值越接近1.0，速度提升应该越慢，然后限制最大速度。
            if(want_vspd < vlmt) {
                want_vspd= vlmt; //极限速度限制。
            }
            if(want_vspd < float(micro_task.vspeed)/10.0) want_vspd=float(micro_task.vspeed)/10.0;

            spd_dif= pspd2 - want_vspd; //正值导致向上增力。

        }
    }



    float acc_want; // = -hd/3.0 - pspd2; //期望加速度。hd>0是低于设定高度。pspd2>0是向下速度，负值是期望加速度向上。

    if(micro_task.vspeed!=0)
    {
        acc_want= -spd_dif; //动态无高差，根据是速度差调节。
    }
    else
    {
        acc_want = -hd/3.0 - pspd2; //期望加速度。hd>0是低于设定高度。pspd2>0是向下速度，负值是期望加速度向上。
    }

    //加速度不对称限制，以防减力过大过猛。20230804+
    FLOAT_LIMIT(acc_want, -2.0, 0.8); //20231217改  20240913 改向上加速度限制1.5, 20241203 放松到-2.0， 0.8
    float acc_dif= pacc - acc_want; //当前加速度-期望加速度 >0 加力。pacc, acc_want都是向下为正。


    //20240714，里面/3.0，外面*3.0，加强高差调节速度，之前在高差上易饱和，调节力度弱，侧飞高度会降低且不能及时纠正。
    float st10= Kp*3.0*SigmoidTrans(hd/3.0); //高差调节，hd有限制，避免一次的加减力过大。
    float st11= Ki*3.0*SigmoidTrans(accuhd/3.0); //高差积分项
    float st12= Kx*imuvacc*(1.2-flex); //imu加速度阻尼，不影响最终速度，除非有偏差。
    
    //气压微分阻尼，能力限制在6m/s的速度上。基本够用。
    float pdif0= (pr0.press - pr1.press)/2.0; //其实也是速度。
    float st13= Kd*SigmoidTrans(pdif0)*(1.0-flex); //由于近地气压大增，会引发加力，如果调节的很大，在纯气压控制下可能无法降落。

    //加速度调节项
    float st14 = Kpa*SigmoidTrans(acc_dif);
    //气压加速度阻尼项 20240913+ 这里不会像imuvacc一样有偏，这个是无偏的。
    float st15 = Kpx*pacc*(1.2-flex); //气压加速度阻尼


    //速度调节项
    float st16= Ks*SigmoidTrans(spd_dif); //如果大了，说明下落速度超过了设定，需要增力
    //速度阻尼项
    float st17= Kd*SigmoidTrans(pspd2)*(1.0-flex); 

    float change= st10+st11+st12+st13+st14+st15+st16+st17; 
    //20231212+ 防止掉高太快。无论是在设定高度的上方或下方，在设定为稳定高度时，
    //有下行速度超过1m/s的减力都被限制，以防止速度降落太快。辅助调节项。
    //速度匹配方面上面和下面都做了修改，有可能可以不用这个调节了。
    if(micro_task.vspeed==0)
    {
        //在无速度设定时，限制上下行速度，避免震荡加大。
        if(pspd2>0.5 && change<0.0)
            change+= Ks*SigmoidTrans(pspd2 - 0.5);
        else if(pspd2<-0.5 && change>0.0) //向上速度超过0.5m/s, 并且在增力，减小力度。
            change+= Ks*SigmoidTrans(pspd2 + 0.5); //是个负值。相当于减力。
    }

    //二次调节，检查过去0.4秒的高度差距，如果始终不能校准设定气压，那么附加上一个调节。
    //缩小区间无法改变震荡。似乎还更差，所以扩大区间到0.6秒，差距保持不变
    //似乎区间大小无法矫正震荡。5/20会冲顶，0/20也冲顶，5/15对付，但坠落反应不及时。
    //10/25对坠落反应较快，容易高出设置。10/20类似，波动性稍大。8/20似乎好于前面所有参数，8/16抖动变大了，不好
    //8/22不必 8/20好。8/18几乎和8/20表现一样好。


//如果以20个数据平均，10个数据为间隔，三个采样点的中心点分别是10个，20个，30个数据之前。
//三个高差的中心点是20个数据之前。算出来的加速度的中点也对应20个数据之前。要找对应的力，应该也是20个数据之前的点。

    //10个平均5个错位，对应加速度中心点在10个数据前。
    //平均范围和错位范围大点虽然可以计算的更精确，但是延迟大了。
    //可以考虑缩小平均长度和错位长度，会有噪音，但延迟较小。
    //实际不需要重叠段。
    float sumhd_near=0; 
    for(size_t i=0;i<4;i++) {
        sumhd_near+= _height_adjust_queue.last(i).hd;
    }
    sumhd_near/=4;

    float sumhd_mid=0; //更早一点的高度差，
    for(size_t i=4;i<8;i++) {
        sumhd_mid+= _height_adjust_queue.last(i).hd;
    }
    sumhd_mid/=4;

    float sumhd_far=0; //更早一点的高度差，
    for(size_t i=8;i<12;i++) {
        sumhd_far+= _height_adjust_queue.last(i).hd;
    }
    sumhd_far/=4;

    //统一的处理方式，可能要再考虑一个积分。即历史高差序列。
    //新法考虑不同的调节系数在不同的距离上。
    float sspd1 = (sumhd_near - sumhd_mid)*12.5; //速度向下为正。
    float sspd2 = (sumhd_mid - sumhd_far)*12.5; //上一个速度
    float sspd3 = (sumhd_near - sumhd_far)*6.25; //粗速度。时间跨度大，应该更精确。

    float sacc = (sspd1 - sspd2)*12.5; //加速度, 下为正，因采样时间缩短，sacc含有较大噪音，可能需要滤波


    float wt_spd = -sumhd_near; //期望速度，向下为正，如果当前高于设定，期望速度为正，将它减小到1/5
    if(wt_spd>0) wt_spd/=2.0; //抑制向下的速度不要太快。20230727

    float wt_acc = wt_spd - sspd1; //向上为负
    
    //20230804 修改向下的加速度限制，最大0.2，最小1.0，这和一次调节一样。
    FLOAT_LIMIT(wt_acc, -1.0, 0.5);  //同样，这里不对称抑制向下的加速度，向下加速应该较小。20230727

    //近处精细，远端钝化，开根号
    //增大参数导致波动稍大
    float t_hd, t_spd, t_acc;

    t_hd=SigmoidTrans(sumhd_near);

    //FLOAT_LIMIT(t_hd, -1.0, 1.0); //这里放开到2.0，同一次调节范围 20230727

    t_spd= SigmoidTrans(sspd1 - wt_spd);
    //FLOAT_LIMIT(t_spd, -1.0, 1.0); //这里放开到3.0，原来1.0， 20230726

    t_acc = SigmoidTrans(sacc-wt_acc);


    //在偏离设定高度时，逐步去掉由加速度带来的影响，防止由于机身抖动带来的持续加速上升意外。
    //越是偏高越是去除的多。
    //20230727 在高速上升突然停止时，不可避免要冲高，然后再回落，回落需要imu的阻尼来控制速度。
    //适当放开这里的限制，允许在冲高15米以后才开始减弱imu的作用。
    //20231011 修改限制范围更小
    //20231215 修改，适度放宽限制范围，上打杆是容易过头的，稍一过头就砍掉imu的作用容易造成掉高过快
    //20240223 继续放松。同步稳定版也放松
    if(sumhd_near < -20.0 ) {
        change -= st12;
    }
    else if(sumhd_near < -18.0 ) {
        change -= st12*0.9;
    }
    else if(sumhd_near < -16.0 ) {
        change -= st12*0.8;
    }
    else if(sumhd_near < -14.0 ) {
        change -= st12*0.7;
    }
    else if(sumhd_near < -12.0 ) {
        change -= st12*0.6;
    }
    else if(sumhd_near < -10.0 ) {
        change -= st12*0.5;
    }
    else if(sumhd_near < -8.0 ) {
        change -= st12*0.4;
    }
    else if(sumhd_near < -6.0 ) {
        change -= st12*0.3;
    }
    else if(sumhd_near < -4.0) {
        change -= st12*0.2;
    }
    else if(sumhd_near < -2.0 ) {
        change -= st12*0.1;
    }

    //未校准的imu,如果垂直方向加速度向上偏离，则推力会减弱，可能导致高度低于设定。此时适当降低imu阻尼。
    //前面那段只考虑了向上失控的情况。
    if(sumhd_near > 12.0) {
        change -= st12*0.4;
    }else if(sumhd_near > 10.0) {
        change -= st12*0.3;
    }else if(sumhd_near > 7.0) {
        change -= st12*0.2;
    }else if(sumhd_near > 4.0) {
        change -= st12*0.1;
    }

//单纯的微分调节，阻尼性质。用当前的数据

//高差调节
    float adj5 = t_hd*0.05*BARO_HEIGHT_RATIO*(1.2-flex); //增大系数反而增加波动。
    change += adj5;

//速度调节
    float adj6 = t_spd*0.05*BARO_HEIGHT_RATIO*(1.2-flex); //0.005
    change += adj6;

//加速度调节
    float adj7 = t_acc*0.05*BARO_HEIGHT_RATIO*(1.2-flex);  //0.005
    change += adj7;

//两个纯阻尼用当前的气压速度和加速度。
    float t_pacc, t_pspd;

    t_pacc = SigmoidTrans(sacc);

    //这里把设定的垂直速度考虑进去, 可能用最后的速度比较好，大跨度速度有延迟。
    t_pspd = SigmoidTrans(sspd3);

    //纯加速度阻尼。速度和加速度都是向下为正。
    //这个值大了似乎不能到达设定高度？
    float adj8= t_pacc*0.1*BARO_HEIGHT_RATIO*(1.2 - flex);  //0.06，0.1
    change += adj8;

    //存粹速度阻尼。这个值大了似乎不能到达设定高度？
    float adj9= t_pspd*0.1*BARO_HEIGHT_RATIO*(1.2 - flex);  //0.08，0.1
    change += adj9;



    change += takeoff_const_thrust; //起飞的持续增力。


    //单次调节限制。太大了震荡大。
    //气压控高的力度变化不宜太大。易引起气压的负反馈现象。发力气压增大，飞机以为是跌落，则继续追加发力，于是发力过大。
    //反之，减力下降时，气压降低，飞机以为是升高，继续减力，结果减力过大过快跌落。

    float onetime= ref_thrust*0.07;
    FLOAT_LIMIT(change, -0.12, 0.12); //这里比气压那边放的更大，近地需要迅速控制速度。
    FLOAT_LIMIT(change, -onetime, onetime);
    float newthrust = ref_thrust + change; 

#if defined USE_TOF
    //系统有测距时，气压控高不设置平滑降落
    FLOAT_LIMIT(newthrust, botlmt, toplmt);
#else
    //20231120+ 允许在只使用气压控高的情况下停机。
    if(micro_task.vspeed >0 && ref_thrust < BOTDUTY) 
    {
        // FLOAT_LIMIT(newthrust, botlmt, toplmt);
        // micro_task.help_landing=false;
        micro_task.smooth_landing=dtime;
    }
    else
    {
        //在设定向下速度>5时，且当前向下的速度较小时，打开辅助降落，这才允许降落。
        //不限制最低值，允许停机。
        //micro_task.help_landing=true;
        micro_task.smooth_landing=0;
    }
#endif

    if(zaxis<0 && !micro_task.smooth_landing)
    {
        if(!micro_task.smooth_landing)
        {
            FLOAT_LIMIT(newthrust, botlmt, 1.0); //颠倒后，限制最大推力1.0
            micro_task.smooth_landing=dtime;
            micro_task.smooth_landing_triger_reason=3; //因机身侧翻激活自动降落，以保护电机电调。
            logmessage("set smooth landing in baro for upside down\n");
            char dbg[64];
            sprintf(dbg, "mark ud in hc, %u", dtime);
            Send_remote_message(dbg);
        }
        else if(dtime - micro_task.smooth_landing > 1000)
        {
            logmessage("upside down, 1s landed\n");
            landed=true;
            char dbg[64];
            sprintf(dbg, "mark landed in hc, %u", dtime);
            Send_remote_message(dbg);
            return true;
        }
    }
    

    height_adjust_record har;
    har.tmark= dtime;
    har.thrust = newthrust;
    har.hd= hd;
    har.spd = pspd2; //下为正。
    har.current=system_power_current; //20240625+
    _height_adjust_queue.push(har);
    landed=false;
    return true;


#undef Kp
#undef Ki
#undef Kd
#undef Kx
#undef Kpa
#undef Kpx
#undef Ks

}

//20240508 新增intof反馈用以避免降落。
bool CMicroTaskExec::height_adjust_with_landing_detect(float botlmt, float toplmt, bool& landed)
{
#if defined USE_TOF
    // if(_inair && _baro_height > 30) //30米以上就不使用tof测距。
    // {
    //     _intof=false;
    //     return hyper_height_adjust_by_baro_with_landing_detect(botlmt, toplmt, landed);
    // }

    bool tofadj=height_adjust_by_tof_with_landing_detect(botlmt, toplmt, landed);
    if(!tofadj) {
        _intof=false;
        return hyper_height_adjust_by_baro_with_landing_detect(botlmt, toplmt, landed);
    }else{
        _intof=true;
        return true;
    } 
 

#else
    return hyper_height_adjust_by_baro_with_landing_detect(botlmt, toplmt, landed);
#endif

}



//在距离地面很近时开启3秒持续减力，期间测距如果距离不达标，或速度设定不是向下，则终止持续减力。
//在首次检测到距离很近时，标记持续减力状态，在没有向下的速度设置时，或测距不够近时取消持续减力状态。
//20240715 稍做修改，为测距频率修改为50准备。
//20240909 经过修改后，目前在2806机器上表现较好，偶有高速坠落，但一般不触地。增加了测量加速度阻尼，增加了积分调节。
//20241130 测试高度控制时，在1000米降落到100米的过程中，在900米高度上有一个有效测距，结果出发tof控高有效，然后标记_intof
//然后在那个时间点上，高度不再继续下降，下降任务结束。这个测距有效按理不该发生，天气晴好，估计是测距模块错误导致的
//考虑到避免这种情况，可以取消只有一个测距数据或2个测距数据有效的测距控高，而是必须连续三个测距有效才进行测距控高。
//返回true, 函数给出新的推力调节，并给出是否降落的标记信号。
//返回false, 没有推力调节数据，降落标记不可信。
#if defined USE_TOF
bool CMicroTaskExec::height_adjust_by_tof_with_landing_detect(float botlmt, float toplmt, bool& landed)
{

    //现在有数据滤波，不需要很大的跨度去计算平均值。
    tof_oflow_info tof0, tof1, tof2, tof3;//3个数据
    uint32_t dtime; //数据时间
    uint32_t tgap; //20230619 一个数据的时间间隔。>0
    float accu_tof_h=0; //高度积分项。
    uint16_t accu_num=0;
    critical_section_enter_blocking(&tof_oflow_queue_section);
    dtime=tof_oflow_queue.last().tmark;
    tgap = dtime - tof_oflow_queue.last(1).tmark;
    tof0= tof_oflow_queue.last(0);
    tof1= tof_oflow_queue.last(1);
    tof2= tof_oflow_queue.last(2);
    tof3= tof_oflow_queue.last(10); //10个数据跨度，用于求速度，如果距离太近速度波动大
    //积分最近20个数据。
    for(int i=0;i<20;i++) {
        if(tof_oflow_queue.last(i).tof_valid) {
            accu_tof_h+= tof_oflow_queue.last(i).fixed_tof_distance;
            accu_num++;
        }
    }
    if(accu_num>0) accu_tof_h/=accu_num;

    critical_section_exit(&tof_oflow_queue_section);

    //数据不可用时，返回false，并标记，这样下次数据可用时知道是切换;
    //要求三个连续的测距有效才执行tof控高，目前的TOFSense-F2 Mini似乎存在误测问题。
    if(!tof0.tof_valid || !tof1.tof_valid || !tof2.tof_valid) {
        //micro_task.tof_dominate=false; //不应标记这个，否则气压控高那边不知道状态切换
        return false;
    }



    uint32_t now = get_time_mark();
    //防止tof模块彻底死机。如果它死机了，没有数据回传了，数据队列会停留在最后时间。
    //然后如果数据还有效，则会持续根据最后数据动作。又因为后面会比较当前的数据是否执行
    //了控高动作，所以就是后期控高无动作，且系统认为还在测距控制中，这样控高就崩溃了。
    //20250223+
    if(now > dtime + 300) {
        return false; //tof测距已经超时，无效数据，交给气压控高。
    }

    if(_height_adjust_queue.last().tmark==dtime) {
        landed=false;
        return true;//20240206+
    }

    //新的防范措施：
    //三个连续测距有效，且数据是新数据，此时可以检查气压速度，如果气压速度是向上的，并且速度较快，即可主动放弃测距控高，以避免测距幻觉。
    //如果气压速度向下，且接近测距速度，可应用测距控高。
    press_info pr0, pr3;
    uint32_t pgap;
    critical_section_enter_blocking(&press_queue_section);
    pgap= press_queue.last().tmark - press_queue.last(25).tmark; //>0
    pr0= press_queue.history_mean(0, 5);
    pr3= press_queue.history_mean(25, 5);
    critical_section_exit(&press_queue_section);
    float pvspd= (pr0.press - pr3.press)*1000.0/pgap/11.72; //气压速度 , 单位m/s。<0 速度向上。
    if(_inair && pvspd < -2.0) return false; //向上速度大于2m/s不使用测距控高。
    //if(pvspd > 6.0*11.72) return false; //向下速度大于6m/s 这应该不是近地面能发生的速度。
    //接着比较测距速度和气压速度的差异，差异不大就算测距有效。


    //20241130 关闭只有一个或两个测距数据有效的情况。因今天在空中发生了不明测距有效
    //这种瞬间的有效测距导致_intof被设置，从而导致降落时误以为接近地面而停止下降
    //这可能是当前使用的测距模块偶尔有误报误测，也有可能是当时有飞鸟从测距下方经过所引起。当时高度900多米，飞鸟的概率极小。
    //但误报测距的概率也是极小，原因不明。

    // if(!tof1.tof_valid) return false; //20241130+
    // if(!tof2.tof_valid) return false; //20241130+

    // if(!tof1.tof_valid && !tof2.tof_valid) {
    //     //只有一个数据可用，后两个数据都不可用。那么参考记录序列里的降落速度，直接刹车。

    //     float ref_thrust=get_refer_thrust_for_height_control();

    //     if(_height_adjust_queue.last().spd > 10.0)
    //     {

    //         height_adjust_record har;
    //         har.tmark= dtime;
    //         har.thrust = ref_thrust*1.08;
    //         har.hd = 0; //设定的高差为0.
    //         if(har.thrust > toplmt) har.thrust=toplmt;
    //         har.spd = _height_adjust_queue.last().spd; //计算不出速度，沿用老的速度。
    //         _height_adjust_queue.push(har);

    //         landed=false;
    //         return true;
    //     }
    //     else if(_height_adjust_queue.last().spd > 5.0)
    //     {
    //         height_adjust_record har;
    //         har.tmark= dtime;
    //         har.thrust = ref_thrust*1.06;
    //         if(har.thrust > toplmt) har.thrust=toplmt;

    //         har.hd = 0; //设定的高差为0.

    //         har.spd = _height_adjust_queue.last().spd; //计算不出速度，沿用老的速度。
    //         _height_adjust_queue.push(har);

    //         landed=false;
    //         return true;
    //     }
    //     else if(_height_adjust_queue.last().spd > 3.0)
    //     {

    //         height_adjust_record har;
    //         har.tmark= dtime;
    //         har.thrust = ref_thrust*1.04;
    //         if(har.thrust > toplmt) har.thrust=toplmt;
            
    //         har.hd = 0; //设定的高差为0.

    //         har.spd = _height_adjust_queue.last().spd; //计算不出速度，沿用老的速度。
    //         _height_adjust_queue.push(har);
    //         landed=false;
    //         return true;
    //     }
    //     else if(_height_adjust_queue.last().spd > 2.0)
    //     {
    //         //速度过快
    //         height_adjust_record har;
    //         har.tmark= dtime;
    //         har.thrust = ref_thrust*1.02;
    //         if(har.thrust > toplmt) har.thrust=toplmt;
            
    //         har.hd = 0; //设定的高差为0.

    //         har.spd = _height_adjust_queue.last().spd; //计算不出速度，沿用老的速度。
    //         _height_adjust_queue.push(har);
    //         landed=false;
    //         return true;
    //     }
    //     else
    //     {
    //         //速度较低，不必调节，让气压调节。
    //         return false;
    //     }
    // }
    // else if(tof1.tof_valid && !tof2.tof_valid) {

    //     //有两个数据可用，只是最后一个数据不可用。可计算速度，不可计算加速度。
    //     float ref_thrust=get_refer_thrust_for_height_control();
        
    //     //距离差，米，
    //     float tof_dist_dif = tof1.fixed_tof_distance - tof0.fixed_tof_distance;
    //      //速度
    //     float tof_vspd = tof_dist_dif*1000.0f/float(tof0.tmark - tof1.tmark); //m/s  >0 速度向下。

    //     if(tof_vspd > 10.0) {

    //         height_adjust_record har;
    //         har.tmark= dtime;
    //         har.thrust = ref_thrust * 1.08;
    //         har.hd = 0; //设定的高差为0.

    //         har.spd = tof_vspd; //计算不出速度，沿用老的速度。
    //         if(har.thrust > toplmt) har.thrust=toplmt;
    //         _height_adjust_queue.push(har);
    //         landed=false;
    //         return true;

    //     }else if(tof_vspd > 5.0) {
    //         //速度过快
    //         height_adjust_record har;
    //         har.tmark= dtime;
    //         har.thrust = ref_thrust * 1.06;
    //         if(har.thrust > toplmt) har.thrust=toplmt;

    //         har.hd = 0; //设定的高差为0.

    //         har.spd = tof_vspd; //计算不出速度，沿用老的速度。
    //         _height_adjust_queue.push(har);
    //         landed=false;
    //         return true;

    //     }else if(tof_vspd > 3.0) {

    //         //速度过快，
    //         height_adjust_record har;
    //         har.tmark= dtime;
    //         har.thrust = ref_thrust * 1.04;
    //         if(har.thrust > toplmt) har.thrust=toplmt;
            
    //         har.hd = 0; //设定的高差为0.

    //         har.spd = tof_vspd; //计算不出速度，沿用老的速度。
    //         _height_adjust_queue.push(har);
    //         landed=false;
    //         return true;

    //     }else if(tof_vspd > 2.0) {
    //         //速度过快，直接+0.3力度

    //         height_adjust_record har;
    //         har.tmark= dtime;
    //         har.thrust = ref_thrust * 1.02;
    //         if(har.thrust > toplmt) har.thrust=toplmt;
            
    //         har.hd = 0; //设定的高差为0.

    //         har.spd = tof_vspd; //计算不出速度，沿用老的速度。
    //         _height_adjust_queue.push(har);
    //         landed=false;
    //         return true;
    //     }
    //     else{
    //         //速度较低，不必调节，让气压调节。
    //         return false;
    //     }
    // }
    // else if(!tof1.tof_valid && tof2.tof_valid) {
    //     //之前没考虑这种情况，tof0，tof2有效，tof1无效，而是跳过这个判断认为所有三个测距都有效，然后继续往后计算，会有小问题。
    //     return false; //不接管，继续气压控制。等待所有测距有效。
    // }


    //至此最近三个测距都有效。确定控制权。

    //如果tof数据中断了，后面会直接返回真，就没有力度调节了。
    //为了防范这种小概率失效，这里先判断数据流是否已经中断。
    
    //如果是起飞，先搞到最低发力。
    if(micro_task.help_takeoff && _height_adjust_queue.last().thrust < botlmt)
    {
        height_adjust_record har;
        har.tmark= dtime;
        har.thrust=_height_adjust_queue.last().thrust + 0.012;
        har.hd= micro_task.tof_height - tof0.fixed_tof_distance;
        har.current= system_power_current;
        har.spd=0;
        _height_adjust_queue.push(har);

        //起飞阶段不修改高度，那是设定高度。
        micro_task.tof_dominate=true;
        landed=false;
        return true;
    }

    

    //高度差。当tof由不可用变得可用时，高度设定变为tof_height，这需要提前设置这个值，使这个值=tof0的测距，
    //这样由气压设定平滑过度到tof设定，否则这个过渡就很不平滑。
    //平地起飞时，设置tof_dominate=true, 并且设置tof_height一个固定高度。

    //控高模式切换检查，到这里数据完整，tof可以抢夺控高权。
    if(!micro_task.tof_dominate)
    {

        //为避免tof测距的错误数据，这里比较tof的速度和气压速度，以检查测距数据的可靠性。
        float tof_v0= (tof0.fixed_tof_distance - tof1.fixed_tof_distance)*1000.0f/(tof0.tmark - tof1.tmark); //速度向上为正
        float tof_v1= (tof1.fixed_tof_distance - tof2.fixed_tof_distance)*1000.0f/(tof1.tmark - tof2.tmark); //速度向上为正
        //这两个速度不应差别太大，其绝对值也不应太大，否则放弃测距控高，改用气压控高。
        if(tof_v0>3.0 || tof_v1>3.0) //测距期间不可能有这么大的向上速度。即便有这么大的向上速度，也交给气压控高了。
        {
            return false; 
        }

        if(tof_v0 < -7.0 || tof_v1 < -7.0) //测距期间不可能有这么大的向下速度。即便有也是失控了。
        {
            return false;
        }

        //接着可以比较和气压速度的差异，如果太大也不使用测距控高。
        //这里求速度差用了加法，因为两者正方向相反。
        //之前这里忘记换算pvspd到标准单位，比较起来误差很大，现在修正。
        if( fabs((tof_v0+tof_v1)/2.0 + pvspd) > 2.0 ) //测距速度和气压速度相差很大，可能是测距数据错误。不使用测距控高。
        {
            // char log[128];
            // sprintf(log, "tofspd=%3.2f, pvspd=%3.2f, skip tof\n", (tof_v0+tof_v1)/2.0, pvspd);
            // logmessage(log);
            return false;
        }

        //经过检查这里接管控高。

        //由气压到tof的状态切换。首次进入tof控高。
        micro_task.tof_height= tof0.fixed_tof_distance;
        //如果标记了起飞阶段，目标高度盯住的是设定的目标高度。
        if(micro_task.help_takeoff) micro_task.tof_height = micro_task.help_takeoff_target_height;
        micro_task.tof_dominate=true;
        //刚从气压控高切换过来，时间间隔基于历史记录的时间。影响非常的小，可以忽略，如果是忽略，则以tof数据间隔调节高度。
        tgap= dtime - _height_adjust_queue.last().tmark; 


        // //记录控高切换时的数据
        // float gps_h;
        // critical_section_enter_blocking(&gps_queue_section);
        // gps_h=gps_queue.last(0).height;
        // critical_section_exit(&gps_queue_section);

        // //TOFSense-F2 Mini 可能确实有稳定问题，在高空常有这个信号出现，这不正常。
        // char log[128];
        // sprintf(log, "b2t, gps_h=%5.1f\n", gps_h);
        // logmessage(log);
        // sprintf(log, "t0=%u,%3.2f,%d\n", tof0.tmark, tof0.tof_distance,tof0.tof_strengh);
        // logmessage(log);
        // sprintf(log, "t1=%u,%3.2f,%d\n", tof1.tmark, tof1.tof_distance,tof1.tof_strengh);
        // logmessage(log);
        // sprintf(log, "t2=%u,%3.2f,%d\n", tof2.tmark, tof2.tof_distance,tof2.tof_strengh);
        // logmessage(log);
    }

    float zaxis, vib_z;
    float imuvacc= get_refer_imu_vacc_for_height_control(zaxis, vib_z);


    float Landing_Height= 0.15 + initial_tof_oflow_info.fixed_tof_distance; //起飞测距+0.15m宽容度。

    //三段式，第一段是检查是否符合取消降落条件。
    //第二段是执行降落，第三段是检查是否符合降落条件。

    //取消平滑降落的检查。
    if(micro_task.smooth_landing && micro_task.smooth_landing_triger_reason==3 && zaxis>0)
    {
        //因机身侧翻激活的降落过程，纠正后取消。
        micro_task.smooth_landing=0;
        micro_task.smooth_landing_triger_reason=0;
    }

    if(micro_task.smooth_landing && micro_task.smooth_landing_triger_reason==1)
    {
        //因近距离激活的降落过程，因距离偏大取消，或请求升高取消。
        if(micro_task.vspeed <=0)
        {
            micro_task.smooth_landing=0; //取消平滑降落过程。继续后期调节。
            micro_task.smooth_landing_triger_reason=0;
            logmessage("cancel landing for vspeed<=0\n");
        }

        //进入降落过程的，要退出去，必须测距大于降落高度一点，否则可能引起来回拉扯。
        if(tof0.fixed_tof_distance > Landing_Height + 0.15)
        {
            micro_task.smooth_landing=0; //取消平滑降落过程。继续后期调节。
            micro_task.smooth_landing_triger_reason=0;
            char dbg[64];
            sprintf(dbg, "cancel landing for dist=%4.3f, thres=%4.3f\n", tof0.fixed_tof_distance, Landing_Height+0.15);
            logmessage(dbg);
        }
    }

    //现在首先检查是否在平滑降落过程。
    if(micro_task.smooth_landing)
    {
        //在2秒内均匀减力到0.7，然后直接关机。

            //执行平滑降落并返回。
            float ref_thrust=_height_adjust_queue.last().thrust; //这里不使用过去平均推力了。
            //已经减力了多久？减力持续了2.5秒，直接停机。
            uint32_t tt= dtime - micro_task.smooth_landing;

            if(tt>=1500 || ref_thrust <= 0.5) {
                
                landed=true;
                _inair=false;
                logmessage("smooth landing done\n");
                height_adjust_record har;
                har.tmark= dtime;
                har.thrust = 0;
                har.hd = 0;
                har.spd = 0;
                har.current=system_power_current;
                _height_adjust_queue.push(har);
                return true;
            }else{

                //还有多少时间可以用来减力？目标是在余下的时间减小到0.5;
                uint32_t tl= 1500 - tt; //毫秒，剩余的时间。 >0
                float td = ref_thrust - 0.5; //要减少的力度  >0
                uint32_t gp= dtime - _height_adjust_queue.last().tmark; //本次调节与上次调节的时间间隔 >0 
                float adj = td * gp/tl; //本次调节。
                FLOAT_LIMIT(adj, 0.001, 0.2); //限制最小调节量。

                landed=false;
                height_adjust_record har;
                har.tmark= dtime;
                har.thrust = ref_thrust - adj;
                har.hd = 0;
                har.spd = 0;
                har.current=system_power_current;
                _height_adjust_queue.push(har);
                return true;
            }

    }
    
    //检查是否进入平滑降落。在平滑降落过程中。这里设定0.2m内打开平滑着陆，高脚机架要调大。
    //20240229， 一架测试机在十几米高空激活了平滑降落，随后关机，当时打了摇杆向下。这说明测距模块测出了一个小距离？
    if(!micro_task.smooth_landing && micro_task.vspeed>0)
    {
        //更严格的进入条件。防止tof数据偶发错误。
        //tof0,1,2的有效性前面的代码有保证，可以直接用。只有tof3的有效性需要检查。
        if( tof3.tof_valid && 
            tof0.fixed_tof_distance < Landing_Height && tof1.fixed_tof_distance < Landing_Height && 
            tof2.fixed_tof_distance < Landing_Height && tof3.fixed_tof_distance < Landing_Height)
        {
            micro_task.smooth_landing=now;
            micro_task.smooth_landing_triger_reason=1;
            logmessage("getin smooth landing in tof,reason=1\n");
        }
    }

    //求速度。用来匹配当前设置速度。
    float tof_vspd;

    if(tof3.tof_valid)
    {
        //tof3跨度较大，不一定是有效测距，有效就利用，无效就不用
        //距离差，米，
        float tof_dist_dif = tof3.fixed_tof_distance - tof0.fixed_tof_distance;
         //速度
        tof_vspd = tof_dist_dif*1000.0f/float(tof0.tmark - tof3.tmark); //m/s  >0 速度向下。
    }
    else
    {
        //距离差，米，
        float tof_dist_dif = tof2.fixed_tof_distance - tof0.fixed_tof_distance;
         //速度
        tof_vspd = tof_dist_dif*1000.0f/float(tof0.tmark - tof2.tmark); //m/s  >0 速度向下。
    }



    //测量加速度，用来替换imu的加速度。
    float tof_v0= (tof0.fixed_tof_distance - tof1.fixed_tof_distance)*1000.0f/(tof0.tmark - tof1.tmark); //速度向上为正
    float tof_v1= (tof1.fixed_tof_distance - tof2.fixed_tof_distance)*1000.0f/(tof1.tmark - tof2.tmark); //速度向上为正
    float tof_acc= (tof_v1 - tof_v0)*1000.0f/(tof0.tmark - tof1.tmark); //加速度，下为正。
    FLOAT_LIMIT(tof_acc, -5.0, 5.0); 

    float tof_dif = tof0.fixed_tof_distance - tof1.fixed_tof_distance; //微分调节项。


    //近地面速度修正，限制过大降落速度。最大降速1.0m/s
    int8_t fix_spd= micro_task.vspeed;

    if(fix_spd >0)
    {
        //20240201 稍放开点限制，允许更大的速度。
        //20250406 再放松点限制，避免机身共振无法降落
        if(tof0.fixed_tof_distance < 0.6) {
            if(fix_spd > 5) fix_spd=5;
        }else if(tof0.fixed_tof_distance < 0.9) {
            if(fix_spd > 6) fix_spd = 6;
        }else if(tof0.fixed_tof_distance < 1.2) {
            if(fix_spd > 7) fix_spd = 7;
        }else if(tof0.fixed_tof_distance < 1.8) {
            if(fix_spd > 8) fix_spd = 8;
        }else if(tof0.fixed_tof_distance < 2.4) {
            if(fix_spd > 9) fix_spd = 9;
        }else if(tof0.fixed_tof_distance < 3.2) {
            if(fix_spd > 10) fix_spd = 10;
        }else{
            if(fix_spd > 11) fix_spd = 11;
        }
    }

    //以上调节垂直速度，以下调节控制高度。
    //如果设定了垂直速度，这里首先调节设定高度。这次的高度调节不反应在本次高差上。下次调节起作用。
    float flex=0;
    //不在起飞阶段才允许调节设定高度。
    if(fix_spd!=0 && !micro_task.help_takeoff)  //vspeed现在有向，负值是调高，正值是调低。
    {
        //给出下一个信号应该盯住的高度。
        //高度设定可以是负数。
        micro_task.tof_height -= tgap * 0.001 * fix_spd*0.1; //vspeed>0 速度向下。
        if(micro_task.tof_height > tof0.fixed_tof_distance + 0.2) micro_task.tof_height= tof0.fixed_tof_distance + 0.2;
        if(micro_task.tof_height < tof0.fixed_tof_distance - 0.2) micro_task.tof_height= tof0.fixed_tof_distance - 0.2;
        flex = SigmoidTrans(fabs(fix_spd*0.1));
    }

    //这个高度不能总认为会很小，风可能吹高吹低机身位置。
    float tofhd= micro_task.tof_height - tof0.fixed_tof_distance;
    float accu_tofhd= micro_task.tof_height - accu_tof_h;

    //起飞常量推力可以考虑取消。
    float takeoff_const_thrust=0; //如果是起飞阶段，增加一个常量给推力。

    if(micro_task.help_takeoff)
    {
        
        // if(tof0.fixed_tof_distance > initial_tof_oflow_info.fixed_tof_distance + 0.2)
        // {
        //     _inair=true;
        // }

        //新办法。
        if(tof0.fixed_tof_distance < initial_tof_oflow_info.fixed_tof_distance + 0.1)
        {
            takeoff_const_thrust = 0.01;
        }
        else if(tof0.fixed_tof_distance < initial_tof_oflow_info.fixed_tof_distance + 0.25)
        {
            takeoff_const_thrust = 0.005;
        }
        else if(tof0.fixed_tof_distance < initial_tof_oflow_info.fixed_tof_distance + 0.35)
        {
            takeoff_const_thrust = 0.003;
        }
        else if(tof0.fixed_tof_distance < initial_tof_oflow_info.fixed_tof_distance + 0.5)
        {
            takeoff_const_thrust = 0.001;
        }
        else
        {
            micro_task.help_takeoff=false; //已经辅助增力到位，关闭辅助推力，以后是遥控器控制高低。
            _inair=true; //标记已在空中。
        }

    }

    //20240223+ 
    if(micro_task.height_break_tmark)
    {
        //刹车阶段取消高差调节，即不使用稳定高度。有imu阻尼，速度阻尼，速度调节。最终速度会回到0.
        tofhd=0;
        accu_tofhd=0;

        float thres=0.2;
        if(dtime < micro_task.height_break_tmark + 3000) thres = 0.15;
        else if(dtime < micro_task.height_break_tmark + 6000) thres=0.20;
        else if(dtime < micro_task.height_break_tmark + 9000) thres=0.25;
        else thres=0.3;

        //刹车标记。vspeed一定为0，此时调节倾向于抑制速度加速度。也有临时的稳定气压用来高差调节，然后速度和加速度降低到一定程度时，设置稳定气压。
        if(fabs(tof_vspd)<thres && fabs(imuvacc) < thres)
        {
            micro_task.tof_height=tof0.fixed_tof_distance; //设置当前稳定高度
            micro_task.height_break_tmark=0; //取消刹车标记。
        }
    }

//原0.1，改为0.08，这里和气压控高系数不同，因为调节频率这里更大。100hz, 气压那边是50hz.
//改回0.1，因以后也考虑用50hz测距，0.08对于50hz测距调节太软
//#define RATIO (0.1) 
//#define RATIO (0.1) //2806比2306动力强太多，控高过于极端，减力过大导致砸落地面。
//不能有太大的加速度和速度，向上和向下的加速度和速度都不能大。所以速度和加速度的调节参数较大。
//单独的kp导致上下跳跃更大。
#define Kp (0.07*TOF_HEIGHT_RATIO) //高差调节系数，高差来源于tof测量。

#define Ki (0.04*TOF_HEIGHT_RATIO) //积分调节参数

//加强速度匹配因子0.1->0.3
//由于向上大幅推杆后突然放松，机器会快速冲高然后迅速减力跌落，所以速度调节系数减小，原来0.5，现在改0.3，20231031
#define Kd (0.35*TOF_HEIGHT_RATIO) //速度调节系数。因高差限制的较小，所以速度调节应该加大才对。
//在tof有效下，这个可以调低
//尝试降低这个调节值，减小依赖以防止震动带来过大影响。
#define Kx (0.40*TOF_HEIGHT_RATIO) //加速度做阻尼很有效，可能是因为速度快，延迟小。但机架震动时很麻烦，容易失控。
//减小为0.25，期待看到在测距控高下的高度上下波动，然后去寻找最佳参考推力延迟来减小震荡。
//分别减小到0.25和0.035，高度波动不明显，继续降低。
//分别减小到0.15和0.02，高度波动也不明显，继续降低。
//分别减小到0.08和0.012，则高度严重震荡，几乎无法控制，最后是上升到气压控制区缓慢降落才控制下来。
//#define Kx (0.20*RATIO) 
#define Kxa (0.05*TOF_HEIGHT_RATIO) //加速度调节项。

//由于向上大幅推杆后突然放松，机器会快速冲高然后迅速减力跌落，所以阻尼加大，原来0.1，现在改0.4，0.8太大，20231031
#define Ks (0.4*TOF_HEIGHT_RATIO) //存粹速度阻尼项。
#define Ktx (0.1*TOF_HEIGHT_RATIO) //tof 加速度阻尼
#define Kz (0.2*TOF_HEIGHT_RATIO) //测距微分阻尼


    //这里不对称限制高差是希望下落时慢点。地面有深坑时不至于迅速坠下。地面隆起时可以较快的提高高度。
    FLOAT_LIMIT(tofhd, -2.0, 2.0); //当前高度高于设定值是负数，限制负值可以使降落速度慢点。

    

    //速度似乎不应该盯住设定速度，高度已经包含了速度因素。
    //期望的速度，向下为正，向上为负。和距离相反。
    float wt_spd; 
    if(fix_spd==0) {
        wt_spd= -tofhd/3.0;
        FLOAT_LIMIT(wt_spd, -0.4, 0.2); //下行速度要慢，上行速度可以快点。
    }else{
        wt_spd=fix_spd*0.1;
        tofhd=0; //有速度设定时，不应该有高差调节项。20241204+
        accu_tofhd=0;
    }

    float trans_tofhd = SigmoidTrans(tofhd);

    //速度向下为正，向上为负。
    float spd_dif= wt_spd - tof_vspd; //速度差, <0希望加力，是减项  tof_vspd<0速度向上，遇到坑就会表现为速度向上。
    float wt_acc = spd_dif; //希望一秒种加速到位，填补速度差。>0希望加速度向下，<0希望加速度向上。

    FLOAT_LIMIT(wt_acc, -0.5, 0.5); //20240909，将加速度抑制在一个很小的范围。近地加速度太大易撞地。

    float trans_spd_dif = -SigmoidTrans(spd_dif); //这里取反了

    //限制范围削弱了加速度的阻尼，但可以避免部分噪音。
    //相当于限制 加速度在1.0m/s^2以内，超过的无能为力。
    //如果安装减震脚，则可以避免大噪音，这里可以处理更大的加速度。
    //20230503放大限制范围到0.15以内，机身有减震，代码有噪音防备。
    //20230513放开限制到0.2以内。有噪音防备代码。
    FLOAT_LIMIT(imuvacc, -3.0, 3.0); 

    float acc_dif= imuvacc - wt_acc; //加速度和测量加速度的差异。如果>0当前向下加速度太大，要加力。
    float trans_acc_dif= SigmoidTrans(acc_dif);

    FLOAT_LIMIT(accu_tofhd, -0.08, 0.08); //积分用来稳定高度，不用来调节大高差。这和balance里的积分类似。
    float trans_accu_tofhd= SigmoidTrans(accu_tofhd); //积分高差

    float st10 = Kp*trans_tofhd; //高差调节
    float st11= Ki*trans_accu_tofhd; //积分高差调节。
    //这里的阻尼需要考虑期望的加速度才更灵活，有时需要快速降低下降速度，而此时单纯阻尼会减低向上加速的效应。
    float st12 = Kx*imuvacc*(1.2-flex); //修改为纯阻尼，后面有调节项。20240909
    float st13 = Kd*trans_spd_dif; //速度比对项，反应速度差的调节，>0则速度比设定值更向下，加力。
    float st14 = Ks*SigmoidTrans(tof_vspd)*(1.2-flex); //速度阻尼项，速度向下就加力，向上就减力
    float st15 = Kxa*trans_acc_dif*(1.2-flex); //加速度调节项。
    float st16 = Kz*tof_dif*(1.05-flex); //测距微分阻尼项。20240223+
    float st17 = Ktx*SigmoidTrans(tof_acc)*(1.2-flex); //tof测量加速度阻尼项。

    float change= st10+st11+st12+st13+st14+st15+st16+st17;

    if(micro_task.vspeed==0)
    {
        //在无速度设定时，限制上下行速度，避免震荡加大。
        if(tof_vspd>0.5 && change<0.0)
            change+= Ks*SigmoidTrans(tof_vspd - 0.5);
        else if(tof_vspd<-0.5 && change>0.0) //向上速度超过0.5m/s, 并且在增力，减小力度。
            change+= Ks*SigmoidTrans(tof_vspd + 0.5); //是个负值。相当于减力。
    }

//二次调节

    float ref_thrust=get_refer_thrust_for_height_control();

//选用10个数据间隔。
    //时间差
    float time_dif10= _height_adjust_queue.last(0).tmark - _height_adjust_queue.last(10).tmark;

    float sumhd_near=0;
    for(size_t i=0;i<10;i++) {
        sumhd_near+= _height_adjust_queue.last(i).hd;
    }
    sumhd_near/=10;

    float sumhd_mid=0; //更早一点的高度差，
    for(size_t i=10;i<20;i++) {
        sumhd_mid+= _height_adjust_queue.last(i).hd;
    }
    sumhd_mid/=10;

    float sumhd_far=0; //更早一点的高度差，
    for(size_t i=20;i<30;i++) {
        sumhd_far+= _height_adjust_queue.last(i).hd;
    }
    sumhd_far/=10;

    //统一的处理方式，可能要再考虑一个积分。即历史高差序列。
    //新法考虑不同的调节系数在不同的距离上。
    float spd_ratio= (1000.0f/time_dif10);
    float sspd1 = (sumhd_near - sumhd_mid)*spd_ratio; //速度向下为正。
    float sspd2 = (sumhd_mid - sumhd_far)*spd_ratio; //上一个速度

    float sacc = (sspd1 - sspd2)*spd_ratio; //加速度, 下为正，因采样时间缩短，sacc含有较大噪音，可能需要滤波

//这里的速度和加速度不匹配设定的vspeed，而是盯住设定的高度。
    float wt_sspd = -sumhd_near;
    float wt_acc2 = wt_sspd - sspd1; //向上为负


    //近处精细，远端钝化，开根号
    //增大参数导致波动稍大
    float t_hd, t_spd, t_acc;
    t_hd= sumhd_near;
    FLOAT_LIMIT(t_hd, -1.0, 1.0);

    //这里的速度并不追逐设定速度，而是追逐设定的目标高度。
    t_spd= sspd1 - wt_sspd;
    FLOAT_LIMIT(t_spd, -1.0, 1.0);

    t_acc = SigmoidTrans(sacc-wt_acc2);

//参数太弱了在纯气压下不容易稳定高度
//20240201取消R2,统一使用RATIO。
//#define R2 (0.1)

    //在偏离设定高度时，逐步去掉由加速度带来的影响，防止由于机身抖动带来的持续加速上升意外。
    //越是偏高越是去除的多。
    if(sumhd_near < -1.0) {
        change -= st12;
    }
    else if(sumhd_near < -0.8 ) {
        change -= st12*0.95; 
    } 
    else if(sumhd_near < -0.6 ) {
        change -= st12*0.9; 
    }
    else if(sumhd_near < -0.5 ) {
        change -= st12*0.7; 
    }
    else if(sumhd_near <-0.4 ) {
        change -= st12*0.5; 
    }
    else if(sumhd_near <-0.3 ) {
        change -= st12*0.3; 
    }
    else if(sumhd_near <-0.2) {
        change -= st12*0.2; 
    }
    else if(sumhd_near <-0.1 ) {
        change -= st12*0.1;
    }

//高差调节
    float adj5 = t_hd*0.1*TOF_HEIGHT_RATIO*(1.2-flex); 
    change += adj5;

//速度调节
    float adj6 = t_spd*0.2*TOF_HEIGHT_RATIO*(1.2-flex); 
    change += adj6;

//加速度调节,阻尼
    float adj7 = t_acc*0.3*TOF_HEIGHT_RATIO*(1.2-flex); 
    change += adj7;

    //单次调节限制。太大了震荡大。
    //20240201，限制调节为0.1，之前0.5
    float onetime= ref_thrust*0.08;
    FLOAT_LIMIT(change, -0.15, 0.15); //这里比气压那边放的更大，近地需要迅速控制速度。
    FLOAT_LIMIT(change, -onetime, onetime);
    
    change += takeoff_const_thrust; //起飞的持续增力,不受限制。

    float newthrust = ref_thrust + change; 

    //上下限。
    FLOAT_LIMIT(newthrust, botlmt, toplmt);

    if(zaxis<0 && !micro_task.smooth_landing)
    {
        if(!micro_task.smooth_landing)
        {
            FLOAT_LIMIT(newthrust, botlmt, 1.0); //颠倒后，限制最大推力1.0
            micro_task.smooth_landing=dtime;
            micro_task.smooth_landing_triger_reason=3; //因机身侧翻激活自动降落，以保护电机电调。
            logmessage("set smooth landing in tof for upside down\n");
        }
        else if(dtime - micro_task.smooth_landing > 1000)
        {
            //似乎不会在这里触发。
            logmessage("upside down, 1s landed\n");
            landed=true;
            return true;
        }
    }

    height_adjust_record har;
    har.tmark= dtime;
    har.thrust = newthrust;
    har.hd = tofhd;
    har.spd = tof_vspd;
    har.current=system_power_current;
    _height_adjust_queue.push(har);

    landed=false;
    return true;

#undef Kp
#undef Ki
#undef Kx
#undef Kd
#undef Ks
#undef Ktx
#undef Kxa
#undef Kz
//#undef RATIO
}
#endif

//20240321 新增微任务空闲模式，这个模式里微任务没有目的，仅保持定高，平衡，定点
//有目的性的操作都由宏任务控制，比如机身旋转，水平移动，高度调节等。
//遥控任务不会使用这个，所以这个任务里面始终存在锚点。无论是水平飞行还是到达目标点后的定点，都有一个锚点。
//所以这里面始终使用pin_to_location来水平定点。
//暂时未使用。
void CMicroTaskExec::micro_task_idle()
{

    if(micro_task.status==_micro_task_status_none)
    {
        micro_task.status=_micro_task_status_inprogress;
    }

    //高度调节给出的变化量基于上一个负担是最好的表现，这可能是因为气压计延迟很小
    bool landed;
    height_adjust_with_landing_detect(INAIR_BOTTHRUST, INAIR_TOPTHRUST, landed);


    float pb=0;
    float rb=0;
 
#if defined(USE_GPS) && defined(USE_OFLOW) 
    //gps定点优先使用。
    float cur_dist, cur_spd, cur_dir, cur_height;
    bool newdata;
    bool b1=pin_to_location(cur_dist, cur_spd, cur_dir, cur_height, newdata); 
    if(b1) {
        pb= gps_major_bias_queue.last().pitch_bias;
        rb= gps_major_bias_queue.last().roll_bias;
    }else{
        bool b0=oflow_position_adjust_minor_change(); 
        if(b0){
            pb= oflow_bias_queue.last().pitch_bias;
            rb= oflow_bias_queue.last().roll_bias;
        }
    }
#elif defined(USE_GPS)
    float cur_dist, cur_spd, cur_dir, cur_height;
    bool newdata;
    bool b1=pin_to_location(cur_dist, cur_spd, cur_dir, cur_height, newdata);
    if(b1) {
        pb= gps_major_bias_queue.last().pitch_bias;
        rb= gps_major_bias_queue.last().roll_bias;
    }
#elif defined(USE_OFLOW)
    //光流定点优先使用。
    bool b0=oflow_position_adjust_minor_change(); 
    if(b0){
        pb= oflow_bias_queue.last().pitch_bias;
        rb= oflow_bias_queue.last().roll_bias;
    }
#endif

    //这是用来执行自动任务的，不考虑摇杆位置。
    float new_burdens[4];
    float new_thrust;
    balance(new_burdens, new_thrust, pb , rb );
    motor_ctrl.SetBurdens(new_burdens, new_thrust);

}

//合适的水平方向速度，采用分段连续函数
//return proper speed for Pin_to_location
//这个函数主要用于处理接近目标时的速度平滑递减。采用动能线性递减则速度就是根号递减。
//速度的线性递减似乎总是导致顿挫，不易弄得平滑。所以改用速度的根号递减。
//目前这个版本不再使用，用的是带gps_quality输入的版本。
float CMicroTaskExec::get_proper_hspd(float dist)
{

    //20240715 修改，细分5米内的速度，因为之前的算法应用在定点上漂摆速度很快
    //在很接近目标点时其实不需要那么大的调节速度，显得不够稳。
    if(dist>5.0) {
        return sqrt(1.2*dist);
    }else if(dist>4.0) {
        return dist*0.49;
    }else if(dist>3.0) {
        return dist*0.35;
    }else if(dist>2.0) {
        return dist*0.25;
    }else if(dist>1.0) {
        return dist*0.15;
    }else{
        return dist*0.12;
    }
}

float CMicroTaskExec::get_proper_hspd(float dist, float gps_quality)
{

    //20240912 新增参数gps_quality，在20米距离内作用。gps_quality在0.4-1.0间，质量越好数字越大
    
    if(dist>5.0) {
        if(dist>20.0) return sqrt(1.2*dist); //20米距离外不依赖gps数据质量。
        else return sqrt(1.2*dist)*gps_quality; //近距离时再乘以一个调节。
        //20250331 降低进近的速度，去掉*1.2系数，避免近进加速度过大。
        //if(dist>30.0) return sqrt(dist);
        //else return sqrt(dist)*gps_quality;
    }else if(dist>4.0) {
        return dist*0.49*gps_quality;
    }else if(dist>3.0) {
        return dist*0.35*gps_quality;
    }else if(dist>2.0) {
        return dist*0.25*gps_quality;
    }else if(dist>1.0) {
        return dist*0.15*gps_quality;
    }else{
        return dist*0.12*gps_quality;
    }
}

//20240416+ 不同的距离有不同的速度，加速度限制，使运动过程更温柔，靠近目标也能定点。
//pin_to_location使用，距离目标远时，降低其调节力度，缓加速。近时增强，稳定定位。

//spd_angle，速度和目标方向的夹角，-180-180
//heading_spd, 指向目标的速度。
//20241124 修改，增强调节力，怕在顺风时到达目标点刹车不够。还有途中遇到强风，限制的太小也容易产生路线偏离
void CMicroTaskExec::get_spd_acc_lmt_by_dist(float dist, float spd_angle, float& spdlmt, float& acclmt)
{
    if(dist>200.0) {
        spdlmt=0.4;
        acclmt=1.6;
    }else if(dist>100) {
        spdlmt=0.5;
        acclmt=2.0;
    }else if(dist>50) {
        spdlmt=0.7;
        acclmt=2.3;
    }else if(dist>10) {
        spdlmt=1.3;
        acclmt=2.7;
    }else{
        spdlmt=1.8;
        acclmt=3.2;
    }

    if(fabs(spd_angle) > 90)
    {
        spdlmt*=2.1;
        acclmt*=2.1;
    }
    else if(fabs(spd_angle) > 60)
    {
        spdlmt*=1.8;
        acclmt*=1.8;
    }
    else if(fabs(spd_angle) > 30)
    {
        spdlmt*=1.5;
        acclmt*=1.5; 
    }
    else if(fabs(spd_angle) > 15)
    {
        spdlmt*=1.2;
        acclmt*=1.2;    
    }
}

//不超过1.0
float CMicroTaskExec::acc_match_adjust_strengh(float accnorm)
{
    if(accnorm<0.05) return 0;
    else if(accnorm < 0.1) return 0.01;
    else if(accnorm < 0.2) return 0.02;
    else if(accnorm < 0.3) return 0.04;
    else if(accnorm < 0.4) return 0.08;
    else if(accnorm < 0.5) return 0.1;
    else if(accnorm < 0.6) return 0.12;
    else if(accnorm < 0.7) return 0.15;
    else if(accnorm < 0.8) return 0.18;
    else if(accnorm < 0.9) return 0.21;
    else if(accnorm < 1.0) return 0.24;
    else if(accnorm < 1.2) return 0.27;
    else if(accnorm < 1.4) return 0.3;
    else if(accnorm < 1.6) return 0.33;
    else return 0.36;
}

//将坐标系逆时针旋转angle角度（度数）得到新的坐标位置。
//如果是点(x,y)自身绕原点旋转某角度，相当于坐标系反旋转得到的位置。
void CMicroTaskExec::rotate_ccw(float x, float y, float angle, float& nx, float& ny)
{
    float a= angle/57.2958;
    float cosA= cos(a);
    float sinA= sin(a);
    nx= x*cosA + y*sinA;
    ny= -x*sinA + y*cosA;
}