
#include "macro_task.h"
#include "macro_task_exec.h"
#include "gps_ublox.h"
#include <math.h>
extern void logmessage(std::string str);
extern uint32_t get_time_mark();
extern float Compass_Heading();

extern _micro_task micro_task;
extern NRandQueue<imu_info> imu_queue;
extern NRandQueue<press_info> press_queue;
#if defined USE_GPS
extern NRandQueue<gps_info> gps_queue;
extern critical_section_t gps_queue_section;
extern gps_info initial_gps_info;
#endif
extern NRandQueue<power_info> power_queue;

extern imu_info initial_imu_info;
extern press_info initial_press_info;
extern critical_section_t press_queue_section;
extern critical_section_t imu_queue_section;
extern critical_section_t tof_queue_section;

//全局共享空中标记
extern bool _inair;
extern bool _intof; //高度控制是否处于测距模式下。

//全局共享机头指向
extern float _heading_yaw;
#if defined USE_QMC5883 || defined USE_HMC5883
extern uint32_t _heading_yaw_last_update_time;
#endif
//磁场中心值
extern float magnet_midx;
extern float magnet_midy;
extern bool update_magnet;
extern float system_power_used;

#ifdef USE_TOF
extern tof_oflow_info initial_tof_oflow_info; //初始tof信息
extern NRandQueue<tof_oflow_info> tof_oflow_queue;
extern critical_section_t tof_oflow_queue_section;
#endif

//发送遥控消息，用来调试，一般微任务不发送信息给遥控
extern void Send_remote_message(std::string msg);
extern void Send_remote_message(uint8_t msgid);
extern float air_press_height_refer(float press_base, float height_change);

extern float guard_energy_thres;   //0=不生效
extern float guard_voltage_thres;  //0=不生效



//关于磁偏角 https://www.ncei.noaa.gov/products/wandering-geomagnetic-poles 可以查到数据，历史及未来。
//这是最近几年的数据。然后根据这个数据可以推算定位点的磁偏角，数据一直在变化。需要每年更新。
// 东经，北纬，年份
// 164.036 86.502 2020.000
// 157.690 86.415 2021.000
// 151.948 86.294 2022.000
// 146.826 86.146 2023.000
// 142.293 85.980 2024.000
// 138.299 85.801 2025.000

#if defined USE_QMC5883 || defined USE_HMC5883
extern bool Read_Compass_Data(int16_t& xd, int16_t& yd, int16_t& zd);

// //用来记录指南针扫描数据。
class _compass_yaw {
    public:
    _compass_yaw() {
        last_update=0;
        compX=0;
        compY=0;
    }

    uint32_t last_update;
    float compX; //指南针数据
    float compY; //指南针数据
};

//不同的yaw角填入不同的位置里。按顺序-180~180每度一个位置。
NRandQueue<_compass_yaw> compass_yaw_map(360);

#endif


void CMacroTaskExec::do_front_task(bool& finish)
{
    if(_task_num==0) {
        finish=true;
        return;
    }

    do_macro_task(_tasks[0], finish);

    if(finish) {
        //移动后面的任务
        for(size_t i=1; i<_task_num;i++)
        {
            _tasks[i-1]=_tasks[i];
        }

        _task_num--; //扣减完成的当前任务。
    }
}

void CMacroTaskExec::do_macro_task(_macro_task& task, bool& finish)
{
    _running_task_id=task.taskid; //20240307+

    switch(task.taskid)
    {
        case _macro_hover:
            macro_hover(task, finish);
            break;
        case _macro_landing_coordinate:
            macro_landing_coordinate(task, finish);
            break;
        case _macro_landing_uncondition:
            macro_landing_uncondition(task, finish);
            break;
        case _macro_ground_up:
            macro_ground_up(task, finish);
            break;
        case _macro_test:
            macro_test(task, finish);
            break;
        case _macro_make_compass_map:
            macro_make_compass_map(task, finish);
            break;
        case _macro_pin_to_coordinate:
            macro_pin_to_coordinate(task, finish);
            break;
        case _macro_point_to_coordinate:
            macro_point_to_coordinate(task, finish);
            break;
        case _macro_vertical_height_adjust:
            macro_vertical_relative_height_adjust(task, finish);
            break;
        case _macro_vertical_absolute_height_adjust:
            macro_vertical_absolute_height_adjust(task, finish);
            break;
        case _macro_max_height_test:
            macro_max_height_test(task, finish);
            break;
        case _macro_energy_guard:
            {
                finish=true; //虚拟警卫任务，设置后立即结束。
                guard_energy_thres= task.guard_energy_thres;
            }
            break;
        case _macro_voltage_guard:
            {
                finish=true; //虚拟警卫任务，设置后立即结束。
                guard_voltage_thres= task.guard_voltage_thres;
            }
            break;
        default:
            {
                //未知任务，删除。
                logmessage("unknown tasks in do_macro_task!\n");
                finish=true;
            }
            break;
    }
}

//这是一个宏任务。自动调节垂直速度，达到预期的高度。
//如果是调低高度以至于降落到地面还达不到要求，那么可能导致任务无法完成而卡住。因为这里并不检查是否已经落地。
//将来可能有必要检查一下。
void CMacroTaskExec::macro_vertical_relative_height_adjust(_macro_task& task, bool& finish)
{
    finish=false;

    if(!_inair && !task.fired) {
        finish=true;
        return;
    }

    if(!task.fired)
    {

        gps_info last_gps;
        press_info pi;
        critical_section_enter_blocking(&press_queue_section);
        pi= press_queue.history_mean(0,10);  //当前气压值
        critical_section_exit(&press_queue_section);

        critical_section_enter_blocking(&gps_queue_section);
        last_gps= gps_queue.last();
        critical_section_exit(&gps_queue_section); 

        task.target_gps_height = last_gps.height + task.vertical_adjust_height;

        float target_press= air_press_height_refer(pi.press, task.vertical_adjust_height); //新的气压推断函数更准确
        
        if(target_press < 30500.0) {
            target_press=30500.0; //受到bmp390量程范围限制，其数据不可能低于30000，低于这个值永远达不到。附加500冗余值。
            Send_remote_message("targ press limit to 30500.");
            logmessage("targ press limit to 30500.\n");
        }

        float tor= fabs(target_press - pi.press)*0.05; //5%气压容错范围
        if(tor<12.0) tor=12.0;

        //交给微任务处理。
        micro_task.taskid=_micro_task_id_nop; //相当于关闭微任务。
        micro_task.status=_micro_task_status_none;
        micro_task.roll=0;
        micro_task.pitch=0;
        micro_task.yawspeed=task.want_yaw_speed;
        task.old_yaw = micro_task.yaw; //记录初始角度
        task.press= target_press;  //记录目标气压, 
        task.press_high = target_press+tor; //气压范围设定
        task.press_low = target_press-tor;
        //标记方向。
        if(task.vertical_adjust_height>=0) {
            task.vertical_climb=true;
        }
        else {
            task.vertical_climb=false;
        }

        micro_task.help_takeoff=false;
        micro_task.taskid=_micro_task_id_keep_stable_withyaw;

        task.fired=true;
        Send_remote_message("fire relative htadj");

        char log[128];
        sprintf(log, "rehadj adjh=%4.2f, curh=%4.2f, targh=%4.2f, curp=%6.2f, tp=%6.2f, lp=%6.2f, hp=%6.2f\n", 
        task.vertical_adjust_height, last_gps.height, task.target_gps_height, pi.press, 
             target_press, task.press_low, task.press_high);
        logmessage(log);
    }
    else
    {
        //观察高差，调节垂直速度。
        press_info pi;
        gps_info last_gps;

        critical_section_enter_blocking(&press_queue_section);
        pi= press_queue.history_mean(0,10);  //当前气压值
        critical_section_exit(&press_queue_section);

        critical_section_enter_blocking(&gps_queue_section);
        last_gps= gps_queue.last();
        critical_section_exit(&gps_queue_section); 

        float hd=0; //hd>0,设定高度还在上方，需要爬升。<0需要降低。

        //新处理方法，区分爬升还是降低，20240626修改。
        if(task.vertical_climb)
        {
            //爬升，是否达到了气压下限？没有达到下限的，以目标气压做高差调节
            if(pi.press > task.press_high) {
                //未达下限，继续爬高
                hd= (pi.press - task.press)/11.2;
            }else if(pi.press < task.press_low) {
                //超过上限，停止
                micro_task.vspeed=0;
                micro_task.press= task.press_low; 
                micro_task.gps_ref_height= last_gps.height;
                micro_task.yawspeed=0;
                micro_task.yaw=task.old_yaw; //恢复初始角度
                finish=true;

                char log[128];
                sprintf(log, "relative hadj end by press low %6.2f.\n", task.press_low);
                logmessage(log);

                return;
            }
            else {
                //在气压上下限间，可以参考GPS
                if(last_gps.height >= task.target_gps_height)
                {
                    micro_task.vspeed=0;
                    micro_task.press= pi.press; //设置为当前气压。
                    micro_task.gps_ref_height= last_gps.height;
                    micro_task.yawspeed=0;
                    micro_task.yaw=task.old_yaw; //恢复初始角度
                    finish=true;

                    char log[128];
                    sprintf(log, "relative hadj end by gps, %f,%f,p=%6.2f.\n", last_gps.height, task.target_gps_height, pi.press);
                    logmessage(log);
                    return;
                }
                else
                {
                    hd= task.target_gps_height - last_gps.height;
                }
            }
        }
        else
        {
            //降低
            if(pi.press < task.press_low) {
                //未达上限，继续降低
                hd= (pi.press - task.press)/11.2;
            }else if(pi.press > task.press_high) {
                //低于下限，停止
                micro_task.vspeed=0;
                micro_task.press= task.press_high; 
                micro_task.gps_ref_height= last_gps.height;
                micro_task.yawspeed=0;
                micro_task.yaw=task.old_yaw; //恢复初始角度
                finish=true;
                char log[128];
                sprintf(log, "relative hadj end by press high %6.2f.\n", task.press_high);
                logmessage(log);
                return;
            }else {
                //在气压上下限间，可以参考GPS
                if(last_gps.height <= task.target_gps_height)
                {
                    micro_task.vspeed=0;
                    micro_task.press= pi.press; //设置为当前气压。
                    micro_task.gps_ref_height= last_gps.height;
                    micro_task.yawspeed=0;
                    micro_task.yaw=task.old_yaw; //恢复初始角度
                    finish=true;

                    char log[128];
                    sprintf(log, "relative hadj end by gps, %f,%f,p=%6.2f.\n", last_gps.height, task.target_gps_height, pi.press);
                    logmessage(log);
                    return;
 
                }
                else
                {
                    hd= task.target_gps_height - last_gps.height;
                }
            }
        }

        
        //执行高差调节。
        if(hd>0)
        {
                //上升。
            float spd= -hd/7.0;
            if(spd<-10.0) spd=-10.0; //最大上升速度10m/s
            if(spd>-1.2) spd=-1.2; //最小1.2

            micro_task.vspeed=10*spd;
        }
        else
        {
                //下降。20240508 新增防落地。
            if(_intof)
            {
                //下降已接近地面，不能再降低了，结束高度调节任务。
                micro_task.vspeed=0;
                micro_task.yawspeed=0;
                micro_task.press=pi.press;
                micro_task.yaw=task.old_yaw; //恢复初始角度
                finish=true;
                Send_remote_message("relative htadj end by tof");
            }
            else
            {
                float spd= -hd/12.0;    //下降比例因子大，不对称
                if(spd > 16.0) spd=16.0; //最大下降速度15m
                if(spd < 1.2) spd=1.2; //最小1.2

                micro_task.vspeed=10*spd;
            }

        }
    }
}

//绝对高度调节，调节标准以GPS高度为基准
//在控制上仍旧以气压为准，防止GPS伪造信号或干扰导致高度偏差。
//气压控制上下界来防止GPS数据异常。
//20241204 新发现一个问题，如果当前状态处于测距控高范围，
//那么向上爬高可能不执行，需要查找原因
void CMacroTaskExec::macro_vertical_absolute_height_adjust(_macro_task& task, bool& finish)
{
    finish=false;

    if(!_inair && !task.fired) {
        finish=true;
        return;
    }

    if(!task.fired)
    {

        //以起飞时的气压和GPS高度为准，计算气压基准。
        //float target_press= initial_press_info.press - (task.target_gps_height - initial_gps_info.height)*11.2;
        gps_info last_gps;

        press_info pi;
        critical_section_enter_blocking(&press_queue_section);
        pi= press_queue.history_mean(0,10);  //当前气压值
        critical_section_exit(&press_queue_section);

        critical_section_enter_blocking(&gps_queue_section);
        last_gps= gps_queue.last();
        critical_section_exit(&gps_queue_section); 

        //区分调节方向
        if(task.target_gps_height > last_gps.height) {
            task.vertical_climb=true;
        }
        else {
            task.vertical_climb=false;
        }

        //线性推断气压推断更准确。
        //float target_press= air_press_height_refer(initial_press_info.press, task.target_gps_height - initial_gps_info.height); 
        float target_press= air_press_height_refer(pi.press, task.target_gps_height - last_gps.height); //以当前气压和高度差推断目标气压。

        if(target_press < 30500.0) {
            target_press=30500.0; //受到bmp390量程范围限制，其数据不可能低于30000，低于这个值永远达不到。附加500冗余值。
            Send_remote_message("targ press limit to 30500.");
            logmessage("targ press limit to 30500.\n");
        }

        //这里容错范围3%小了，在地面气压101800时，气压偏高于日常，然后向上飞到海拔200米的气压计算偏离，导致在190米高度时触到了低压值停止上升了。
        //为了尽可能使用GPS定高，放松到5%
        float tor= fabs(target_press - pi.press)*0.05; //气压容错范围 5%
        if(tor<12.0) tor=12.0;

        //交给微任务处理。
        micro_task.taskid=_micro_task_id_nop; //相当于关闭微任务。
        micro_task.status=_micro_task_status_none;
        micro_task.roll=0;
        micro_task.pitch=0;
        micro_task.yawspeed=task.want_yaw_speed;

        task.old_yaw= micro_task.yaw; //记录初始角度，任务完成后恢复。
        task.press= target_press;  //记录目标气压, 
        task.press_high = target_press + tor;
        task.press_low = target_press - tor;
        micro_task.help_takeoff=false;
        micro_task.taskid=_micro_task_id_keep_stable_withyaw;

        task.fired=true;

        if(task.vertical_climb)
        {
            char msg[64];
            sprintf(msg, "htadj climb, %5.1f", task.target_gps_height);
            Send_remote_message(msg);
            logmessage("htadj climb\n");
        }    
        else
        {
            char msg[64];
            sprintf(msg, "htadj drop, %5.1f", task.target_gps_height);
            Send_remote_message(msg);
            logmessage("htadj drop\n");
        }

        char log[128];
        sprintf(log, "htadj h=%4.2f, tp=%6.2f, lp=%6.2f, hp=%6.2f, pu=%6.1f\n", 
            task.target_gps_height, target_press, task.press_low, task.press_high, system_power_used);
        logmessage(log);
    }
    else
    {
        //观察高差，调节垂直速度。
        //20241005 在测试4000米爬高时，由于当前气压模型计算的气压和实际偏差很大，
        //导致到达设定高度附近时，依旧根据气压差认为有很大高差而没有及时减速，需要考虑GPS高差来平衡速度控制。
        //问题的本质是，根据气压的推算，没有达到气压上限或下限，系统一直按照气压的大小去调节高差。这主要是为了防止GPS欺骗。
        //但气压计算偏差很大时，确实有问题，下降时，如果气压上限设定如果高于地面气压，那就会撞地。

        press_info pi;
        critical_section_enter_blocking(&press_queue_section);
        pi= press_queue.history_mean(0,10);  //当前气压值
        critical_section_exit(&press_queue_section);

        float hd=0; //hd>0,设定高度还在上方，需要爬升。<0需要降低。

        //新处理方法，区分爬升还是降低，20240626修改。
        if(task.vertical_climb)
        {

            //爬升，是否达到了气压下限？没有达到下限的，以目标气压做高差调节
            if(pi.press > task.press_high) {
                //未达下限，继续爬高
                hd= (pi.press - task.press)/11.2;  //>0
            }else if(pi.press < task.press_low) {
                //超过上限，停止
                micro_task.press= task.press_low; 
                gps_info last_gps;
                critical_section_enter_blocking(&gps_queue_section);
                last_gps= gps_queue.last();
                critical_section_exit(&gps_queue_section); 
                micro_task.gps_ref_height= last_gps.height;

                micro_task.yawspeed=0;
                micro_task.vspeed=0;
                micro_task.yaw=task.old_yaw; //恢复初始角度
                finish=true;

                char log[128];
                sprintf(log, "htadj end by press low %6.2f. pu=%6.1f\n", task.press_low, system_power_used);
                logmessage(log);

                return;
            }
            else {
                //在气压上下限间，可以参考GPS
                gps_info last_gps;
                critical_section_enter_blocking(&gps_queue_section);
                last_gps= gps_queue.last();
                critical_section_exit(&gps_queue_section); 
                if(last_gps.height >= task.target_gps_height)
                {
                    micro_task.vspeed=0;
                    micro_task.press= pi.press; //设置为当前气压。
                    micro_task.gps_ref_height= last_gps.height;
                    micro_task.yawspeed=0;
                    micro_task.yaw=task.old_yaw; //恢复初始角度
                    finish=true;

                    char log[128];
                    sprintf(log, "htadj climb end by gps, %4.1f,%4.1f,p=%6.2f,pu=%6.1f\n", last_gps.height, task.target_gps_height, pi.press, system_power_used);
                    logmessage(log);
                    return;
                }
                else
                {
                    hd= task.target_gps_height - last_gps.height;
                }
            }
        }
        else
        {
            //降低
            if(pi.press < task.press_low) {
                //未达上限，继续降低
                hd= (pi.press - task.press)/11.2; //负值
            }else if(pi.press > task.press_high) {
                //低于下限，停止
                micro_task.vspeed=0;
                micro_task.press= task.press_high; 
                gps_info last_gps;
                critical_section_enter_blocking(&gps_queue_section);
                last_gps= gps_queue.last();
                critical_section_exit(&gps_queue_section); 
                micro_task.gps_ref_height=last_gps.height;

                micro_task.yawspeed=0;
                micro_task.yaw=task.old_yaw; //恢复初始角度
                finish=true;
                return;
            }else {
                //在气压上下限间，可以参考GPS
                gps_info last_gps;
                critical_section_enter_blocking(&gps_queue_section);
                last_gps= gps_queue.last();
                critical_section_exit(&gps_queue_section); 

                if(last_gps.height <= task.target_gps_height)
                {
                    micro_task.vspeed=0;
                    micro_task.press= pi.press; //设置为当前气压。
                    micro_task.gps_ref_height= last_gps.height;
                    micro_task.yawspeed=0;
                    micro_task.yaw=task.old_yaw; //恢复初始角度
                    finish=true;

                    char log[128];
                    sprintf(log, "htadj drop end by gps, %f,%f,p=%6.2f,pu=%6.1f.\n", last_gps.height, task.target_gps_height, pi.press, system_power_used);
                    logmessage(log);
                    return;
 
                }
                else
                {
                    hd= task.target_gps_height - last_gps.height;

                    //出现反常，记录一下
                    if(hd>=0) {
                        char log[128];
                        sprintf(log, "htadj drop but hd>0 set by gps, %f,%f,%f\n", last_gps.height, task.target_gps_height, hd);
                        logmessage(log);
                    }
                }
            }
        }

        //执行高差调节。
        if(hd>0)
        {
            //上升。
            float spd= -hd/7.0;
            if(spd<-10.0) spd=-10.0; //最大上升速度10m/s
            if(spd>-1.2) spd=-1.2; //最小1.2

            micro_task.vspeed=10*spd;
        }
        else
        {
            //下降。20240508 新增防落地。
            if(_intof)
            {
                //下降已接近地面，不能再降低了，结束高度调节任务。
                micro_task.vspeed=0;
                micro_task.yawspeed=0;
                micro_task.press=pi.press;
                micro_task.yaw=task.old_yaw; //恢复初始角度
                finish=true;
                Send_remote_message("htadj end by tof");
            }
            else
            {
                float spd= -hd/12.0;    //下降比例因子大，不对称， 因子15偏大，最后阶段速度偏慢。
                if(spd > 16.0) spd=16.0; //最大下降速度16m
                if(spd < 1.2) spd=1.2; //最小1.2 放松最低速度到1.2，之前1.0
                micro_task.vspeed=10*spd;
            }

        }
    }
}

//这个做宏任务的辅助函数，不是一个任务。
//利用内部方向来指向目标点位。20240304修改。
void CMacroTaskExec::turn_head_to_coordinate(double lati, double logi)
{
#ifdef USE_GPS
    gps_info last_gps;
    float dist, angle;
    critical_section_enter_blocking(&gps_queue_section);
    last_gps=gps_queue.last();
    critical_section_exit(&gps_queue_section);

    global_gps_dist_angle(last_gps.latitude, last_gps.longitude, lati, logi, dist, angle);

    float yaw= _heading_yaw - angle;
    if(yaw > 180) yaw-=360;
    if(yaw < -180) yaw+=360;

    micro_task.yaw=yaw;

    char buf[64];
    sprintf(buf,"turn head, hy=%4.1f, dest angle=%4.1f, set yaw=%4.1f\n", _heading_yaw, angle, yaw);
    logmessage(buf);


#endif
}

//机头指向目标点方向。前提是已经方向定位，task带入目标的坐标。
//这个东西可以不做一个宏任务来处理，需要时直接设定yaw角度即可，也不用等待完成。
void CMacroTaskExec::macro_point_to_coordinate(_macro_task& task, bool& finish)
{
#ifdef USE_GPS
#if defined USE_QMC5883 || defined USE_HMC5883

    finish=false;

    if(!_inair && !task.fired) {
        finish=true;
        return;
    }

    if(!task.fired) {
        //初次进入任务状态，设置旋转，记录当前yaw。
        turn_head_to_coordinate(task.gps_latitude, task.gps_longitude);
        task.fired=true;
        return;
    }
    else
    {
        //观察yaw角是否接近设定。
        float cur_yaw;
        critical_section_enter_blocking(&imu_queue_section);
        cur_yaw = imu_queue.last().angle[2];
        critical_section_exit(&imu_queue_section);

        float angle= micro_task.yaw - cur_yaw; //求两者的夹角
        if(angle < -180) angle+=360;
        else if(angle > 180) angle-=360;

        if(fabs(angle)<3.0) {
            finish=true;
            return;
        }
    }
#endif
#endif
}

//基于指南针360度的数据更新指向角
bool CMacroTaskExec::UpdateHeadingYaw()
{

    #if defined USE_QMC5883 || defined USE_HMC5883
//寻找最大最小值。
    float max_magnet_x, min_magnet_x;
    float max_magnet_y, min_magnet_y;
    size_t min_magnet_x_pos=0;

    max_magnet_x=-50000;
    min_magnet_x=50000;
    max_magnet_y=-50000;
    min_magnet_y=50000;

    //检查一下有多少空洞数据。
    int empty=0;
    for(size_t i=0;i<360;i++) {
        if(compass_yaw_map.front(i).last_update==0) {
            empty++;
        }
    }

    char buf[64];
    sprintf(buf, "%d empty pos", empty);
    Send_remote_message(buf);
    
    //采用11均值, 部分角度上可能没有更新值，要避开无效数据。
    for(size_t i=0;i<360;i++) {

        float x= 0;
        float y= 0;

        int n=0;

        for(size_t j=i;j<i+11;j++)
        {
            if(compass_yaw_map.front(j).last_update!=0)
            {

                x+= compass_yaw_map.front(j).compX;
                y+= compass_yaw_map.front(j).compY;
                n++;
            }
        }

        x/=n;
        y/=n;

        if(x > max_magnet_x)
        {
            max_magnet_x = x;
        }

        if(x < min_magnet_x)
        {
            min_magnet_x=x;
            min_magnet_x_pos=i;
        }

        if(y> max_magnet_y)
        {
            max_magnet_y=y;
        }

        if(y< min_magnet_x)
        {
            min_magnet_y =y;
        }

    }


    float difx= max_magnet_x - min_magnet_x;
    float dify= max_magnet_y - min_magnet_y;

    //正常情况两头相差8000-9000，但可能受不同的电机影响有差异。
    //有时受干扰，差异能到3万多。有时在7300
    //标准放宽到4500-9000试试。Y方向似乎差别总是小点。放松一些，否则容易失败。主要看X方向。
    if(difx>5500 && difx < 9000 && dify > 3500 && dify<9000)
    {
        //设定机身北方指向。磁偏角暂时固定为2.8度。具体数据参考GPS.
        //因为上面采用5均值，所以最小x位置向后移动2个位置。
        min_magnet_x_pos+=5; //因为执行11个平均值
        min_magnet_x_pos%=360; //防越界。

        float minX_Yaw= float(min_magnet_x_pos) - 180.0;

        #if defined USE_GPS
        gps_info last_gps;
        float dist, magdec;
        critical_section_enter_blocking(&gps_queue_section);
        //所有GPS不提供磁偏角，自己算。
        last_gps=gps_queue.last();
        critical_section_exit(&gps_queue_section);

        //使用2023年的磁极位置。
        global_gps_dist_angle(last_gps.latitude, last_gps.longitude, 86.146, 146.826, dist, magdec);
        minX_Yaw-= magdec;  //顺时针旋转magdec
        #else
        minX_Yaw-=2.8f;
        #endif

        if(minX_Yaw <= -180.0) minX_Yaw+=360;
        if(minX_Yaw >= 180.0) minX_Yaw-=360;

        _heading_yaw=minX_Yaw;  //记录探测结果。
        _heading_yaw_last_update_time=get_time_mark(); 

        magnet_midx = float(min_magnet_x + max_magnet_x)/2.0;
        magnet_midy = float(min_magnet_y + max_magnet_y)/2.0;

        //一个正常的数据，头尾差似乎在8300-8800之间。即最大值减去最小值的范围。
        //如果这个值的范围在这个区间可以认为没有磁场干扰，否则认为有干扰。
        //因为测量范围是2Gs，头尾数据是-32768-32768，地磁场强度是500-600 mGs, 
        update_magnet=true;
        char buf[64];
        sprintf(buf, "good compass, headingyaw=%4.2f", _heading_yaw);
        Send_remote_message(buf);
        return true;
        
    }
    else
    {
        char buf[64];
        sprintf(buf, "bad compass, difx=%4.2f,dify=%4.2f", difx, dify);
        Send_remote_message(buf);
        return false;
    }

    #else
    return false;
    #endif
}

//这个版本好处是不需要一些静态变量做记录，随时可以做全部方向扫描或者部分方向扫描
//可以在飞行的同时持续进行指南针扫描定位方向。
//20230821 在定制机身上发生卡死，遥控无法控制。需要检查。
void CMacroTaskExec::macro_make_compass_map(_macro_task& task, bool& finish)
{
    #if defined USE_QMC5883 || defined USE_HMC5883

    finish=false;

    if(!_inair && !task.fired) {
        //机身不在空中，无法执行，任务结束。
        finish=true;
        return;
    }

    if(!task.fired) {
        //初次进入任务状态，设置旋转，记录当前yaw。

        //为判断是否旋转了一圈，清理所有记录的更新时间。
        for(size_t i=0;i<360;i++) {
            compass_yaw_map.front(i).last_update=0;
        }

        //为简化代码，这里只做初始化设置。
        micro_task.status = _micro_task_status_none;
        micro_task.taskid = _micro_task_id_keep_stable_withyaw; //指定微任务保持悬停，目前有光流定点
        micro_task.yawspeed = 3; //设置一个旋转速度，微任务将自动旋转机身。
        task.fired=true;
        return;
    }
    else
    {

        float curyaw;
        float curpitch;
        float curroll;
        critical_section_enter_blocking(&imu_queue_section);
        curpitch= imu_queue.last().angle[0];
        curroll= imu_queue.last().angle[1];
        curyaw = imu_queue.last().angle[2];
        critical_section_exit(&imu_queue_section);

        int ipos = int(curyaw+180);
        if(ipos<0) ipos=0; 
        if(ipos>359) ipos=359;

        uint32_t now = get_time_mark();

        //要更新的位置已经被更新了，而且上次更新距离现在超过1秒，不是同一时刻两次更新，这就是转了一圈了。
        uint32_t last_update = compass_yaw_map.front(ipos).last_update;
        if(last_update!=0 && now - last_update > 1000) 
        {

            //已经一圈了。结束。
            finish=true;
            //停止转动，
            micro_task.yawspeed=0;

            Send_remote_message("update heading");

            bool b=UpdateHeadingYaw(); //这里面设置了指向北方，将来去掉。

            if(b) {
                micro_task.yaw=_heading_yaw; //转向北方，需要时间去执行到位。
                Send_remote_message("update heading ok");
            }else{
                Send_remote_message("update heading fail");
            }

            Send_remote_message("quit update heading");

            return;
        }
        else if(last_update!=0) {
            //当前位置有数据了，可以不必再读。
            return;
        }
        else
        {
            //采集数据。
            int16_t cx,cy,cz;
            bool b=Read_Compass_Data(cx,cy,cz);

            if(b) {

                //做一个转化，全部调节为水平。参考ymfc-32
                //http://www.brokking.net/ymfc-32_downloads.html
                float compXH= cx*cos(curpitch*-0.0174533)
                +cy*sin(curroll*0.0174533)*sin(curpitch*-0.0174533)
                -cz*cos(curroll*0.0174533)*sin(curpitch*-0.0174533);

                float compYH= cy*cos(curroll*0.0174533)+cz*sin(curroll*0.0174533);

                compass_yaw_map.front(ipos).compX=compXH; //原为cx,替换掉，20230816+
                compass_yaw_map.front(ipos).compY=compYH; //原为cy,替换掉，20230816+
                compass_yaw_map.front(ipos).last_update=now;

            }else{
                //失败留下数据空洞。不算全部失败，个别空洞可以弥补。
                compass_yaw_map.front(ipos).compX=0; //原为cx,替换掉，20230816+
                compass_yaw_map.front(ipos).compY=0; //原为cy,替换掉，20230816+
                compass_yaw_map.front(ipos).last_update=0;
                finish=true; //一次读失败就全部失败。如果不是，可能会卡死，原因不明。
                Send_remote_message(40);//read compass failed
            }
            return;
        }

    }

    #else
        finish=true;
        return;
    #endif
}


//雪天起飞时的一个问题，飞机放在地面，测距模块沾上了雪花，起飞后测距始终是一个小数值，
//这可能导致起飞后一直上升。
void CMacroTaskExec::macro_ground_up(_macro_task& task, bool& finish)
{
    finish=false;

    if(_inair && !task.fired) {
        finish=true;
        _running_task_status=_macro_task_succeed;
        logmessage("already inair, no takeoff\n");
        return;
    }

    if(!task.fired) {

    //起飞前记录初始值。

        #ifdef USE_GPS
        critical_section_enter_blocking(&gps_queue_section);
        initial_gps_info= gps_queue.last();
        if(initial_gps_info.hacc>0.3)
        {
            //定位不是很精确，采用最近多个数据的平均值，减小误差。
            float hs=0;
            double lati=0;
            double logi=0;

            for(size_t i=0;i<8;i++) {
                hs+= gps_queue.last(i).height;
                lati+= gps_queue.last(i).latitude;
                logi+= gps_queue.last(i).longitude;
            }

            hs/=8.0;
            lati/=8.0;
            logi/=8.0;
            initial_gps_info.height=hs;
            initial_gps_info.latitude=lati;
            initial_gps_info.longitude=logi;
        }
        critical_section_exit(&gps_queue_section);
        #endif
    
       // #ifdef USE_ICM426XX
        critical_section_enter_blocking(&imu_queue_section);
        initial_imu_info= imu_queue.last();
        critical_section_exit(&imu_queue_section);
       // #endif

        #ifdef USE_TOF
        critical_section_enter_blocking(&tof_oflow_queue_section);
        initial_tof_oflow_info= tof_oflow_queue.last();
        //这里的测距主要用于降落的判断上。为了避免小测距的波动和不可靠性，计算一个平均值。
        int cnt=0;
        float sum_dist=0;
        uint16_t sum_strengh=0;
        for(size_t i=0;i<30;i++) //最多有40个记录，所以不能追溯的太远。
        {
            if(tof_oflow_queue.last(i).tof_valid)
            {
                cnt++;
                sum_dist+= tof_oflow_queue.last(i).fixed_tof_distance;
                sum_strengh+= tof_oflow_queue.last(i).tof_strengh;
            }
        }
        if(cnt>0) {
            sum_dist/=cnt;
            sum_strengh/=cnt;
        }
        initial_tof_oflow_info.fixed_tof_distance=sum_dist;//用平均值替换最后一个测距
        if(initial_tof_oflow_info.fixed_tof_distance > 0.2) initial_tof_oflow_info.fixed_tof_distance=0.2;
        else if(initial_tof_oflow_info.fixed_tof_distance <=0.0) initial_tof_oflow_info.fixed_tof_distance=0.0;
        critical_section_exit(&tof_oflow_queue_section);

        char dbg[128];
        sprintf(dbg, "init tof=%3.2f, tof_strengh=%d", initial_tof_oflow_info.fixed_tof_distance, sum_strengh);
        Send_remote_message(dbg);

        if(cnt==0) {
            //没有有效测距数据，无法起飞。
            finish=true;
            _inair=false;
            _running_task_status=_macro_task_failed;
            Send_remote_message("no valid tof data! fail to takeoff.");
            return;
        }
        //初始测距存在问题，结束起飞，需要检查测距模块。
        //20240303场外测试时，初始测距<0.01，导致这里结束，但电机没有关，只是不再增力了。修改低阈值为0.005
        //20241212 测距距离可以是0
        if(initial_tof_oflow_info.fixed_tof_distance > 0.6 )
        {
            sprintf(dbg, "bad init tof value, %4.2f, %d\n", initial_tof_oflow_info.fixed_tof_distance, sum_strengh);
            logmessage(dbg);
            finish=true;
            _inair=false;
            _running_task_status=_macro_task_failed;
            return;
        }
        #endif

        
        //#ifdef USE_BMP390
        critical_section_enter_blocking(&press_queue_section);
        initial_press_info = press_queue.history_mean(0,5);
        critical_section_exit(&press_queue_section);
        //#endif


        micro_task.taskid=_micro_task_id_nop; //相当于关闭微任务。
        micro_task.status=_micro_task_status_none;
        //上次飞行可能留下锚点未清除。
        micro_task.gps_anchor=false;
        micro_task.oflow_anchor_x=false;
        micro_task.oflow_anchor_y=false;
        micro_task.roll=0;
        micro_task.pitch=0;
        micro_task.yawspeed=0;
        //micro_task.vspeed=0; //起飞速度
        micro_task.yaw=0; //起飞前yaw是有reset
        micro_task.height_break_tmark=0;
        micro_task.smooth_landing=0;

        #if defined USE_TOF
        //通常6个压强不够离地。8个大气压离地20厘米
        micro_task.press= initial_press_info.press - 12.0;  //目标气压一定要对，到达这个气压就算起飞成功。如果设置错误则会误认起飞成功。
        micro_task.help_takeoff_target_press= initial_press_info.press - 12.0; //辅助起飞气压。
        micro_task.help_takeoff_target_height= 0.9; //起飞高度。
        micro_task.tof_height = 0.9; 
        micro_task.tof_dominate=true; //主用tof
        micro_task.help_takeoff=true; //辅助起飞标记。
        //micro_task.help_landing=false;
        #else 
        micro_task.press= initial_press_info.press - 12;  //目标气压一定要对，到达这个气压就算起飞成功。如果设置错误则会误认起飞成功。
        micro_task.help_takeoff_target_press= initial_press_info.press - 10; //辅助起飞气压。
        micro_task.help_takeoff_target_height= 1.0; //辅助起飞高度。
        micro_task.tof_height = 0.0; //起飞目标高度。
        micro_task.tof_dominate=false; //主用tof
        micro_task.help_takeoff=true; //辅助起飞标记。
        //micro_task.help_landing=false; 
        #endif

        #if defined USE_GPS
        micro_task.position_keep_switch=1; //修改为默认GPS定点。20230925
        #else
        micro_task.position_keep_switch=0;
        #endif
        micro_task.taskid=_micro_task_id_ground_up; //最后给出新任务代号
        micro_task.height_keep_switch=0; //tof to baro
        micro_task.gps_height_press_bias=0; //20230821+ gps高差稳定
        micro_task.gps_height_press_bias_last_update=0;
        task.fired=true;
        task.timemark=get_time_mark(); //标记命令时间。不过目前超时检查放在微任务里。
        _running_task_status=_macro_task_running; //running
        Send_remote_message("ground up fired");
        return; 
    }


    if(micro_task.status==_micro_task_status_done)
    {
        //起飞成功。任务结束。
        finish=true;
        _inair=true;
        _running_task_status=_macro_task_succeed; //finish
        Send_remote_message("ground up done");
        return;
    }
    else if( micro_task.status==_micro_task_status_fail) 
    {
        finish=true;
        _inair=false;
        _running_task_status=_macro_task_failed; //failed
            //micro_task.taskid=_micro_task_id_nop; //不会回到最初状态，因为标记了finish，整个任务就结束了。
        Send_remote_message("ground up fail");
        return;
    }
    else
    {

        //记录测距高度。
        // char dbg[64];
        // if(tof_oflow_queue.last().tof_valid)
        // {
        //     sprintf(dbg,"tof=%4.3f\n", tof_oflow_queue.last().fixed_tof_distance);
        //     logmessage(dbg);
        // }
        // else
        // {
        //     logmessage("tof invalid\n");
        // }

        //中间状态，主动检查气压情况，防范测距错误，在雪地起飞时，测距常有问题。
        //这种检查只是防备tof失效，在tof没有打开时，则无必要。

        press_info pi;
        critical_section_enter_blocking(&press_queue_section);
        pi = press_queue.history_mean(0,5);
        critical_section_exit(&press_queue_section);

        if(pi.press < initial_press_info.press - 36.0)
        {
                //气压已经指示在空中了，但测距却没有指示，可能测距存在问题。
                //标记在空中，禁用测距，结束起飞任务。
                _inair=true;
                finish=true;
                _running_task_status=_macro_task_succeed; //finish
                micro_task.help_takeoff=false;
                micro_task.height_keep_switch=1; //only baro height control
                Send_remote_message("tof have trouble, use baro.");
                logmessage("tof trouble, force baro control\n");

                //记录近期测距值，有60个记录值，间隔5个一采样，记录若干数据。
                #ifdef USE_TOF
                char dbg[128];
                critical_section_enter_blocking(&tof_oflow_queue_section);
                for(size_t i=0;i<60;i+=5)
                {
                    bool v= tof_oflow_queue.last(i).tof_valid; //这里误用了oflow_valid， 20240608修正
                    if(v) {
                        sprintf(dbg, "%4.2f\n", tof_oflow_queue.last(i).fixed_tof_distance);
                        logmessage(dbg);
                    }else{
                        logmessage("invtof\n");
                    }
                }
                critical_section_exit(&tof_oflow_queue_section);
                #endif

                return;
        }
    }
    

}

//暂时执行的是本地悬停，没考虑坐标，hover任务带时间限制。
void CMacroTaskExec::macro_hover(_macro_task& task, bool& finish)
{
    

    finish=false;

    // if(!_inair && !task.fired) {
    //     finish=true;
    //     return;
    // }

    if(!task.fired) {
        
        micro_task.taskid=_micro_task_id_nop; //相当于关闭微任务。
        micro_task.status=_micro_task_status_none;

        micro_task.pitch=0;
        micro_task.roll=0;
        micro_task.vspeed=0;
        task.old_yaw=micro_task.yaw; //记录初始角度，过程中旋转，任务完成后恢复初始角度
        micro_task.yawspeed=task.want_yaw_speed; //20240324, 修正严重错误，之前赋值给了vspeed.
        task.timemark=get_time_mark();
        micro_task.taskid=_micro_task_id_keep_stable_withyaw;
        task.fired=true;
        logmessage("macro: hover\n");
        char buf[128];
        sprintf(buf, "hover, %d secs", task.limit_time);
        Send_remote_message(buf);
        return;
    }


    if((get_time_mark() - task.timemark)/1000 > task.limit_time)
    {
        finish=true;
        micro_task.yawspeed=0; //悬停结束，关闭自旋。
        micro_task.yaw=task.old_yaw; //恢复到初始角度。
        logmessage("macro: hover finish. \n");
        Send_remote_message("hover finish");
        return;
    }
 
}




//主要由微任务执行飞行到目标点。可以保持旋转以不断检查指南针方向。
//基本写完，等待详细测试。用来一键返航或自动飞行。
void CMacroTaskExec::macro_landing_coordinate(_macro_task& task,bool& finish)
{
#ifdef USE_GPS

    finish=false;

    //20240309 增加判断条件 !task.fired 不然真降落成功时会标记为失败
    //这里的判断主要是为了限制执行时在空中。而不用来判断执行是否成功。
    // if(!_inair && !task.fired) {
    //     finish=true;
    //     _running_task_status=_macro_task_failed;
    //     logmessage("macro: not in air, no landing\n");
    //     return;
    // }

    if(!task.fired)
    {
        micro_task.taskid=_micro_task_id_nop; //相当于关闭微任务。
        micro_task.status=_micro_task_status_none;
        micro_task.target_gps_latitude= task.gps_latitude;
        micro_task.target_gps_logitude= task.gps_longitude;
        micro_task.target_refer_gps_height = task.gps_height; //参考地面高度。
        //micro_task.target_distance_tolerance = 3.0; //距离容差。
        micro_task.target_distance_tolerance = task.gps_location_tolerance;

        micro_task.pitch=0;
        micro_task.roll=0;
        micro_task.yawspeed=0; //不旋转。
        micro_task.vspeed=0; //保持高度。
        micro_task.taskid=_micro_task_id_vertical_landing_location;
        //turn_head_to_coordinate(task.gps_latitude, task.gps_longitude); //初始指向移动到微任务处理。
        task.fired=true;
        _running_task_status=_macro_task_running;
        logmessage("macro: fire landing coordinate. \n");
        return; 
    }
    else
    {
        //20250314 增加一个判断 !_inair , 更安全，这里判断错了主线程就会退出，然后清理次线程的微控制。
        if(micro_task.status==_micro_task_status_shutdown && !_inair)
        {
            finish=true;
            _running_task_status=_macro_task_succeed;
            logmessage("macro: landing location is done. \n");
            return;
        }
    }

#else
    finish=true;
    return;
#endif
}

//无条件就地降落
void CMacroTaskExec::macro_landing_uncondition(_macro_task& task, bool& finish)
{

    finish=false;
    
    //20240309 增加判断条件 !task.fired
    // if(!_inair && !task.fired) {
    //     finish=true;
    //     _running_task_status=_macro_task_failed;
    //     return;
    // }


    if(!task.fired) {
        micro_task.taskid=_micro_task_id_nop; 
        micro_task.status=_micro_task_status_none;

        micro_task.vspeed=20; //2m/s的降落速度。近地有自动限速。
        micro_task.timemark=get_time_mark();
        micro_task.roll=0;  
        micro_task.pitch=0;
        micro_task.taskid=_micro_task_id_vertical_landing;
        task.fired=true;
        _running_task_status=_macro_task_running;
        logmessage("macro: fire landing. \n");
        return;
    }

    //直接依赖底层报告
    //20250314 增加一个判断 !_inair , 更安全，这里判断错了主线程就会退出，然后清理次线程的微控制。
    if(micro_task.status==_micro_task_status_shutdown && !_inair)
    {
        _inair=false;
        finish=true;
        _running_task_status=_macro_task_succeed;
        micro_task.taskid=_micro_task_id_nop;
        micro_task.status=_micro_task_status_none;
        logmessage("macro: landing is done. \n");
        return;
    }


}



//利用微任务的micro_task_pin_to_location去转移位置，支持飞行中旋转
//可以在执行这个任务的过程中扫描磁场定位方向。
//目前做了时间限制，超时强制结束任务。没做逗留，到达即完成。
void CMacroTaskExec::macro_pin_to_coordinate(_macro_task& task, bool& finish)
{
#ifdef USE_GPS

    finish=false;
    
    // if(!_inair && !task.fired) {
    //     finish=true;
    //     _running_task_status= _macro_task_failed;
    //     logmessage("not in air, no pin\n");
    //     Send_remote_message("not in air. pin fail.");
    //     return;
    // }


    if(!task.fired) {
        //设置的容差不许负数，不许小于2米。
        if(task.gps_location_tolerance<2.0) task.gps_location_tolerance=2.0;
        micro_task.taskid=_micro_task_id_nop;
        micro_task.status=_micro_task_status_none;
        micro_task.target_gps_latitude=task.gps_latitude;
        micro_task.target_gps_logitude=task.gps_longitude;
        micro_task.target_distance_tolerance= task.gps_location_tolerance; //距离容差。

        micro_task.pitch=0;
        micro_task.roll=0;
        micro_task.vspeed=0; //保持高度。
        micro_task.yawspeed=0; 

        //检查目标距离，如果大于30米，就把机头指向目标点。
        //可以使用但意思不大。
        gps_info nowp; 
        critical_section_enter_blocking(&gps_queue_section);
        nowp = gps_queue.last();
        critical_section_exit(&gps_queue_section);

        //记录起点位置。20240720+
        micro_task.start_gps_latitude=nowp.latitude;
        micro_task.start_gps_logitude=nowp.longitude;

        float distDest, angleDest;
        global_gps_dist_angle(nowp.latitude, nowp.longitude, task.gps_latitude, task.gps_longitude, distDest, angleDest);

        //turn_head_to_coordinate(task.gps_latitude, task.gps_longitude); //下沉到微任务处理
        micro_task.taskid=_micro_task_id_pin_to_location;
        _running_task_status= _macro_task_running;

        task.timemark=get_time_mark(); //任务发出时间记录
        task.fired=true;
        
        char buf[128];
        sprintf(buf, "macro: fire pin, %f, %f, pu=%6.1f, dist=%5.1f\n", task.gps_latitude, task.gps_longitude,system_power_used, distDest);
        logmessage(buf);
        sprintf(buf, "fire pin, dist=%5.1f, dir=%4.1f", distDest, angleDest);
        Send_remote_message(buf);
        return;
    }

    if(micro_task.status==_micro_task_status_inprogress)
    {
        //现在纠偏下沉到微任务,这里可以考虑监视任务的执行时间，如果超长则中断。任务暂时没有时间限制
        if(task.limit_time!=0) {
            //这是时间限制，有效标记。秒。
            float last= get_time_mark() - task.timemark;
            if(last > task.limit_time*1000)
            {
                //已经超时了。打断任务。
                logmessage("pin_to_loc timeout, force to end.\n");
                Send_remote_message("pin_to_loc timeout.");
                _running_task_status= _macro_task_failed;
                finish=true;
            }
        }
        return;
    }
    else if(get_time_mark() - task.timemark > 500 &&  micro_task.status==_micro_task_status_done)
    {
        //校验一下。
        gps_info nowp; 
        critical_section_enter_blocking(&gps_queue_section);
        nowp = gps_queue.last();
        critical_section_exit(&gps_queue_section);

        float distDest, angleDest;
        global_gps_dist_angle(nowp.latitude, nowp.longitude, task.gps_latitude, task.gps_longitude, distDest, angleDest);

        //最后掉头也下沉到微任务处理。
        if(distDest <= task.gps_location_tolerance) {
            finish=true;
          
            _running_task_status= _macro_task_succeed;
            char buf[128];
            sprintf(buf, "pin done, pu=%6.1f\n",system_power_used);
            logmessage(buf);
        }
     
        return;
    }

    
 #endif
}

//最大爬高测试，一直往上爬高，直到电量消耗达到极限返回。200kj的电量保留1/4返回
void CMacroTaskExec::macro_max_height_test(_macro_task& task, bool& finish)
{
    finish=false;
    
    if(!_inair && !task.fired) {
        finish=true;
        _running_task_status= _macro_task_failed;
        Send_remote_message("not in air. fail.");
        return;
    }


    if(!task.fired) {
        //设置的容差不许负数，不许小于2米。
        gps_info nowp; 
        critical_section_enter_blocking(&gps_queue_section);
        nowp = gps_queue.last();
        critical_section_exit(&gps_queue_section);
        //记录发起地点，将来降落也在这里。
        task.gps_height=nowp.height;
        task.gps_latitude=nowp.latitude;
        task.gps_longitude=nowp.longitude;

        micro_task.taskid=_micro_task_id_keep_stable_withyaw;
        micro_task.status=_micro_task_status_none;
        micro_task.vspeed=-80; //执行8m/s向上的速度。
        micro_task.yawspeed=0; 

        _running_task_status= _macro_task_running;

        task.timemark=get_time_mark(); //任务发出时间记录
        task.fired=true;
        
        char buf[128];
        sprintf(buf, "macro: max height test\n");
        logmessage(buf);
        Send_remote_message("max height test");
        return;
    }
    else{
        //监视电量，监视水平位移是否超标，防止风大吹跑了。
        if(system_power_used > 150000)
        {
            //记录并返回，这里假设电池满电200000焦。
            finish=true;
            _running_task_status= _macro_task_succeed;
            logmessage("max height test reach max power used.");
            gps_info nowp; 
            critical_section_enter_blocking(&gps_queue_section);
            nowp = gps_queue.last();
            critical_section_exit(&gps_queue_section);
            char log[128];
            sprintf(log, "max height=%6.2f, finish.\n", nowp.height);
            logmessage(log);
            return;
        }

        gps_info nowp; 
        critical_section_enter_blocking(&gps_queue_section);
        nowp = gps_queue.last();
        critical_section_exit(&gps_queue_section);

        float distDest, angleDest;
        global_gps_dist_angle(nowp.latitude, nowp.longitude, task.gps_latitude, task.gps_longitude, distDest, angleDest);

        if(distDest > 20.0)
        {
            finish=true;
            _running_task_status= _macro_task_failed;
            logmessage("hori dist>20.0, break test\n");

            gps_info nowp; 
            critical_section_enter_blocking(&gps_queue_section);
            nowp = gps_queue.last();
            critical_section_exit(&gps_queue_section);
            char log[128];
            sprintf(log, "max height=%6.2f, finish.\n", nowp.height);
            logmessage(log);

            return;
        }
    }
}

//测试命令可以修改为，起飞，悬停几秒，降落。
void CMacroTaskExec::macro_test(_macro_task& task,bool& finish)
{

//单臂测试代码
    // finish=false;
    
    // if(mtask.taskid==_micro_task_id_nop)
    // {
    //     mtask.taskid=_micro_task_id_single_test;
    //     mtask.timemark=get_time_mark();
    //     finish=false;
    //     return;
    // }
    // else if(mtask.taskid==_micro_task_id_single_test)
    // {
    //     if(mtask.status==_micro_task_status_done||mtask.status==_micro_task_status_fail)
    //     {
    //         finish=true;
    //         return;
    //     }
    // }

    // return;

//双臂测试代码
    // if(mtask.taskid==_micro_task_id_nop)
    // {
    //     mtask.taskid=_micro_task_id_two_motor_test;
    //     mtask.timemark=get_time_mark();
    //     finish=false;
    //     return;
    // }
    // else if(mtask.taskid==_micro_task_id_two_motor_test)
    // {
    //     if(mtask.status==_micro_task_status_done||mtask.status==_micro_task_status_fail)
    //     {
    //         finish=true;
    //         return;
    //     }
    // }

    // return;

    // //单边测试代码
    // if(mtask.taskid==_micro_task_id_nop)
    // {
    //     mtask.taskid=_micro_task_id_side_test;
    //     mtask.timemark=get_time_mark();
    //     finish=false;
    //     return;
    // }
    // else if(mtask.taskid==_micro_task_id_side_test)
    // {
    //     if(mtask.status==_micro_task_status_done||mtask.status==_micro_task_status_fail)
    //     {
    //         finish=true;
    //         return;
    //     }
    // }

    // return;    

// 起飞悬停降落代码
// 20221228上午测试了这个代码，一个是飞太高了，一个是悬停过程不明显，一个是降落太快。
// 飞太高可能是因为惯性上冲，越过了悬停位置，

    finish=false;
    if(micro_task.taskid==_micro_task_id_ground_up)
    {
        if(micro_task.status==_micro_task_status_done)
        {
            //起飞成功，改为悬停几秒。
            logmessage("macro: gndup done. keep stable.\n");
            micro_task.taskid= _micro_task_id_keep_stable_withyaw;
            micro_task.status= _micro_task_status_none;
            //micro_task.press = press_queue.last().press; //当前气压悬停
            micro_task.tof_height = 0.5;
            micro_task.vspeed=0.0;
            micro_task.yaw = initial_imu_info.angle[2];
            micro_task.roll=0; 
            micro_task.pitch=0;
            micro_task.timemark=get_time_mark(); //标记开始悬停的时间。
            _inair=true;
            return;
        }
        else if( micro_task.status==_micro_task_status_fail) 
        {
            logmessage("macro: gndup fail. quit\n");
            finish=true;
            _inair=false;
            micro_task.taskid=_micro_task_id_nop; //不会回到最初状态，因为标记了finish，整个任务就结束了。
            return;
        }
        //中间状态不干预
    }
    else if(micro_task.taskid==_micro_task_id_keep_stable_withyaw)
    {
        //主要是时间控制。
        uint32_t now= get_time_mark();
        if(now - micro_task.timemark > 60000) 
        {
            micro_task.taskid= _micro_task_id_vertical_landing;
            micro_task.vspeed= 2; //速度降落。
            micro_task.status= _micro_task_status_none;
            logmessage("macro: keepstable is over, landing. \n");
            return;
        }
    }
    else if(micro_task.taskid==_micro_task_id_vertical_landing)
    {
        if(micro_task.status==_micro_task_status_done)
        {
            logmessage("macro: landing is done. quit.\n");
            finish=true;
            _inair=false;
            micro_task.taskid=_micro_task_id_nop; //不会回到最初状态，因为标记了finish，整个任务就结束了。
            return;
        }
    }
    else if(micro_task.taskid==_micro_task_id_nop)
    {
        //最初状态。
        if(!_inair) 
        {
            //正常是不在空中的状态。发出起飞命令，如果状态不对，不会有起飞命令发出。
            micro_task.taskid=_micro_task_id_ground_up;
            micro_task.status=_micro_task_status_none;
            press_info pi = press_queue.history_mean(0,3);
            micro_task.press= pi.press - 8.0;  //目标气压一定要对，到达这个气压就算起飞成功。如果设置错误则会误认起飞成功。
            micro_task.tof_height=0.5;
            micro_task.yaw = initial_imu_info.angle[2]; //必须设置这个航向角
            micro_task.vspeed=0; //起飞速度
            char buf[128];
            sprintf(buf, "macro: groundup, press=%f, set press=%f, tof=%f\n", press_queue.last().press , micro_task.press, micro_task.tof_height);
            logmessage(buf);
            
            return;
        }
    }



//起飞/降落代码
    // finish=false;
    // if(mtask.taskid==_micro_task_id_ground_up)
    // {
    //     if(mtask.status==_micro_task_status_done)
    //     {
    //         //起飞成功，直接降落。
    //         logmessage("macro: gndup done. landing.\n");
    //         mtask.taskid= _micro_task_id_vertical_landing;
    //         mtask.status= _micro_task_status_none;
    //         mtask.vspeed=1.0; //速度， 微任务在内部有近地速度限制。
    //         //mtask.timemark=get_time_mark(); //标记开始悬停的时间。
    //         mtask.yaw=init_gi.angle[2];
    //         _inair=true;
    //         return;
    //     }
    //     else if( mtask.status==_micro_task_status_fail) 
    //     {
    //         logmessage("macro: gndup fail. quit\n");
    //         finish=true;
    //         _inair=false;
    //         mtask.taskid=_micro_task_id_nop; //不会回到最初状态，因为标记了finish，整个任务就结束了。
    //         return;
    //     }
    //     //中间状态不干预
    // }
    // else if(mtask.taskid==_micro_task_id_vertical_landing)
    // {
    //     if(mtask.status==_micro_task_status_done)
    //     {
    //         logmessage("macro: landing is done. quit.\n");
    //         finish=true;
    //         _inair=false;
    //         mtask.taskid=_micro_task_id_nop; //不会回到最初状态，因为标记了finish，整个任务就结束了。
    //         return;
    //     }
    // }
    // else if(mtask.taskid==_micro_task_id_nop)
    // {
    //     //最初状态。
    //     if(!_inair) 
    //     {
    //         //正常是不在空中的状态。发出起飞命令，如果状态不对，不会有起飞命令发出。
    //         mtask.taskid=_micro_task_id_ground_up;
    //         mtask.status=_micro_task_status_none;
    //         mtask.pitch=0;
    //         mtask.roll=0;
    //         mtask.yaw=init_gi.angle[2];
    //         mtask.press=init_gi.press-14.0;  //目标气压一定要对，到达这个气压就算起飞成功。如果设置错误则会误认起飞成功。
    //         mtask.press_height= init_gi.height+1.2;
    //         mtask.tof_height= 1.2f; //起飞成功的判断高度。优先采用这个判断起飞是否成功。
    //         char buf[64];
    //         sprintf(buf, "macro: init press=%f, set press=%f, tof=%f\n", init_gi.press, mtask.press, mtask.tof_height);
    //         logmessage(buf);
            
    //         return;
    //     }
    // }
}


// //板载测试
// void CMacroTaskExec::macro_board_test(_macro_task& task, bool& finish)
// {
//     if(micro_task.taskid==_micro_task_id_nop)
//     {
//         micro_task.taskid=_micro_task_id_board_test;
//         micro_task.status=_micro_task_status_none;
//         finish=false;
//         return;
//     }
//     else if(micro_task.status==_micro_task_status_done)
//     {
//         finish=true;
//         _inair=true; //for test, 假设在空中，遥控可操纵。
//         return;
//     }

// }

