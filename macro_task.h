#ifndef _MACRO_TASK_H
#define _MACRO_TASK_H

#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "pico/platform.h"
#include "pico/multicore.h"

#include "gps.h"
#include "icm426xx.h"
#include "bmp390_spi.h"


enum _macro_task_id
{
    _macro_none=0,      //无效任务
    _macro_hover=1,     //悬停，可以指定高度，位置，持续时间。
    _macro_landing_coordinate=5, //某个坐标点降落
    _macro_landing_uncondition=7, //无条件就地降落，电力不足时可使用。 
    _macro_ground_up=9, //一键起飞任务
    _macro_yaw_judge= 11,  //判断航向角, 不再使用
    _macro_test=13, //测试任务
    //_macro_board_test=15, //板载测试任务
    _macro_vertical_height_adjust=15, //垂直相对高度调节任务
    _macro_vertical_absolute_height_adjust=16, //绝对高度调节，以GPS高度为准
    _macro_on_remote=17, //听从遥控指令。
    _macro_compass_check=19, //原地定高悬停，旋转360度，建立指南针的数据，找到正北。不再使用
    _macro_energy_guard=21, //20241008, 实际不执行，只是为了顺序关系安排一个保卫任务。
    _macro_voltage_guard=22, //20241008, 实际不执行，只是为了顺序关系安排一个保卫任务。
    _macro_make_compass_map=23, //通过旋转机身360度，建立指南针数据图谱，找到北方。不再使用
    _macro_pin_to_coordinate=25, //飞到某个坐标点, 或保持位置稳定在某个点。
    _macro_fly_ahead_and_back=26, //前飞一段距离，然后返回。目前没有启用。
    _macro_point_to_coordinate=27, //机头指向目标点。
    _macro_max_height_test=28, //最大爬高测试。
};

enum _macro_task_running_status
{
    _macro_task_running=0, //执行中
    _macro_task_succeed=1, //成功
    _macro_task_failed=2,  //失败
};

class _macro_task
{
    public:
    _macro_task() {
        taskid=_macro_none;
        press=0;
        press_high=0;
        press_low=0;
        gps_height=0;
        gps_longitude=0;
        gps_latitude=0;
        gps_location_tolerance=2.0;
        vertical_adjust_height=0;
        target_gps_height=0;
        want_yaw_speed=0;
        limit_time=0;
        timemark=0;
        guard_energy_thres=0;
        guard_voltage_thres=0;
        vertical_climb=false;
        old_yaw=0;
        fired=false;
    }

    _macro_task& operator=(const _macro_task& o)
    {
        taskid=o.taskid;
        press=o.press;
        press_high=o.press_high;
        press_low=o.press_low;
        gps_height=o.gps_height;
        gps_latitude=o.gps_latitude;
        gps_longitude=o.gps_longitude;
        gps_location_tolerance=o.gps_location_tolerance;
        limit_time=o.limit_time;
        vertical_adjust_height=o.vertical_adjust_height;
        target_gps_height=o.target_gps_height;
        want_yaw_speed=o.want_yaw_speed;
        timemark=o.timemark;
        fired=o.fired;
        vertical_climb=o.vertical_climb; //20250312+
        old_yaw=o.old_yaw; //20250312+
        guard_energy_thres=o.guard_energy_thres;
        guard_voltage_thres=o.guard_voltage_thres;
        return *this;
    }  

    _macro_task_id taskid;
    //飞行高度限制
    float press; //大气压， 如果给出，优先选择这个控制高度
    float press_high; //高压范围，目标气压容差范围，在这个范围内可以用GPS数据
    float press_low; //低压范围，两个范围用于高度调节控制,20240618+
    float vertical_adjust_height; //相对垂直高度调节参数。_macro_vertical_climb的任务参数
    float target_gps_height; //调节绝对高度使用。
    int8_t   want_yaw_speed; //宏任务希望机身的旋转速度。主要是悬停时使用，也可以是高度调节时使用。
    float gps_height; //如果没有气压高度，则选这个高度控制
    double gps_longitude;
    double gps_latitude;
    float gps_location_tolerance; //距离容差，>0，小于这个值代表水平定位已到达。
    uint16_t limit_time; //任务的时间限制，超时则退出，秒。
    uint32_t timemark;  //记录任务开始时间
    bool fired; //命令初始发出标记
    bool vertical_climb; //垂直方向的标记，爬升=true,降低=false; 在垂直运动启动时必须标记这个。这是避免发生气压高度和GPS高度对不准而产生的震荡。
    //std::string taskname; //任务名称，20231121+
    float old_yaw; //有些任务会带有旋转，在任务开始时记录初始角度，任务结束后恢复初始角度。
    float guard_energy_thres; //警卫任务能量阈值 20241008+
    float guard_voltage_thres; //警卫任务电压阈值 20241008+
};



#endif