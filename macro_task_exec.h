
#ifndef _MACRO_TASK_EXEC_H
#define _MACRO_TASK_EXEC_H

#include "macro_task.h"
#include "power.h"
#include "skydogge.h"
#include "randqueue.h"
#include "tools.h"
//#include <queue>
//#include <tuple>

class _yaw_map{
    public:
    _yaw_map() {
        yaw=0;
        dir=0;
        speed=0;
        tmark=0;
    }
    float yaw; //内部传感器方位角， 0-180， -180-0
    float dir; //gps航向角，0-360
    float speed; //速度。
    uint32_t tmark; //构造时间。
};

class CMacroTaskExec
{
    public:
        CMacroTaskExec(): _task_num(0) {

        }

        void do_macro_task(_macro_task& task, bool& finish);
        void macro_hover(_macro_task& task, bool& finish);
        void macro_landing_coordinate(_macro_task& task, bool& finish);
        void macro_landing_uncondition(_macro_task& task, bool& finish); 
        void macro_pin_to_coordinate(_macro_task& task, bool& finish); //移动到坐标点。
        void macro_ground_up(_macro_task& task, bool& finish); //一键起飞
        void macro_make_compass_map(_macro_task& task, bool& finish);    //构造指南针数据图谱
        void macro_point_to_coordinate(_macro_task& task, bool& finish); //机头指向坐标点。
        void macro_vertical_relative_height_adjust(_macro_task& task, bool& finish); //相对高度调节
        void macro_vertical_absolute_height_adjust(_macro_task& task, bool& finish); //绝对高度调节，以GPS高度为准。
        void macro_max_height_test(_macro_task& task, bool& finish); //最大爬高测试。
        bool UpdateHeadingYaw();
        //机头指向目标点方向。
        void turn_head_to_coordinate(double lati, double logi);

        //测试
        void macro_test(_macro_task& task, bool& finish);
        //void macro_board_test(_macro_task& task,bool& finish);
        //void macro_time_test(_macro_task& task,bool& finish);

        size_t task_num() {
            return _task_num;
        }

        bool empty_task() {
            return _task_num==0;
        }
        
        void do_front_task(bool& finish);
        

        void addtask_ground_up() {
            _macro_task tk; 
            //tk.taskname="takeoff";
            tk.taskid=_macro_ground_up;
            _tasks[_task_num++]=tk;
           
        }

        //定点调节高度，水平定位采用pin_to_location
        //做一个自动爬升任务主要是方便遥控一键达到高度。
        //在其他自动任务里，这个应该不太使用。meter>0 向上，meter<0 向下
        void addtask_vertical_height_adjust(float meter, int8_t yawspd=0) {
            _macro_task tk;
            tk.taskid=_macro_vertical_height_adjust;
            tk.vertical_adjust_height=meter;
            tk.want_yaw_speed=yawspd;
            _tasks[_task_num++]=tk;
        }

        //绝对高度调节，以GPS高度为基准
        //仅接受一个>0的高度。
        void addtask_vertical_absolute_height_adjust(float gps_h, int8_t yawspd=0) {
            if(gps_h>0) {
                _macro_task tk;
                tk.taskid=_macro_vertical_absolute_height_adjust;
                tk.target_gps_height=gps_h;
                tk.want_yaw_speed=yawspd;
                _tasks[_task_num++]=tk;
            }
        }

        void addtask_landing() {
            _macro_task tk;
            //tk.taskname="landing";
            tk.taskid=_macro_landing_uncondition;
            _tasks[_task_num++]=tk;
        }

        void addtask_keepstable(float timeout, int8_t yawspd=0) {
            _macro_task tk;
            tk.taskid=_macro_hover;
            tk.limit_time = timeout;
            tk.want_yaw_speed=yawspd;
            _tasks[_task_num++]=tk;
        }

        void addtask_makecompassmap() {
            _macro_task tk;
            tk.taskid=_macro_make_compass_map;
            _tasks[_task_num++]=tk;
        }


        void addtask_maxheighttest() {
            _macro_task tk;
            tk.taskid=_macro_max_height_test;
            _tasks[_task_num++]=tk;
        }

        //limit_time=0无效，抵达目标的时间限制（秒），如果时间到了目标没有到达，则执行下一个任务。
        //linger_time=0无效，逗留时间（秒），从到达后开始计算。
        void addtask_pin_to_corrdinate(double logi, double lati, float tolerance=3.0, uint32_t limit_time=0) {
            _macro_task tk;
            
            tk.taskid=_macro_pin_to_coordinate;
            tk.gps_longitude=logi;
            tk.gps_latitude=lati;
            tk.gps_location_tolerance=tolerance;

            if(tk.gps_location_tolerance<0) tk.gps_location_tolerance=2.5;
            else if(tk.gps_location_tolerance>30) tk.gps_location_tolerance=30;

            tk.limit_time=limit_time;
            
            _tasks[_task_num++]=tk;
        }


        void addtask_landing_to_corrdinate(double logi, double lati, float height, float tolerance=1.0) {
            _macro_task tk;
            tk.taskid=_macro_landing_coordinate;
            tk.gps_longitude=logi;
            tk.gps_latitude=lati;
            tk.gps_height=height; //参考地面高度
            tk.gps_location_tolerance=tolerance;

            if(tk.gps_location_tolerance<1.0) tk.gps_location_tolerance=1.0;
            else if(tk.gps_location_tolerance>3.0) tk.gps_location_tolerance=3.0;

            _tasks[_task_num++]=tk;
        }

        //宏命令统一接口，任务串只接受L()降落，A()绝对高度调节，H()相对高度调节，K()保持位置，G()水平移动
        //R,RH,LH等命令已经被上一层翻译了。R，RH翻译成G, LH翻译成L
        //接口的好处是带了任务名称，方便理解当前的任务。
        void add_task(std::string name, std::string taskstr)
        {

        }

        void add_enegy_guard(float thres)
        {
            _macro_task tk;
            tk.taskid=_macro_energy_guard;
            tk.guard_energy_thres=thres;
            _tasks[_task_num++]=tk;
        }

        void add_voltage_guard(float thres)
        {
            _macro_task tk;
            tk.taskid=_macro_voltage_guard;
            tk.guard_voltage_thres=thres;
            _tasks[_task_num++]=tk;
        }

        void clear_task() {
            _task_num=0;
        }
    private:

        //动态分配内存，可疑。
        size_t _task_num;
        _macro_task _tasks[30]; //30个任务槽应该够用。
        
        public:
        ////20240307+ 做两个新标记，以便从自动任务退出时，是由遥控接手还是不接手。
        _macro_task_id _running_task_id; //正运行的任务id. //20240307+
        _macro_task_running_status _running_task_status; //0=running, 1=finish, 2=failed. //20240307+



};

#endif
