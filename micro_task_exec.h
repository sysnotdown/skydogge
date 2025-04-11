
#ifndef _MICRO_TASK_EXEC_H
#define _MICRO_TASK_EXEC_H
#include <list>
#include <queue>
#include <vector>
#include <fstream>


#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "pico/platform.h"
#include "pico/multicore.h"

#include "icm426xx.h"
#include "micro_task.h"
#include "motor.h"
//#include "tof400.h"
#include "randqueue.h"
#include "skydogge.h"
#include "tools.h"

class press_info;

class mortor_sum_burden
{
public:
    mortor_sum_burden() {
        tmark=0;
        sum=0;
    }
    mortor_sum_burden(const float v) {
        tmark=0;
        sum=v;
    }

    mortor_sum_burden& 
    operator+=(const mortor_sum_burden &o)
    {
        sum+=o.sum;
        return *this;
    }  
      
    mortor_sum_burden
    operator+(const mortor_sum_burden &rhs)
    {
        mortor_sum_burden o;
        o = *this;
        o += rhs;
        return o;
    }

    mortor_sum_burden& operator=(const mortor_sum_burden& o)
    {
        tmark=o.tmark;
        sum=o.sum;
        return *this;
    } 

    uint32_t tmark;
    float sum;

};

class tof_press_map{
    public:
        tof_press_map() {
            _tmark=0;
            _press=0;
            _tof=0;
        }

        void setdata(float press, float tof, uint32_t tm) {
            _press= press; _tof=tof; _tmark=tm;
        }


        float topress(float tof) {
            return _press - (tof - _tof)*12.0;
        }

        float totof(float press) {
            if(_tmark!=0)
                return _tof - (press - _press)/12.0;
            else return -1;
        }

        uint32_t gettm() {
            return _tmark;
        }

protected:
        float _press;
        float _tof;
        uint32_t _tmark;
};

//20240128，新增exec_num, exec_sum_thrust以记录平衡函数对于垂直推力的调节借用。
class height_adjust_record
{
    public:
    uint32_t tmark;
    float thrust; //推力的垂直分量，balance负责解析。由于balance可以在一定范围内调节，所以控高需要参考具体的执行推力去给出新的推力。
    float hd;
    float spd; //20230803+ 新增速度记录，在下降到tof距离时可以参考这个速度来调节。
    float current; //20240420 新增电流记录。
    height_adjust_record() {
        tmark=0;
        thrust=0;
        hd=0;
        spd=0;
        current=0;
    }

    height_adjust_record& operator=(const height_adjust_record& o)
    {
        tmark=o.tmark;
        thrust=o.thrust;
        hd=o.hd;
        spd=o.spd;
        current=o.current;
        return *this;
    }
};

//oflow/gps定位调节导致的偏离角
class oflow_bias {
public:
    oflow_bias() {
        roll_bias=0;
        pitch_bias=0;
        yaw=0;
        virtual_castx=0;
        virtual_casty=0;
        want_virtual_accx=0;
        want_virtual_accy=0;
        offset_sumx=0;
        offset_sumy=0;
        tmark=0;
    }

    uint32_t tmark;
    float roll_bias;
    float pitch_bias;
    float yaw;
    float offset_sumx;
    float offset_sumy;
    //世界坐标下的虚拟调节倾角。
    float virtual_castx;
    float virtual_casty;
    //期待在下一个信号到来前，有多少加速度积分量（即速度变化量），一个信号一个量，随后交给微调去跟踪处理。
    float want_virtual_accx;
    float want_virtual_accy;

    oflow_bias& operator=(const oflow_bias& o)
    {
        roll_bias=o.roll_bias;
        pitch_bias=o.pitch_bias;
        virtual_castx=o.virtual_castx;
        virtual_casty=o.virtual_casty;
        yaw=o.yaw;
        offset_sumx=o.offset_sumx;
        offset_sumy=o.offset_sumy;
        tmark=o.tmark;
        return *this;
    } 

};


//gps定点粗调记录
class gps_major_bias {
public:
    gps_major_bias() {
        clear();
    }

    void clear() {
        world_castx=0;
        world_casty=0;
        gps_dist=0;
        gps_distEast=0;
        gps_distNorth=0;
        gps_spdEast=0;
        gps_spdNorth=0;
        gps_spdDir=0;
        gps_spd=0;
        roll_bias=0;
        pitch_bias=0;
        flight_real_x=0;
        flight_real_y=0;
        gps_tmark=0;
        current=0;
        world_real_x=0;
        world_real_y=0;
        inuse=0; 
    }

    uint32_t gps_tmark;
    //世界坐标下的虚拟调节点投影。
    float world_castx;
    float world_casty;
    //gps数据。
    float gps_dist;
    float gps_distEast;
    float gps_distNorth;
    float gps_spdEast;
    float gps_spdNorth;
    float gps_spdDir;
    float gps_spd;
    float roll_bias;
    float pitch_bias;
    float flight_real_x; //20240706+ 记录最新机身侧倾推力分量，真实值。
    float flight_real_y;
    float world_real_x; //20240728+ 记录机身真实推力转世界方向的推力。东向。
    float world_real_y; //北向。
    float current;  //电流
    uint8_t inuse; //谁在使用它并记录它，未使用0，定点1，刹车2，平飞3, 20240713+

    gps_major_bias& operator=(const gps_major_bias& o)
    {
        world_castx=o.world_castx;
        world_casty=o.world_casty;
        gps_dist=o.gps_dist;
        gps_distEast=o.gps_distEast;
        gps_distNorth=o.gps_distNorth;
        gps_spdEast=o.gps_spdEast;
        gps_spdNorth=o.gps_spdNorth;
        gps_spdDir=o.gps_spdDir;
        gps_spd=o.gps_spd;
        roll_bias=o.roll_bias;
        pitch_bias=o.pitch_bias;
        flight_real_x=o.flight_real_x;
        flight_real_y=o.flight_real_y;
        gps_tmark=o.gps_tmark;
        current=o.current;
        world_real_x= o.world_real_x;
        world_real_y= o.world_real_y;
        inuse=o.inuse;
        return *this;
    }

};



//imu定点保持记录结构
class imu_bias
{
public:
    imu_bias() {
        tmark=0;
        pitch_bias=0;
        roll_bias=0;
        sum_accx=0;
        sum_accy=0;
    }

    imu_bias& operator=(const imu_bias& o)
    {
        roll_bias=o.roll_bias;
        pitch_bias=o.pitch_bias;
        tmark=o.tmark;
        sum_accx=o.sum_accx;
        sum_accy=o.sum_accy;
        return *this;
    } 

    uint32_t tmark;
    float pitch_bias;
    float roll_bias;
    float sum_accx; //累积的加速度总和
    float sum_accy; //累积的加速度总和
};

//用于坐标转换的辅助结构，避免过多参数传递。
class coordinate_trans_struct
{
    public:
        float yawA;
        float yawB;
        float xA;
        float yA;
        float rollA;
        float pitchA;
        float xB;
        float yB;
        float rollB;
        float pitchB;
};

//heading yaw的调节历史记录 20230817
//这个结构在gps定点中用来记录调节历史
class heading_yaw_adjust_record
{
    public:
        heading_yaw_adjust_record() {
            dtime=0;
            spd_angle=0;
            spd=0;
        }

        uint32_t dtime;
        float spd_angle;
        float spd;
};

//当前假设X轴的正向是机头方向。Y轴正向是机身左侧方向。
class CMotor;
class CMicroTaskExec
{

public:

    void do_micro_task();
    void micro_task_ground_up();
    
    void micro_task_vertical_landing();
    void micro_task_vertical_landing_target_location();
    void micro_task_keep_stable_with_yaw();
    void micro_task_move_ahead_with_yaw();
    void micro_task_move_around();
    void micro_task_idle(); //20240321+
    void get_refer_burdens_for_balance(float burdens[4]);
    float get_refer_thrust_for_height_control();

    //float get_refer_imu_vacc_for_height_control();
    float get_refer_imu_vacc_for_height_control(float& zaxis, float& vib_z);
#if defined USE_TOF
    bool height_adjust_by_tof_with_landing_detect(float botlmt, float toplmt, bool& landed); //20240226+ 返回落地标记。
#endif
    bool hyper_height_adjust_by_baro_with_landing_detect(float botlmt, float toplmt, bool& landed);
    bool height_adjust_by_baro_with_landing_detect(float botlmt, float toplmt, bool& landed); //20240226+ 同老版，不返回落地标记
    bool height_adjust_by_baro_with_landing_detect_stable(float botlmt, float toplmt, bool& landed);
    bool height_adjust_with_landing_detect(float botlmt, float toplmt, bool& landed); //20240226+

    void micro_task_on_remote();
    
    void balance(float new_burdens[4], float& new_thrust, float pitch, float roll);

    int switch_fixpoint_mod();

    void micro_task_pin_to_location(); //gps 定点微任务实现
    bool pin_to_location(float& cur_dist, float& cur_spd, float& cur_dir,  float& cur_height, bool& newdata); //定点调节函数，不是微任务
 
    
    //给出适当的roll,pitch去调节机身位置到固定点。
    //两个定点方式可以组合到一起。
    bool oflow_position_adjust_minor_change(); //还行，从oflow_position_adjust()微调以适应旋转。

    //坐标数据由yawA系统转到yawB系统的变换
    void coordinate_trans(coordinate_trans_struct& trans);

    bool gps_position_adjust();
    bool gps_position_brake();

    float SigmoidTrans(float a);
    bool adjust_burdens(float tt, float burdens[4], float& thrust_adj_ratio);
    float GpsQuality(uint8_t ns, float hacc);
    void Flight2World(float cur_yaw, float fx, float fy, float& wx, float& wy);//20240113+
    void World2Flight(float cur_yaw, float wx, float wy, float& fx, float& fy);//20240113+
    void Flight2World(float cosheading, float sinheading, float fx, float fy, float& wx, float& wy); //20240113+
    void World2Flight(float cosheading, float sinheading, float wx, float wy, float& fx, float& fy); //20240113+
    int auto_landing_vertical_speed(float height, float dist); //20240217+
    float gps_speed_reliable(float gps_spd); //20240217+
    float gps_dist_reliable(float gps_dist, float hacc); //20240218+
    float gps_brake_limit_acc(float spd); //20240313+
    float get_proper_hspd(float dist);
    void get_spd_acc_lmt_by_dist(float dist, float spd_angle, float& spdlmt, float& acclmt);
    float acc_match_adjust_strengh(float accnorm); //acc匹配强度系数
    void rotate_ccw(float x, float y, float angle, float& nx, float& ny); //将坐标系逆时针旋转angle角度（度数）得到新的坐标位置。20240721+
    float get_proper_hspd(float dist, float gps_quality);
    float PowerCostForPin(float cur_spd, float current, float current_grow, int psolution, float ref_current, float& ref_mod_limit);
    float PowerCostForClimbing(int psolution, float ref_current, float ref_vspd);
    float calculate_baro_vertical_speed_50hz_n5();
};

#endif