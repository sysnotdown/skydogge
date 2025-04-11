/**
 * skydoggy v0.8
 *
 * 20230216
 **/

//20230419测试基本满意，
//现在的基本问题是：
//1.回传飞行数据，遥控有屏幕可以显示数据了。
//主要是GPS, 位置，高度，时间，精度，方向角度。
//2.穿透自动任务的遥控。即自动接管。

//3.机身方向自主判断。目前没有指南针，需要用到GPS.
//考虑一个独立的模块，接收所有imu数据和gps数据，结合起来分析。
//在imu发生变化时，可以计算到方向，甚至是风的大小和角度


#include "skydogge.h"
#include <random>
#include <map>
#include <stack>
#include "tools.h"
#include "tusb.h" //识别usb是否链接

extern std::default_random_engine rgen;
std::normal_distribution<float> motors0(0,0.1);
std::normal_distribution<float> test_jhemcu(0.25, 0.998);
std::uniform_real_distribution<float> random_burs1(0.4, 0.998);
std::uniform_real_distribution<float> random_burs2(0.6, 0.998);
std::uniform_real_distribution<float> random_burs3(0.7, 0.998);
using namespace std;



#ifdef USE_GPS
//CGPS uart_gps;
CGPS_Ublox uart_gps;
#endif

#ifdef USE_QMC5883
CQMC5883 i2c_cpass_qmc5883;
#endif

#ifdef USE_HMC5883
CHMC5883 i2c_cpass_hmc5883;
#endif

#ifdef USE_REMOTE_CTRL
#if defined USE_DUAL_LORA
CDual_Lora remote_ctrl;
#endif
#endif



#ifdef USE_TOF
//CMTF01 tof_oflow;
CTofSenseF tof_oflow;
#endif

#ifdef USE_ICM426XX
CICM426XX icm426xx;
#endif

#ifdef USE_BMP390
CBMP390_Spi bmp390;
#endif


CMotor motor_ctrl;
CMicroTaskExec micro_exec;
CMacroTaskExec macro_exec;

//__in_flash() 用于相反的目的，只放在flash里。

__not_in_flash("data") _micro_task micro_task;


//日志，最后写到flash. 
//20230201 晚上写日志失败，
//20240102 修改日志记录的大小，原先是32扇区，修改为16扇区，以避免内存不够用的可能性
//随着系统越来越复杂，占用空间越来越大，日志记录占用太多可能导致意外问题。
#define LOGSIZE (16*FLASH_SECTOR_SIZE)
//日志区间前推1个扇区位置写入配置参数。
#define CALIBDATA (17*FLASH_SECTOR_SIZE)

//critical_section_t log_section; //20240906+ 之前没有这个，但两个线程都存在Log_msg调用，所以理论上需要这个。
mutex_t log_mutex; //20250313 修改，mutex不阻断中断，应该更合适。
NRandQueue_Origin<char> log_msg(LOGSIZE);

//中断接收imu数据，然后发出执行微控制的动作信号，微控制不在中断内执行。
__not_in_flash("data")  semaphore_t imu_data_to_action;


#ifdef USE_REMOTE_CTRL
//遥控命令存放
__not_in_flash("data") remote_info last_remote_info; //收到的最后一个遥控信息。
__not_in_flash("data") uint8_t last_predefined_task; 
//__not_in_flash("data") bool remote_predefined_task_signal=false; //上一个遥控信号是预定义任务。
//这里可能有动态分配问题
__not_in_flash("data") std::string last_task_content; //遥控发送的任务安排信息。
__not_in_flash("data") bool remote_task_signal=false; //上一个遥控信号是任务。
//20231129 把这个也不放在flash
__not_in_flash("data") semaphore_t remote_info_to_action; //信号量，代表接收到新的遥控信息。
#endif

//发送给遥控器的记录数据。一类是GPS,一类是其他。
#if defined USE_REMOTE_CTRL
uint32_t last_gps_sent=0;
uint32_t last_other_sent=0;
#endif



#if defined USE_GPS
//gps_info大约32字节，200个数据消耗6.4kb内存
//也许可以考虑不放在运行内存里
//__not_in_flash("data") NRandQueue<gps_info> gps_queue(50);  //5秒数据记录够了。
__not_in_flash("data") NRandQueue<gps_info> gps_queue(15);  //20240709 压缩内存使用，15个够了。
__not_in_flash("data") critical_section_t gps_queue_section; //或许可以用recursive_mutex_t代替，不能用mutex_t
//__not_in_flash("data") mutex_t gps_queue_mutex; //20250314 修改，mutex不阻断中断，应该更合适。
//semaphore_t gps_data_to_action; //没有哪个动作在等待GPS
#endif


#if defined USE_TOF || defined USE_OFLOW
//tof不大于8米，100hz
//目前光流控制最远涉及40个，降落控制最远涉及20个。控高涉及6个。
__not_in_flash("data") NRandQueue<tof_oflow_info> tof_oflow_queue(40);
__not_in_flash("data") critical_section_t tof_oflow_queue_section; //或许可以用recursive_mutex_t代替，不能用mutex_t
//__not_in_flash("data") mutex_t tof_oflow_queue_mutex;
#endif

float total_horizon_move=0; //水平移动总距离。

class arguement_name
{
public:
	arguement_name() {
		name[0] = 0;
		name[1] = 0;
		name[2] = 0;
		name[3] = 0;
	}
	arguement_name(const char n[4])
	{
		name[0] = n[0];
		name[1] = n[1];
		name[2] = n[2];
		name[3] = n[3];
	}
	//名称是否合法，大小写字母+数字等。
	static bool isLeagleLetter(const char c) {
		if (c >= 32 && c <= 57) return true;
		if (c >= 65 && c <= 90) return true;
		if (c >= 97 && c <= 122) return true;
		return false;
	}
	static bool isLeagleName(const char n[4]) {
		return isLeagleLetter(n[0]) && isLeagleLetter(n[1]) && isLeagleLetter(n[2]) && isLeagleLetter(n[3]);
	};

	char name[4];
	void operator=(std::string ss) {
		if (ss.size() >= 4) {
			name[0] = ss[0];
			name[1] = ss[1];
			name[2] = ss[2];
			name[3] = ss[3];
		}
		else if (ss.size() == 3) {
			name[0] = ' ';
			name[1] = ss[0];
			name[2] = ss[1];
			name[3] = ss[2];
		}
		else if (ss.size() == 2) {
			name[0] = ' ';
			name[1] = ' ';
			name[2] = ss[0];
			name[3] = ss[1];
		}
		else if (ss.size() == 1) {
			name[0] = ' ';
			name[1] = ' ';
			name[2] = ' ';
			name[3] = ss[0];
		}
		else if (ss.size() == 0) {
			name[0] = ' ';
			name[1] = ' ';
			name[2] = ' ';
			name[3] = ' ';
		}
	}

	void operator=(const char n[4]) {
		name[0] = n[0];
		name[1] = n[1];
		name[2] = n[2];
		name[3] = n[3];
	}

	void operator=(const arguement_name& o) {
		name[0] = o.name[0];
		name[1] = o.name[1];
		name[2] = o.name[2];
		name[3] = o.name[3];
	}

	bool operator==(const arguement_name& o) const
	{
		return name[0] == o.name[0] && name[1] == o.name[1] && name[2] == o.name[2] && name[3] == o.name[3];
	}

	bool operator<(const arguement_name& o) const
	{
		if (name[0] < o.name[0]) return true;
		if (name[0] > o.name[0]) return false;
		if (name[1] < o.name[1]) return true;
		if (name[1] > o.name[1]) return false;
		if (name[2] < o.name[2]) return true;
		if (name[2] > o.name[2]) return false;
		if (name[3] < o.name[3]) return true;
		return false;
	}
};


std::map<arguement_name, float> adjustable_running_arguements;


#if defined USE_ICM426XX
//imu序列
//计算垂直加速度均值涉及100个数据。求参考推力涉及0.3秒数据，150个。
//早期的平衡代码里设计角度积分约200个数据，新代码里取消了这个积分，所以用新平衡代码则 150个数据记录就够了。
//oflow控制引用近0.3秒，所以调大这个数据到150个。
//uint32_t imu_data_counter; //总计数，用来间隔采样Yaw
//20230712+ 为了分析指南针的准度，这里放开存储到300个，0.6秒的时间跨度。
//imu_info大约占64字节，过量存储会严重浪费系统内存。目前引用最远的是initial_direction_guess()，用来猜测方向。
//可能需要考虑压缩存储空间。
//__not_in_flash("data") NRandQueue<imu_info> imu_queue(300); //采样500hz下只能存放0.6秒数据。
__not_in_flash("data") NRandQueue<imu_info> imu_queue(120); //20240708 压缩内存使用

//NRandQueue<Yaw_info> yaw_queue(100);    //间隔采样的yaw，用来观察陀螺仪是否需要校准，这个只在地面有效。空中无法观察yaw角漂移。

__not_in_flash("data") critical_section_t imu_queue_section;
#endif

#if defined USE_BMP390
//大气压序列，主要在气压控高使用，里面涉及到20个数据，存储25个够用。
//20231130,增加记录到155个，以便气压降落时判断准确
//要检查这里是否需要存这么多数据。
//__not_in_flash("data") NRandQueue<press_info> press_queue(155);
__not_in_flash("data") NRandQueue<press_info> press_queue(50); //20240708 压缩内存使用
__not_in_flash("data") critical_section_t press_queue_section; 
#endif

//出发点信息
#ifdef USE_ICM426XX
imu_info initial_imu_info;
#endif
#ifdef USE_GPS
gps_info initial_gps_info;
#endif

#ifdef USE_BMP390
press_info initial_press_info; 
#endif

#ifdef USE_TOF
//tof_info initial_tof_info; //初始机架高度。
tof_oflow_info initial_tof_oflow_info; //初始tof信息
#endif

//最后降落地的信息，每次降落后或起飞前记录。
#ifdef USE_GPS
gps_info last_ground_gps_info;
#endif

#ifdef USE_BMP390
press_info last_ground_press_info; 
#endif

float system_power_voltage=15.0;
float system_power_current=0;

__not_in_flash("data") NRandQueue<float> current_queue(32); //20毫秒一个记录，0.6秒
//__not_in_flash("data") recursive_mutex_t current_queue_mutex; //20250315+
//通过积分计算系统的能量消耗，焦耳。4节21700理论能量26万瓦，实际悬停到10.0v电压得到的积分能量只有16万瓦。
//这和电流表的校准有很大关系。电流表偏差较大就不会准。
//20231204主测试机飞丢后，启用备用机，同样的电调，这个电调的电流表比老的大了20%，似乎更准确些，这样
//4节21700的总放电量积分出来应该大20%，是19.2万瓦。
//每个电调的电流表可能都不一致，跑一次悬停测试可以测试出总放电积分，得到新的总电量值，即可重新校准耗电量估算。
//远东21700 6000mah确实容量很大，放电达到270kj, 常规的5000mah电池放电在220kj左右。
//由此，第一次用远东21700 6000mah电池悬停测试，达到破记录的64分钟，放电到10.0v. 放到11v也达到61分钟。
float system_power_used=0.0;
uint32_t system_power_last_check_time=0;

//20231115+ 机器控制权是否在遥控？在自动任务里是false;
bool on_remote=true;


//安装角度偏差，物理调节
float install_bias_angle0= 0; //IBA0
float install_bias_angle1= 0; //IBA1


//由于安装偏差导致的静态下的地面坐标系的净加速度偏差。
float install_bias_gnd_acc0=0; //BGA0
float install_bias_gnd_acc1=0; //BGA1
float install_bias_gnd_acc2=0; //BGA2

float imu_gyro_raw_bias0= 0; //GRB0
float imu_gyro_raw_bias1= 0; //GRB1
float imu_gyro_raw_bias2= 0; //GRB2
float imu_acce_raw_ratio= 1.0; //静态下加速度计模量。 IARR

//陀螺仪温度补偿系数，在静止下校准imu，观察温度和yaw角的变化，
//如果用热风枪对着吹，让温度升高，正常升温导致yaw负漂移，如果调节准确则没有漂移，调节过大则变成正漂移。
float imu_gyroz_temp_comp=0.005; //z轴陀螺仪温度补偿参数。0.005对42605来说是个比较好的设置，可能每片都不同。

float imu_calib_temprature; //校准时的平均温度。ICTP


//磁力计校准中心点。
float magnet_midx=0; //MAGX
float magnet_midy=0; //MAGY
float magnet_midz=0; //MAGZ
bool  update_magnet=false; //磁场被更新标记，有更新就在任务结束后写入记录。

//几个平衡参数都必须>=0
//总体还好，微抖，估计要降BALD.
// BALD:0.700000
// BALI:0.020000
// BALM:0.800000
// BALP:0.200000
// BALW:0.100000
// BALX:0.050000
// BALZ:0.020000

//2806电机由于质量分布在外围，d值要增加，不然晃动很大，增加为0.9，主参数也更大，1.1
float balance_main_ratio=0.9; //平衡函数主参数，现在放入可调节参数里。BALM
float balance_p_ratio=0.4; //BALP 角度偏差调节项
float balance_i_ratio=0.02; //BALI  角度偏差积分项
//角速度阻尼是影响平衡稳定性的最重要参数，太小会大幅摇摆最后翻倒，太大则灵活性很差，且机身会共振。
//软机臂易共振，不宜调太大，追求稳定要调大，同时增加机身刚性，如果刚性不够则需要折中取舍。
//既要低刚性机身又要高稳定性是矛盾的。远航机追求轻量化，机身刚性不如穿越机，就不能调节太大。
//角加速度阻尼也非常重要，因为代码切换失误将角加速度降低到之前的十分之一的水平时，其他参数非常难调整到一个合适的值。
float balance_d_ratio=0.70; //BALD  角速度阻尼项 降低到0.2摇摆侧翻, 0.6以上机身易抖动。0.5不抖。
float balance_x_ratio=0.50; //BALX 角加速度阻尼项
float balance_w_ratio=0.12; //BALW 角速度调节项
float balance_z_ratio=0.01; //BALZ 角加速度调节项

//后期修正参数，数据转换后的残余误差在这里修正，看作是用户修正数据，每次起飞时可以做修正。
//出场设置这里为0，因为出场设置时，根据当时环境矫正到无偏，不需要这个纠偏，
//但后期飞行温度环境变化，会发生改变，则需要用这个参数调节。
//用户校准数据不保存，只在每次校准后有效。
float imu_gyro_user_bias0=0;
float imu_gyro_user_bias1=0;
float imu_gyro_user_bias2=0;
bool _in_imu_calib=false; //是否正在校准Imu

bool _inair=false; //全局飞行中标记。
bool _intof=false; //全局标记，代表是否高度控制处于测距模式下。

float _baro_height=0.0; //气压高度，距离起飞点。这个值用来限制tof测距的有效性，避免发生测距幻觉导致失控。

//指向正北方的yaw角度，通过这个角度可以判断当前机头朝向。
//起飞时指定机头方向，这个值就自动设置。如果朝北这个值为0，朝东，这个值是90，朝南180.
float _heading_yaw=0.0; 

#if defined USE_QMC5883 || defined USE_HMC5883
uint32_t _heading_yaw_last_update_time=0; //0代表没有更新。
#endif

//强制使用稳定版的气压控高，避免测试版控高出现失控无法挽回的问题。
bool _force_stable_baro_height_control=false;


//从GPS模块读取的飞行距离。需要打开odo数据回传。
//掉电记录消失，没有太大意思。
// uint32_t _total_gps_distance; //总飞行距离.
// uint32_t _gps_distance_since_last_reset; //本次飞行距离。

//电池总能量，焦耳，默认设置为常用的 4s 5000mah电池，能量为230kj左右
//和电流表有关，一个电池可以测一次全放电，记录总放出能量。
float total_battery_energy=230000;
//控制方案，如果设置为0，就是没有电流控制，默认1对应默认的电流和速度设置。
//从1到6允许的放电逐渐增加，可遥控设置。0是不限制电流，需要电池够强否则可能过放。
//遥控在任何时候都可设置。
uint8_t power_control_solution=1; 


//电流调节系数，可用一块电池放光电观察电能积分来校准，这之前先校准电压。
//float current_adjustment=1.35; //电流表调节系数。2306  碳纤架
float current_adjustment=1.2; //2806 kv900 碳纤架 悬停测试显示1.2比较准确。
//float current_adjustment=1.1; //2806 kv1050 老机架 
//float current_adjustment=0.95; //天狼星2306.5电机 碳纤架

//电压调节系数，可用万用表测量电池电压来校准。
//float voltage_adjustment=1.0267; //电压表调节系数。2306 碳纤架
float voltage_adjustment=1.008; //电压表调节系数。2806 kv900 碳纤架
//float voltage_adjustment=0.93; //电压表调节系数。2806 kv1050 老机架
//float voltage_adjustment=0.995; //天狼星2306.5电机 碳纤架

//20241008+ 能量警卫/电压警卫。在自动任务时设置，可以警卫能量使用，超出则取消自动任务自动返航。
float guard_energy_thres=0.0;   //0=不生效
float guard_voltage_thres=0.0;  //0=不生效

class _predefined_location {
    public:
    _predefined_location() {lati=0.0;logi=0.0;height=0.0;defined=false;}
    float lati;
    float logi;
    float height;
    bool defined;
};




initial_heading_method _initial_heading_update_method=head_compass0;

//预定义命令码
enum _predefined_command_ids
{
    _predefined_cmd_imu_calib=1,
    _predefined_cmd_user_imu_calib=2,
    _predefined_cmd_compass_calib=3,
    _predefined_cmd_check_motor=4,
    _predefined_cmd_force_stable_baro_ctrl=5,
    _predefined_cmd_takeoff_compass=6,
    _predefined_cmd_takeoff_north=8,
    _predefined_cmd_takeoff_south=9,
    _predefined_cmd_takeoff_east=10,
    _predefined_cmd_takeoff_west=11,
    _predefined_cmd_takeoff_east_south=12,
    _predefined_cmd_takeoff_east_north=13,
    _predefined_cmd_takeoff_west_north=14,
    _predefined_cmd_takeoff_west_south=15,
    _predefined_cmd_landing_home=16,
    _predefined_cmd_return_home=17,
    _predefined_cmd_ps0=20,
    _predefined_cmd_ps1=21,
    _predefined_cmd_ps2=22,
    _predefined_cmd_ps3=23,
    _predefined_cmd_ps4=24,
    _predefined_cmd_ps5=25,
    _predefined_cmd_ps6=26,
    _predefined_cmd_output_log=30,
    _predefined_cmd_takeover=32,
    _predifined_cmd_check_motor=45,
    _predifined_cmd_check_tof=46,
    _predefined_cmd_break_and_return_home=48, //新增预定义命令，终止自动任务返回起点。20250106

};

//温度及对应的裸漂移数据，每芯片单独校准。
//42688的校准数据。
//根据这些测试数据，如果是线性拟合，则 yaw= const - 0.03*temp; 线性参数是 -0.03; 温度升高，则负向漂移越大。
//可以比较当前温度和校准温度的差异，根据温度差去补偿。

// std::map<float, float> yaw_drift_map={
//     {9.5,-0.9419},
//     {10.0,-0.9455},
//     {10.5,-0.9512},
//     {11.0,-0.9654},
//     {11.5,-0.9754},
//     {12.0,-0.9866},
//     {12.5,-0.9937},
//     {13.0,-1.0},
//     {13.5,-1.0136},
//     {14.0,-1.027},
//     {14.5,-1.036},
//     {15.0,-1.0475},
//     {15.5,-1.0693},
//     {16.0,-1.0932},
//     {16.5,-1.1067},
//     {17.0,-1.1272},
//     {17.5,-1.1452},
//     {18.0,-1.1717},
//     {18.5,-1.1882},
//     {19.0,-1.2068},
//     {19.5,-1.2275},
//     {20.0,-1.2507},
//     {20.5,-1.2639},
//     {21.0,-1.2789},
//     {21.5,-1.2896},
//     {22.0,-1.3017},
//     {22.5,-1.3093},
//     {23.0,-1.3208},
//     {23.5,-1.3340},
//     {24.0,-1.3537},
//     {24.5,-1.3635},
//     {25.0,-1.3701},
//     {25.5,-1.3797},
//     {26.0,-1.3884},
// };

float Compass_Heading();

#define FLOAT_LIMIT(a, low, high) { if(a<low) a=low; if(a>high) a=high;}

void Send_remote_message(uint8_t mid)
{
#if defined USE_REMOTE_CTRL
    remote_ctrl.SendPredefinedMessage(mid);
#endif
}

void Send_remote_message(std::string msg)
{
#if defined USE_REMOTE_CTRL
    remote_ctrl.SendMessage(msg);
#endif
}


void Send_remote_powerinfo(float v, float amp, float consume)
{
#if defined USE_REMOTE_CTRL
    remote_ctrl.SendPowerInfo(v, amp, consume);
#endif 
}

void Send_remote_gpsinfo(gps_info& gps)
{
#if defined USE_REMOTE_CTRL
    remote_ctrl.SendGpsInfo(gps);
#endif 
}

void Send_remote_status(bool inair, bool gps_bad, bool onr, uint16_t last_sig_ms)
{
#if defined USE_REMOTE_CTRL
    remote_ctrl.SendStatus(inair, gps_bad, onr, last_sig_ms);
#endif 
}


void Send_remote_baroinfo(float press, float temp)
{
#if defined USE_REMOTE_CTRL
    remote_ctrl.SendBaroInfo(press, temp);
#endif
}



void Send_remote_Motorinfo(float m0, float m1, float m2, float m3)
{
#if defined USE_REMOTE_CTRL
    remote_ctrl.SendMotorInfo(m0, m1, m2, m3);
#endif 
}

void Send_remote_Imu_info(float pitch, float roll, float yaw, float hding)
{
#if defined USE_REMOTE_CTRL
    remote_ctrl.SendImuInfo(pitch, roll, yaw, hding);
#endif
}

//气压和海拔高度的关系。
//实测，近地面 每升高1米，气压降低11.25帕。
//海拔1600米，每升高1米，气压降低10.44帕。
//看起来是线性关系。
//20241005修改，按照新的方程，p= 101300 -11.719*h + 0.000496*h*h  
//dp/dh= -11.719+0.000496*h
float air_press_height_relation(float gps_height)
{
   //return 11.2 - gps_height*0.000525; //10000米时，1米气压差11.25-5.25=6.0
   FLOAT_LIMIT(gps_height, 0.0, 9000.0);
   return 11.719 - gps_height*0.000496; 
}


//纯气压推断高差。
//以水平面气压100000为基准
float height_by_air_press(float press)
{
    return (11.719 - sqrt(137.335 - 0.001984*(101300-press)))*1008.0645;
}

float air_press_height_relation_by_press(float press)
{
    float h=height_by_air_press(press);
    return 11.719 - h*0.000496; 
}

//以高度来推算气压值，以水平面气压100000为基准
//根据20241005实测数据，后经过2次拟合，气压和高度的关系是
//p= 101300 -11.719*h + 0.000496*h*h  以水平面气压101300为准
float air_press_by_height(float height)
{
    //return 100000 - (22.4 - 0.000525*height)*height/2.0;  == 100000 - 11.2*height + 0.0002625*height*height;
    //20241005 修改，以今日测试数据拟合为准
    return 101300 - 11.719*height + 0.000496*height*height; //以水平面气压101300为准
}

float height_dif_by_air_press(float press_a, float press_b)
{
    float ha= height_by_air_press(press_a);
    float hb= height_by_air_press(press_b);
    return hb-ha;
}

//根据当前气压和高度差，计算推导的目标高度的气压值， 以水平面气压100000为基准
float air_press_height_refer(float press_base, float height_change)
{
    float hb= height_by_air_press(press_base);
    float h=hb+height_change;
    return air_press_by_height(h);
}


void led_fade_in()
{

    pwm_config cfg=pwm_get_default_config();
    //pwm_freq_set(cfg, 1000); //频率设置为1000.
    cfg.top=LED_TOP_V;
    cfg.div=LED_FEQ_DIV;

    gpio_set_function(LED_PIN, GPIO_FUNC_PWM);
    uint sna = pwm_gpio_to_slice_num(LED_PIN); 
    uint cna = pwm_gpio_to_channel(LED_PIN);
    pwm_init(sna, &cfg, false); //初始化但不运行。

    for(int i=0;i<100;i++)
    {
        uint32_t cc = float(i) /100.0 * (LED_TOP_V + 1);
        pwm_set_chan_level(sna, cna, cc);
        pwm_set_enabled(sna, true);
        sleep_ms(20);
    }

}

void led_fade_out()
{

    pwm_config cfg=pwm_get_default_config();
    //pwm_freq_set(cfg, 1000); //频率设置为1000.
    cfg.top=LED_TOP_V;
    cfg.div=LED_FEQ_DIV;

    gpio_set_function(LED_PIN, GPIO_FUNC_PWM);
    uint sna = pwm_gpio_to_slice_num(LED_PIN); 
    uint cna = pwm_gpio_to_channel(LED_PIN);
    pwm_init(sna, &cfg, false); //初始化但不运行。

    for(int i=0;i<100;i++)
    {
        uint32_t cc = float(100-i) /100.0 * (LED_TOP_V + 1); 
        pwm_set_chan_level(sna, cna, cc); 
        pwm_set_enabled(sna, true);
        sleep_ms(20);
    }

}

//gap_ms是闪烁快慢的度量，250比较合适。
void led_blink(int times, int gap_ms)
{

    for(int i=0;i<times;i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(gap_ms);
        gpio_put(LED_PIN, 0);
        sleep_ms(gap_ms);
    } 

}



float check_system_power_voltage()
{
    //从2024/7/5开始，新造的机器调整分压电阻，支持6s电池的电压。采用30k/4.3k搭配，7.977倍降压.

    //采用3.4v电压参考更接近真实值。
    adc_select_input(2); //VOLTAGE_PIN(28) , 和电路版设计有关的固定值
    uint16_t result = adc_read();
    //*0.96是实际测量出来的差异比例。
    //return 3.4*5.0*0.96*float(result)/4096.0f; //采用降压板，1/5分压电路的测量，这个比较准确。
    return 3.3*7.98*voltage_adjustment*float(result)/4096.0f; 
}

//电流表是电调带的功能，接在29/27/26号口上。
float check_system_power_current()
{

    switch(CURRENT_PIN)
    {
        case 26:
        adc_select_input(0);
        break;
        case 27:
        adc_select_input(1);
        break;
        case 28:
        adc_select_input(2);
        break;
        case 29:
        adc_select_input(3);
        break;
        default:
        break;
    }

    uint16_t result = adc_read();
    //这里假设最大数值是40安。目前这个测量值可能是真值的一半。
    //实际电流，测量电流，测量/实际
    //0.52， 0.35， 67%
    //0.66， 0.41， 62%
    //0.79， 0.42， 53%
    //0.91， 0.4678， 51.4%
    //0.96,  0.472,  49.2%
    //1.06    0.575, 54.2%
    //1.16    0.595  51.2%
    //1.263   0.641  50.7%

    //测量电流是实际电流的50%，所以系数调节为80应该合适。较大电流下没有校准，可能还有偏差。
    //20240407 修改比例值，之前是80，修改为100，因测量值偏低。而且测量电压值也是偏低了。
    //动态电压测量值偏低的原因尚不清楚。实际13.1，在8.6安电流下，测量电压12.4左右。
    //之前电流偏低，电压偏低，这导致能量积分偏低较多。
    //之前实测4节5000mah电池放电总量163kj, 修正后应该是204kj, 理论值250kj
    //之前实测4节4000mah电池放电总量130kj, 修正后应该是163kj, 理论值200kj
    //偏离理论值依旧较大的一个原因是动态电压的测量似乎是偏低的。在电流是8A电压是13.1时，测出的电压是12.4
    return 100.0*current_adjustment*float(result)/4096.0f; //100这个比例适合20*20的小板电调
    
}

uint64_t get_time_us_mark()
{
    absolute_time_t t=get_absolute_time();
    return to_us_since_boot(t);
}

uint32_t get_time_mark()
{
    absolute_time_t t=get_absolute_time();
    return to_ms_since_boot(t);
}

//现在可以发送的较快，最终发射速度取决于通信板。
void Send_remote_proper_info()
{
    //现在高频率发送数据只是发往通信板。通信板再根据无线速率尽力发送。
    //发送频率高，只代表通信板上的数据更新的快。不代表发到遥控器的数据更快。

        uint32_t now =get_time_mark();

#ifdef USE_GPS
        //gps信息发送 25字节， 0.2秒间隔发送
        if(now - last_gps_sent > 250) {
            gps_info a;
            critical_section_enter_blocking(&gps_queue_section);
            a=gps_queue.last();
            critical_section_exit(&gps_queue_section);
            Send_remote_gpsinfo(a);
            //由于这些信息和GPS一组发送，所以调整到这里和GPS一起发给通信板。

            //修改为0.1毫秒最小间隔，如果用毫秒时间长了会溢出uint16范围，20240121修改。
            Send_remote_status(_inair, micro_task.gps_bad_mark , on_remote , (now-last_remote_info.tmark)/100);

            last_gps_sent = now;
        }
#endif

        //其他信息总共38字节，打包发送。每秒1次。
        if(now - last_other_sent > 250)
        {

            float singles[4];
            motor_ctrl.GetBurdens(singles);

            float press; float temp;
            critical_section_enter_blocking(&press_queue_section);
            press=press_queue.last().press;
            temp=press_queue.last().temprature;
            critical_section_exit(&press_queue_section);

            float pitch, roll, yaw; 
            critical_section_enter_blocking(&imu_queue_section);
            pitch= imu_queue.last().angle[0]; 
            roll= imu_queue.last().angle[1]; 
            yaw=imu_queue.last().angle[2]; 
            critical_section_exit(&imu_queue_section);
            

            //Send_remote_powerinfo(fvol0, current);
            Send_remote_powerinfo(system_power_voltage, system_power_current, system_power_used);
            Send_remote_Motorinfo(singles[0], singles[1], singles[2], singles[3]);
            Send_remote_baroinfo(press, temp); 
            if(_inair)
            {
                Send_remote_Imu_info(pitch, roll, yaw, _heading_yaw);
            }
            else
            {
                //20231208,不在空中时返回指南针指向角，0-360
                #if defined USE_HMC5883 || defined USE_QMC5883
                Send_remote_Imu_info(pitch, roll, yaw, Compass_Heading());
                #else
                Send_remote_Imu_info(pitch, roll, yaw, 0);
                #endif
            }

            last_other_sent=now;

        }

}

#if  defined(USE_QMC5883) || defined (USE_HMC5883)
bool Read_Compass_Data(int16_t& xd, int16_t& yd, int16_t& zd)
{
#if defined (USE_HMC5883)
    return i2c_cpass_hmc5883.read(xd, yd, zd);
#elif defined (USE_QMC5883)
    return i2c_cpass_qmc5883.read(xd, yd, zd);
#endif
}


bool Compass_Heading(float& ch, float& cur_yaw)
{

    int16_t cx,cy,cz;
    bool b;

    //为防止偶发的读数失败，尝试多次。
    for(int i=0;i<5;i++)
    {
        b=Read_Compass_Data(cx,cy,cz);
        if(b) break;
        else {
            Send_remote_message(40); //read compass failed.
            sleep_ms(10);
        }
    }

    if(!b) {
        Send_remote_message("read compass failed 5 times.");
        return false;
    }

    // //磁偏差修正，
    // //磁偏差修正，
    // float fcx=cx;
    // float fcy=cy;
    // float fcz=cz;

    // fcx -= magnet_midx;
    // fcy -= magnet_midy;
    // fcz -= magnet_midz;

    float curpitch, curroll;
    critical_section_enter_blocking(&imu_queue_section);
    curpitch=imu_queue.last().angle[0];
    curroll=imu_queue.last().angle[1];
    cur_yaw=imu_queue.last().angle[2];//返回
    critical_section_exit(&imu_queue_section);

    //做一个转化，全部调节为水平。参考ymfc-32
    //http://www.brokking.net/ymfc-32_downloads.html
    //http://www.brokking.net/YMFC-32/YMFC-32_document_1.pdf

    float compXH= cx*cos(curpitch*-0.0174533)
                +cy*sin(curroll*0.0174533)*sin(curpitch*-0.0174533)
                -cz*cos(curroll*0.0174533)*sin(curpitch*-0.0174533);

    float compYH= cy*cos(curroll*0.0174533)
                          +cz*sin(curroll*0.0174533);

    //float fx= float(cx) - magnet_midx;
    //float fy= float(cy) - magnet_midy;
    // float fx= compXH - magnet_midx;
    // float fy= compYH - magnet_midy;
    compXH -= magnet_midx;
    compYH -= magnet_midy;

    float heading;
    //0-360的航向角，顺时针。
    if(compXH <=0 && compYH > 0)
    {
        //0-90
        float ang=atan2(-compXH, compYH)*57.2958; //度数。如果cx绝对值极大，则x轴越靠近北方。返回值接近90度。
        heading = 90-ang;
    }
    else if(compXH <=0 && compYH <=0)
    {
        //270-360
        float ang= atan2(-compXH, -compYH)*57.2958; //前者越大，返回值越接近90度。
        heading = 270+ang;
    }
    else if(compXH >0 && compYH <=0)
    {
        //180-270
        float ang= atan2(compXH, -compYH)*57.2958;
        heading = 270-ang;
    }
    else
    {
        //cx>0, cy>0
        //90-180
        float ang= atan2(compXH, compYH)*57.2958;
        heading = 90+ang;
    }

    ch=heading;
    return true;
}

//从指南针读航向角, 0-360度，世界坐标，同GPS方向角
float Compass_Heading()
{

    int16_t cx,cy,cz;
    bool b=Read_Compass_Data(cx,cy,cz);;
    if(!b) {
        Send_remote_message("read compass failed.");
        return 0;
    }

    // //磁偏差修正，
    // float fcx=cx;
    // float fcy=cy;
    // float fcz=cz;

    // fcx -= magnet_midx;
    // fcy -= magnet_midy;
    // fcz -= magnet_midz;

    float curpitch, curroll;
    critical_section_enter_blocking(&imu_queue_section);
    curpitch=imu_queue.last().angle[0];
    curroll=imu_queue.last().angle[1];
    critical_section_exit(&imu_queue_section);

    //做一个转化，全部调节为水平。参考ymfc-32
    //http://www.brokking.net/ymfc-32_downloads.html
    //http://www.brokking.net/YMFC-32/YMFC-32_document_1.pdf

    float compXH= cx*cos(curpitch*-0.0174533)
                +cy*sin(curroll*0.0174533)*sin(curpitch*-0.0174533)
                -cz*cos(curroll*0.0174533)*sin(curpitch*-0.0174533);

    float compYH= cy*cos(curroll*0.0174533)
                          +cz*sin(curroll*0.0174533);

    //float fx= float(cx) - magnet_midx;
    //float fy= float(cy) - magnet_midy;
    // float fx= compXH - magnet_midx;
    // float fy= compYH - magnet_midy;
    compXH -= magnet_midx;
    compYH -= magnet_midy;


    //航向角和芯片安装方向有关。

    //之前的安装方向适合下面的代码。
    // float heading;
    // //0-360的航向角，顺时针。
    // if(compXH <=0 && compYH > 0)
    // {
    //     //0-90
    //     float ang=atan2(-compXH, compYH)*57.2958; //度数。如果cx绝对值极大，则x轴越靠近北方。返回值接近90度。
    //     heading = 90-ang;
    // }
    // else if(compXH <=0 && compYH <=0)
    // {
    //     //270-360
    //     float ang= atan2(-compXH, -compYH)*57.2958; //前者越大，返回值越接近90度。
    //     heading = 270+ang;
    // }
    // else if(compXH >0 && compYH <=0)
    // {
    //     //180-270
    //     float ang= atan2(compXH, -compYH)*57.2958;
    //     heading = 270-ang;
    // }
    // else
    // {
    //     //cx>0, cy>0
    //     //90-180
    //     float ang= atan2(compXH, compYH)*57.2958;
    //     heading = 90+ang;
    // }

    // return heading;

    //现在主控板上面装的是qmc5883, 之前是hmc5883,
    //芯片朝向不同，现在的安装方向是，y轴指向左侧，x轴指向前方，当他们指向北方时，读数是最大值，指向南方是最小值。
    
    float ang= 90.0 - atan2(compXH, compYH)*57.2958;
    if(ang<0) ang+= 360;
    return ang;

}
#endif

//老版本
// void logmessage(std::string str)
// {
//     char buf[20];
//     uint32_t t= get_time_mark();
//     sprintf(buf, "%u,", t); //20240719 这里修改为逗号，更好处理数据。
//     std::string cont= std::string(buf)+str;
//     mutex_enter_blocking(&log_mutex); //20250313
//     for(size_t i=0;i<cont.size();i++) log_msg.push(cont[i]);
//     mutex_exit(&log_mutex); //20250313
// }

//20250314 整体修改
void logmessage(std::string str)
{
    char buf[20];
    uint32_t t= get_time_mark();
    sprintf(buf, "%u,", t); //20240719 这里修改为逗号，更好处理数据。

    mutex_enter_blocking(&log_mutex); //20250313
    for(size_t i=0;i<strlen(buf);i++) log_msg.push(buf[i]);
    for(size_t i=0;i<str.size();i++) log_msg.push(str[i]);
    mutex_exit(&log_mutex); //20250313
}

//20250314 注销，已经不用这个了。
// bool write_calib_data_to_flash()
// {
//     #define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - CALIBDATA)
//     uint8_t buf[FLASH_SECTOR_SIZE]={0};
//     sprintf((char*)buf, 
//     "imu_gyro_raw_bias0=%f\nimu_gyro_raw_bias1=%f\nimu_gyro_raw_bias2=%f\nimu_acce_raw_ratio=%f\n"
//     "install_bias_angle0=%f\ninstall_bias_angle1=%f\ninstall_bias_gnd_acc0=%f\ninstall_bias_gnd_acc1=%f\n"
//     "install_bias_gnd_acc2=%f\nimu_calib_temprature=%f\nmagnet_midx=%f\nmagnet_midy=%f\nmagnet_midz=%f",
//         imu_gyro_raw_bias0, imu_gyro_raw_bias1, imu_gyro_raw_bias2, imu_acce_raw_ratio, 
//         install_bias_angle0, install_bias_angle1, install_bias_gnd_acc0, install_bias_gnd_acc1, 
//         install_bias_gnd_acc2, imu_calib_temprature, magnet_midx, magnet_midy, magnet_midz
//     );

//     int nint=save_and_disable_interrupts();
//     flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE); //必须是整扇区，4096的倍数。
//     flash_range_program(FLASH_TARGET_OFFSET, buf, FLASH_SECTOR_SIZE);
//     restore_interrupts(nint);

//     //read back and check content.
//     const uint8_t* cont = (const uint8_t*)(XIP_BASE + FLASH_TARGET_OFFSET);
//     bool ok=true;
//     for(size_t i=0;i<FLASH_SECTOR_SIZE;i++) {
//         if(cont[i]!=buf[i]) {
//             ok=false;
//             break;
//         }
//     }
//     return ok;
//     #undef FLASH_TARGET_OFFSET
// }

//20250314 注销不用
// void read_calib_data_from_flash()
// {
//     #define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - CALIBDATA)
// 	std::string bufstr;
// 	//整个扇区读入。
// 	bufstr.append((const char *)(XIP_BASE + FLASH_TARGET_OFFSET), FLASH_SECTOR_SIZE);
// 	//每行是一个数据记录项。
// 	size_t ed = bufstr.find_first_of('\0');
// 	if (ed != std::string::npos) {
// 		bufstr.erase(ed);
// 	}

//     //printf("args:%s\n", bufstr.c_str());

// 	while (!bufstr.empty())
// 	{
// 		ed = bufstr.find_first_of('\n');
// 		std::string oneline;
// 		if (ed != std::string::npos) {
// 			oneline = bufstr.substr(0, ed);
// 			bufstr.erase(0, ed+1);
// 		}
// 		else {
// 			oneline = bufstr;
// 			bufstr.clear();
// 		}

// 		size_t eq = oneline.find_first_of('=');
// 		if (eq != std::string::npos)
// 		{
// 			std::string hd = oneline.substr(0, eq);
// 			std::string tail = oneline.substr(eq+1);
//             float v= atof(tail.c_str());
//             if(hd.compare("imu_gyro_raw_bias0")==0)
//             {
//                 imu_gyro_raw_bias0=v;
//             }
//             else if(hd.compare("imu_gyro_raw_bias1")==0)
//             {
//                 imu_gyro_raw_bias1=v;
//             }
//             else if(hd.compare("imu_gyro_raw_bias2")==0)
//             {
//                 imu_gyro_raw_bias2=v;
//             }
//             else if(hd.compare("imu_acce_raw_ratio")==0)
//             {
//                 imu_acce_raw_ratio=v;
//             }
//             else if(hd.compare("install_bias_angle0")==0)
//             {
//                 install_bias_angle0=v;
//             }
//             else if(hd.compare("install_bias_angle1")==0)
//             {
//                 install_bias_angle1=v;
//             }
//             else if(hd.compare("install_bias_gnd_acc0")==0)
//             {
//                 install_bias_gnd_acc0=v;
//             }
//             else if(hd.compare("install_bias_gnd_acc1")==0)
//             {
//                 install_bias_gnd_acc1=v;
//             }
//             else if(hd.compare("install_bias_gnd_acc2")==0)
//             {
//                 install_bias_gnd_acc2=v;
//             }
//             else if(hd.compare("imu_calib_temprature")==0)
//             {
//                 imu_calib_temprature=v;
//             }
//             else if(hd.compare("magnet_midx")==0)
//             {
//                 magnet_midx=v;
//             }
//             else if(hd.compare("magnet_midy")==0)
//             {
//                 magnet_midy=v;
//             }
//             else if(hd.compare("magnet_midz")==0)
//             {
//                 magnet_midz=v;
//             }

// 		}
// 	}

//     #undef FLASH_TARGET_OFFSET
// }

//展开参数。
void Spread_arguements()
{
//将得到的数据赋值给运行数据。
    if(adjustable_running_arguements.find("IBA0")!=adjustable_running_arguements.end())
    {
        install_bias_angle0=adjustable_running_arguements["IBA0"];
    }
    if(adjustable_running_arguements.find("IBA1")!=adjustable_running_arguements.end())
    {
        install_bias_angle1=adjustable_running_arguements["IBA1"];
    }
    if(adjustable_running_arguements.find("BGA0")!=adjustable_running_arguements.end())
    {
        install_bias_gnd_acc0=adjustable_running_arguements["BGA0"];
    }
    if(adjustable_running_arguements.find("BGA1")!=adjustable_running_arguements.end())
    {
        install_bias_gnd_acc1=adjustable_running_arguements["BGA1"];
    }
    if(adjustable_running_arguements.find("BGA2")!=adjustable_running_arguements.end())
    {
        install_bias_gnd_acc2=adjustable_running_arguements["BGA2"];
    }
    if(adjustable_running_arguements.find("GRB0")!=adjustable_running_arguements.end())
    {
        imu_gyro_raw_bias0=adjustable_running_arguements["GRB0"];
    }
    if(adjustable_running_arguements.find("GRB1")!=adjustable_running_arguements.end())
    {
        imu_gyro_raw_bias1=adjustable_running_arguements["GRB1"];
    }
    if(adjustable_running_arguements.find("GRB2")!=adjustable_running_arguements.end())
    {
        imu_gyro_raw_bias2=adjustable_running_arguements["GRB2"];
    }
    if(adjustable_running_arguements.find("IARR")!=adjustable_running_arguements.end())
    {
        imu_acce_raw_ratio=adjustable_running_arguements["IARR"];
    }
    if(adjustable_running_arguements.find("ICTP")!=adjustable_running_arguements.end())
    {
        imu_calib_temprature=adjustable_running_arguements["ICTP"];
    }
    if(adjustable_running_arguements.find("MAGX")!=adjustable_running_arguements.end())
    {
        magnet_midx=adjustable_running_arguements["MAGX"];
    }
    if(adjustable_running_arguements.find("MAGY")!=adjustable_running_arguements.end())
    {
        magnet_midy=adjustable_running_arguements["MAGY"];
    }
    if(adjustable_running_arguements.find("MAGZ")!=adjustable_running_arguements.end())
    {
        magnet_midz=adjustable_running_arguements["MAGZ"];
    }

    if(adjustable_running_arguements.find("CADJ")!=adjustable_running_arguements.end())
    {
        current_adjustment=adjustable_running_arguements["CADJ"];
    }
    if(adjustable_running_arguements.find("VADJ")!=adjustable_running_arguements.end())
    {
        voltage_adjustment=adjustable_running_arguements["VADJ"];
    }
    if(adjustable_running_arguements.find("ITC1")!=adjustable_running_arguements.end()) //20250310+
    {
        imu_gyroz_temp_comp=adjustable_running_arguements["ITC1"];
        //温度补偿系数检查
        if(imu_gyroz_temp_comp<0.0||imu_gyroz_temp_comp>0.01)
        {
            printf("imu_gyroz_temp_comp=%f is bad, set to default\n", imu_gyroz_temp_comp);
            imu_gyroz_temp_comp=0.005;
            adjustable_running_arguements["ITC1"]=0.005;
        }
    }
    if(adjustable_running_arguements.find("BALM")!=adjustable_running_arguements.end()) //20250324+
    {
        balance_main_ratio=adjustable_running_arguements["BALM"];
        //也许设置的参数不当，需要防范不当参数进入运行，检查各参数。
        if(balance_main_ratio > 3.0|| balance_main_ratio < 0.3)
        {
            printf("balance_main_ratio=%f is bad, set to default\n", balance_main_ratio);
            balance_main_ratio=1.0;
            adjustable_running_arguements["BALM"]=1.0;
        }
    }
    if(adjustable_running_arguements.find("BALP")!=adjustable_running_arguements.end()) //20250324+
    {
        balance_p_ratio=adjustable_running_arguements["BALP"];
        //也许设置的参数不当，需要防范不当参数进入运行，检查各参数。
        if(balance_p_ratio > 2.0|| balance_p_ratio < 0.01)
        {
            printf("balance_p_ratio=%f is bad, set to default 0.5\n", balance_p_ratio);
            balance_p_ratio=0.5;
            adjustable_running_arguements["BALP"]=0.5;
        }
    }
    if(adjustable_running_arguements.find("BALI")!=adjustable_running_arguements.end()) //20250324+
    {
        balance_i_ratio=adjustable_running_arguements["BALI"];
        //也许设置的参数不当，需要防范不当参数进入运行，检查各参数。
        if(balance_i_ratio > 1.0|| balance_i_ratio < 0.01)
        {
            printf("balance_i_ratio=%f is bad, set to default 0.1\n", balance_i_ratio);
            balance_i_ratio=0.1;
            adjustable_running_arguements["BALI"]=0.1;
        }
    }
    if(adjustable_running_arguements.find("BALD")!=adjustable_running_arguements.end()) //20250324+
    {
        balance_d_ratio=adjustable_running_arguements["BALD"];
        //也许设置的参数不当，需要防范不当参数进入运行，检查各参数。
        if(balance_d_ratio > 1.0|| balance_d_ratio < 0.01)
        {
            printf("balance_d_ratio=%f is bad, set to default 0.6\n", balance_d_ratio);
            balance_d_ratio=0.6;
            adjustable_running_arguements["BALD"]=0.6;
        }
    }
    if(adjustable_running_arguements.find("BALX")!=adjustable_running_arguements.end()) //20250324+
    {
        balance_x_ratio=adjustable_running_arguements["BALX"];
        //也许设置的参数不当，需要防范不当参数进入运行，检查各参数。
        if(balance_x_ratio > 1.0|| balance_x_ratio < 0.0)
        {
            printf("balance_x_ratio=%f is bad, set to default 0.15\n", balance_x_ratio);
            balance_x_ratio=0.15;
            adjustable_running_arguements["BALX"]=0.15;
        }
    }
    if(adjustable_running_arguements.find("BALW")!=adjustable_running_arguements.end()) //20250324+
    {
        balance_w_ratio=adjustable_running_arguements["BALW"];
        //也许设置的参数不当，需要防范不当参数进入运行，检查各参数。
        if(balance_w_ratio > 1.0|| balance_w_ratio < 0.0)
        {
            printf("balance_w_ratio=%f is bad, set to default 0.1\n", balance_w_ratio);
            balance_w_ratio=0.1;
            adjustable_running_arguements["BALW"]=0.1;
        }
    }
    if(adjustable_running_arguements.find("BALZ")!=adjustable_running_arguements.end()) //20250324+
    {
        balance_z_ratio=adjustable_running_arguements["BALZ"];
        //也许设置的参数不当，需要防范不当参数进入运行，检查各参数。
        if(balance_z_ratio > 1.0|| balance_z_ratio < 0.0)
        {
            printf("balance_z_ratio=%f is bad, set to default 0.02\n", balance_z_ratio);
            balance_z_ratio=0.02;
            adjustable_running_arguements["BALZ"]=0.02;
        }
    }



    if(voltage_adjustment<0.5 || voltage_adjustment>2.0)
    {
        printf("voltage_adjustment=%f is not ok, set to default\n", voltage_adjustment);
        voltage_adjustment=1.0;
        adjustable_running_arguements["VADJ"]=1.0;
    }

    if(current_adjustment<0.5 || current_adjustment>2.0)
    {
        printf("current_adjustment=%f is not ok, set to default\n", current_adjustment);
        current_adjustment=1.0;
        adjustable_running_arguements["CADJ"]=1.0;
    }
}

//两个新的函数读写flash里的参数。参数名4字节+float4自己组成一个数据点。
void read_arguements_from_flash()
{
    #define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - CALIBDATA)
	std::string bufstr;
	//整个扇区读入。
	bufstr.append((const char *)(XIP_BASE + FLASH_TARGET_OFFSET), FLASH_SECTOR_SIZE);
    //拆解，每8个字节一个记录点。
    for(size_t i=0;i<FLASH_SECTOR_SIZE/8;i++)
    {
        char name[4];
        float val;
        name[0]=bufstr[i*8+0];
        name[1]=bufstr[i*8+1];
        name[2]=bufstr[i*8+2];
        name[3]=bufstr[i*8+3];
        memcpy(&val, bufstr.data()+i*8+4,4);
        bool b=arguement_name::isLeagleName(name);
        if(!b) break;

        arguement_name sn(name);
        adjustable_running_arguements[sn]=val;
    }

    Spread_arguements();


    #undef FLASH_TARGET_OFFSET
}

bool write_arguements_to_flash()
{
    #define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - CALIBDATA)
    uint8_t buf[FLASH_SECTOR_SIZE]={0};

    //填充内容，每8个字节一段。
    //首先将运行数据填写到adjustable_running_arguements
    adjustable_running_arguements["IBA0"]=install_bias_angle0;
    adjustable_running_arguements["IBA1"]=install_bias_angle1;
    adjustable_running_arguements["BGA0"]=install_bias_gnd_acc0;
    adjustable_running_arguements["BGA1"]=install_bias_gnd_acc1;
    adjustable_running_arguements["BGA2"]=install_bias_gnd_acc2;
    adjustable_running_arguements["GRB0"]=imu_gyro_raw_bias0;
    adjustable_running_arguements["GRB1"]=imu_gyro_raw_bias1;
    adjustable_running_arguements["GRB2"]=imu_gyro_raw_bias2;
    adjustable_running_arguements["IARR"]=imu_acce_raw_ratio;
    adjustable_running_arguements["ICTP"]=imu_calib_temprature;
    adjustable_running_arguements["MAGX"]=magnet_midx;
    adjustable_running_arguements["MAGY"]=magnet_midy;
    adjustable_running_arguements["MAGZ"]=magnet_midz;
    adjustable_running_arguements["CADJ"]=current_adjustment;
    adjustable_running_arguements["VADJ"]=voltage_adjustment;
    adjustable_running_arguements["ITC1"]=imu_gyroz_temp_comp;//20250310+
    adjustable_running_arguements["BALM"]=balance_main_ratio;//20250324+
    adjustable_running_arguements["BALP"]=balance_p_ratio;//20250401+
    adjustable_running_arguements["BALI"]=balance_i_ratio;//20250401+i
    adjustable_running_arguements["BALD"]=balance_d_ratio;//20250401+
    adjustable_running_arguements["BALX"]=balance_x_ratio;//20250401+
    adjustable_running_arguements["BALW"]=balance_w_ratio;//20250401+
    adjustable_running_arguements["BALZ"]=balance_z_ratio;//20250401+

    size_t curpos=0; //位置
    for(auto it=adjustable_running_arguements.begin(); it!=adjustable_running_arguements.end(); it++)
    {
        if(!arguement_name::isLeagleName(it->first.name)) continue;

        buf[curpos+0]=it->first.name[0];
        buf[curpos+1]=it->first.name[1];
        buf[curpos+2]=it->first.name[2];
        buf[curpos+3]=it->first.name[3];
        memcpy(buf+curpos+4, &it->second, 4);
        curpos+=8;
        if(curpos>=FLASH_SECTOR_SIZE-8) break; //防止溢出。
    }

    int nint=save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE); //必须是整扇区，4096的倍数。
    flash_range_program(FLASH_TARGET_OFFSET, buf, FLASH_SECTOR_SIZE);
    restore_interrupts(nint);

    //read back and check content.
    const uint8_t* cont = (const uint8_t*)(XIP_BASE + FLASH_TARGET_OFFSET);
    bool ok=true;
    for(size_t i=0;i<FLASH_SECTOR_SIZE;i++) {
        if(cont[i]!=buf[i]) {
            ok=false;
            break;
        }
    }
    return ok;
    #undef FLASH_TARGET_OFFSET
}

void write_log_to_flash()
{
    //修改为使用后面32个扇区用来记录日志。128KB字节数。
    #define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - LOGSIZE) //2M flash 最后32个扇区留作记录空间。

    std::string bufstr;

    int sectornum=0;

    if(log_msg.size()<LOGSIZE)
    {
        for(size_t i=0;i<log_msg.size();i++)
        {
            bufstr+=(log_msg[i]);

            if(bufstr.size()==FLASH_SECTOR_SIZE) {
                //采用按需擦除，一个个扇区擦除。
                int nint=save_and_disable_interrupts();
                flash_range_erase(FLASH_TARGET_OFFSET+sectornum*FLASH_SECTOR_SIZE, FLASH_SECTOR_SIZE); //必须是整扇区，4096的倍数。
                flash_range_program(FLASH_TARGET_OFFSET+sectornum*FLASH_SECTOR_SIZE, (uint8_t *)bufstr.data(), FLASH_SECTOR_SIZE);
                restore_interrupts(nint);
                sectornum++;
                bufstr.clear();
            }

        }

        //存在有零头的情况。
        if(!bufstr.empty()) {
            while(bufstr.size()<FLASH_SECTOR_SIZE) bufstr.push_back(0); //充满一个扇区数据
            int nint=save_and_disable_interrupts();
            flash_range_erase(FLASH_TARGET_OFFSET+sectornum*FLASH_SECTOR_SIZE, FLASH_SECTOR_SIZE); //必须是整扇区，4096的倍数。
            flash_range_program(FLASH_TARGET_OFFSET+sectornum*FLASH_SECTOR_SIZE, (uint8_t *)bufstr.data(), FLASH_SECTOR_SIZE);
            restore_interrupts(nint);
            sectornum++;
            bufstr.clear();            

        }
    }
    else
    {
        for(size_t i=0;i<LOGSIZE;i++)
        {
            bufstr+=(log_msg[i]);

            if(bufstr.size()==FLASH_SECTOR_SIZE) {
                //采用按需擦除，一个个扇区擦除。
                int nint=save_and_disable_interrupts();
                flash_range_erase(FLASH_TARGET_OFFSET+sectornum*FLASH_SECTOR_SIZE, FLASH_SECTOR_SIZE); //必须是整扇区，4096的倍数。
                flash_range_program(FLASH_TARGET_OFFSET+sectornum*FLASH_SECTOR_SIZE, (uint8_t *)bufstr.data(), FLASH_SECTOR_SIZE);
                restore_interrupts(nint);
                sectornum++;
                bufstr.clear();
            }

        }        
    }

    #undef FLASH_TARGET_OFFSET
}


//读取日志记录并输出
void read_log_from_flash()
{

    //修改为最后10个扇区记录
    #define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - LOGSIZE) //2M flash 最后10个扇区留作记录空间。

    size_t segnum= LOGSIZE/FLASH_SECTOR_SIZE; //扇区数量

    std::string bufstr;

    for(size_t sec=0;sec<segnum;sec++)
    {
        //如果logsize特别大，一次性读入内存会有问题，需要一段一段的处理。
        //一次读一扇区。
        //printf("read sec %d\n", sec);

        bufstr.append((const char *)(XIP_BASE + FLASH_TARGET_OFFSET + sec * FLASH_SECTOR_SIZE), FLASH_SECTOR_SIZE);

        size_t end = bufstr.find_first_of('\0');
        
        //寻找记录结束点。
        if(end!=std::string::npos) {
            //printf("found end mark in this sec\n");
            bufstr.erase(end);
            break;
        }

        // //按回车'\n'切割内容行。
        while(!bufstr.empty())
        {
            size_t spt= bufstr.find_first_of('\n');
            if(spt==std::string::npos) {
                break;
            }else{
                std::string line= bufstr.substr(0,spt+1);
                bufstr.erase(0, spt+1);
                printf(line.c_str());
            }
        }


        //大量数据内找不到换行符号，错误。
        if(bufstr.size()>=3*FLASH_SECTOR_SIZE) {
            printf("\nno return in long string, break\n");
            break;
        }
    }

    //按回车'\n'切割内容行。
    while(!bufstr.empty())
    {
        size_t spt= bufstr.find_first_of('\n');
        if(spt==std::string::npos) {
            printf(bufstr.c_str());
            break;
        }else{
            std::string line= bufstr.substr(0,spt+1);
            bufstr.erase(0, spt+1);
            printf(line.c_str());
        }
    }

    printf("\nread_log_from_flash end\n");
    sleep_ms(100);

    #undef FLASH_TARGET_OFFSET
}

//遥控器发过来的电源控制方案，修改。默认1
void power_control_solution_adjust(uint8_t ps)
{
    power_control_solution=ps;
}



//显示tof数据
#ifdef USE_TOF
void Check_tof_data()
{

    //oflow经过转换，最小分辨率是0.2，没看见有比0.2更小的数值了。
    //
    sleep_ms(50);

    while(1)
    {
        if(last_predefined_task!=0) {
            last_predefined_task=0;
            printf("task!=0, quit tof check\n");
            Send_remote_message("quit tof check");
            break;
        }

        tof_oflow_info ref;
        critical_section_enter_blocking(&tof_oflow_queue_section);
        ref= tof_oflow_queue.last();
        critical_section_exit(&tof_oflow_queue_section);

        if(ref.tmark!=0)
        {
            //printf("td=%f, %d\n", ref.fixed_tof_distance, tof_oflow_queue.last().tmark- tof_oflow_queue.last(1).tmark);
            if(ref.tof_valid)
            {
                //char buf[128];
                //sprintf(buf, "td=%3.2f", ref.tof_distance);
                //Send_remote_message(buf);
                printf("td=%3.2f, %3.2f\n", ref.tof_distance, ref.fixed_tof_distance);
            }
            else
            {
                printf("tof invalid\n");
            }
        }
        else
        {
            printf("no tof data\n");
        }

        sleep_ms(1000);
    }

}
#endif
//微观姿态调节任务，跑在第二核心上。
//现在新增加给方向判断喂数据的功能。先喂数据，后执行微动作。
static void micro_task_exec()
{

    //设置微任务初始数据
    micro_task.taskid= _micro_task_id_nop; //初始状态。
    micro_task.status= _micro_task_status_none; //初始状态。
    //micro_task.help_landing=false;
    micro_task.help_takeoff=false;
    micro_task.tof_dominate=true; //初始状态。
    micro_task.vspeed=0;
    micro_task.yawspeed=0;

    while (1) {
        //执行微动作。
        sem_acquire_blocking(&imu_data_to_action);
        sem_reset(&imu_data_to_action, 0); //清理信号标记

        //执行微动作。
        //加锁会阻断传感器数据进入，进而阻断中断，可以考虑不加锁。内部去处理加锁情况。
        micro_exec.do_micro_task();

    }
}

float angle_distance(float angle_from, float angle_to)
{
    if(angle_from < 0) angle_from += 360;
    if(angle_to < 0) angle_to +=360;
    if(angle_to < angle_from) angle_to += 360; //让angle_to始终大于angle_from

    float counter_clockwise_angle= angle_to - angle_from; //逆时针旋转角度。范围(0 ~ 360), 始终为正数。

    if(counter_clockwise_angle<=180.0) return counter_clockwise_angle;
    else return counter_clockwise_angle - 360.0f;    
}

void burn_motor_test()
{
    float bur[4];

    Send_remote_message("begin stage1 test");
    for(int i=5;i<95;i++)
    {
        bur[0]=float(i+5)/100.0;
        bur[1]=float(i+5)/100.0;
        bur[2]=float(i+5)/100.0;
        bur[3]=float(i+5)/100.0;
        motor_ctrl.SetBurdens(bur, bur[0]*4);
        sleep_ms(2000);
    }

    Send_remote_message("begin stage2 test");

    for(int i=95;i>=5;i--)
    {
        bur[0]=float(i+5)/100.0;
        bur[1]=float(i+5)/100.0;
        bur[2]=float(i+5)/100.0;
        bur[3]=float(i+5)/100.0;
        motor_ctrl.SetBurdens(bur, bur[0]*4);
        sleep_ms(2000);
    }

    Send_remote_message("begin stage3 test");

    for(size_t i=0;i<2000;i++)
    {
        bur[0]=random_burs1(rgen);
        bur[1]=random_burs1(rgen);
        bur[2]=random_burs1(rgen);
        bur[3]=random_burs1(rgen);
        sleep_ms(50);
    } 

    for(size_t i=0;i<10000;i++)
    {
        bur[0]=random_burs2(rgen);
        bur[1]=random_burs2(rgen);
        bur[2]=random_burs2(rgen);
        bur[3]=random_burs2(rgen);
        sleep_ms(50);
    } 

    for(size_t i=0;i<10000;i++)
    {
        bur[0]=random_burs3(rgen);
        bur[1]=random_burs3(rgen);
        bur[2]=random_burs3(rgen);
        bur[3]=random_burs3(rgen);
        sleep_ms(50);
    } 

    Send_remote_message("finish motor test");
}

void basic_motor_check()
{
        Send_remote_message(16);

        float bur[4];
        bur[0]=0.1;
        bur[1]=0;
        bur[2]=0;
        bur[3]=0;
        motor_ctrl.SetBurdens(bur, 0.1);
        for(int i=0;i<5;i++)
        {
            Send_remote_proper_info();
            sleep_ms(1000);
        }

        Send_remote_message(17);

        bur[0]=0;
        bur[1]=0.1;
        bur[2]=0;
        bur[3]=0;
        motor_ctrl.SetBurdens(bur, 0.1);
        for(int i=0;i<5;i++)
        {
            Send_remote_proper_info();
            sleep_ms(1000);
        }

        Send_remote_message(18);

        bur[0]=0;
        bur[1]=0;
        bur[2]=0.1;
        bur[3]=0;
        motor_ctrl.SetBurdens(bur, 0.1);
        for(int i=0;i<5;i++)
        {
            Send_remote_proper_info();
            sleep_ms(1000);
        }

        Send_remote_message(19);

        bur[0]=0;
        bur[1]=0;
        bur[2]=0;
        bur[3]=0.1;
        motor_ctrl.SetBurdens(bur, 0.1);
        for(int i=0;i<5;i++)
        {
            Send_remote_proper_info();
            sleep_ms(1000);
        }

        bur[0]=0.1;
        bur[1]=0.1;
        bur[2]=0.1;
        bur[3]=0.1;
        motor_ctrl.SetBurdens(bur, 0.4);
        for(int i=0;i<10;i++)
        {
            Send_remote_proper_info();
            sleep_ms(1000);
        }

        bur[0]=0;
        bur[1]=0;
        bur[2]=0;
        bur[3]=0;
        motor_ctrl.SetBurdens(bur, 0);
}

void basic_motor_check2()
{
        Send_remote_message("check all motor");

        float bur[4];
        bur[0]=0.1;
        bur[1]=0.1;
        bur[2]=0.1;
        bur[3]=0.1;
        motor_ctrl.SetBurdens(bur, 0.4);
        sleep_ms(5000);

        bur[0]=0;
        bur[1]=0;
        bur[2]=0;
        bur[3]=0;
        motor_ctrl.SetBurdens(bur, 0);
}

//摇杆控制电机测试。
void motor_test_by_remote()
{

    Send_remote_message("about to test motor by remote");
    sleep_ms(3000);

    while(1)
    {
        if(last_predefined_task!=0) {
            last_predefined_task=0;
            motor_ctrl.SetBurdens(0, 0, 0, 0, 0);
            Send_remote_message("quit test");
            break;
        }
        //20230905 修改，现在用整数来表示垂直速度，1个整数代表0.1m/s的速度。
        //摇杆范围-16-16，*2范围是-32-32,对应3.2m/s速度范围。
        //现在控高比较稳定，修改为*3，对应上下速度范围4.8米/秒
        //这里首先防备传过来的数据在预期范围，如果数据错误，超过了范围，比如是48
        //那么*3之后就溢出了int8范围，可能导致上下方向和预期是反的。
        if(last_remote_info.pos4 < -16) last_remote_info.pos4=-16;
        if(last_remote_info.pos4 > 16) last_remote_info.pos4=16;

        int8_t vs= -last_remote_info.pos4*3; //int8 范围上下128，*3在范围内。
        if(vs > 48) vs=48; //降低
        else if(vs<-48) vs=-48; //升高

        //正值为降低，那么 vs=48对应电机速度0，vs=-48对应电机最大，暂定为0.8.
        //其他值为线性处理。
        float bur= 1.0 - (float(vs)+48.0)/96.0; //0-1.0范围内

        motor_ctrl.SetBurdens(bur*0.8, bur*0.8, bur*0.8, bur*0.8, 3.2*bur);
        
        sleep_ms(50);
    }
}

void cross_motor_check()
{
    for(int i=0;i<1;i++)
    {
        Send_remote_message(22);
        float bur[4];
        bur[0]=0.1;
        bur[1]=0;
        bur[2]=0;
        bur[3]=0.1;
        motor_ctrl.SetBurdens(bur, 0.2);
        sleep_ms(1000);

        Send_remote_message(23);
        bur[0]=0;
        bur[1]=0.1;
        bur[2]=0.1;
        bur[3]=0;
        motor_ctrl.SetBurdens(bur, 0.2);
        sleep_ms(1000);

        bur[0]=0;
        bur[1]=0;
        bur[2]=0;
        bur[3]=0;
        motor_ctrl.SetBurdens(bur, 0);
    }
}

void power_prepare()
{
    adc_init(); //Initialise the ADC HW
    adc_gpio_init(VOLTAGE_PIN); //标准Pico才能在28脚准确测量电压，兼容板测不准。
    adc_gpio_init(CURRENT_PIN); //27现在接电流表，之前用过29脚。
}



void basic_sensor_check()
{
    Send_remote_message(20);

    //准备工作：观察设备数据流，等待gps定位精度足够
    //等待数据的过程中，不断回传GPS数据。

    while(1) {

        imu_info b;
        press_info c;
        tof_oflow_info d;
        tof_oflow_info e;
        critical_section_enter_blocking(&imu_queue_section);
        b=imu_queue.front(); //请求第一个数，要求队列灌满
        critical_section_exit(&imu_queue_section);

        critical_section_enter_blocking(&press_queue_section);
        c=press_queue.front(); //请求第一个数，要求队列灌满
        critical_section_exit(&press_queue_section);


#if defined USE_TOF
        critical_section_enter_blocking(&tof_oflow_queue_section);
        d=tof_oflow_queue.front();
        e=tof_oflow_queue.last();
        critical_section_exit(&tof_oflow_queue_section);
#endif

#if defined USE_TOF
        if(b.tmark!=0 && c.tmark!=0 && d.tmark!=0 && e.tof_valid) {

            break;
        }
#else
        if(b.tmark!=0 && c.tmark!=0) {

            break;
        }
#endif

        if(b.tmark==0) {
            printf("no imu data\n");
            Send_remote_message("no imu data");
        }
        if(c.tmark==0) {
            printf("no press data\n");
            Send_remote_message("no press data");
        }

#ifdef USE_TOF 
        if(d.tmark==0) {
            //暂时不显示这个消息是因为要调试GPS，防止掩盖GPS的输出
            Send_remote_message("tof data not full");
        }

        if(!e.tof_valid) {
            Send_remote_message("last tof data invalid");
        }
#endif
        sleep_ms(1000);

    } 

    Send_remote_message(21);
}


void Update_Heading_Yaw(float cur_yaw)
{

 #if defined USE_QMC5883 || defined USE_HMC5883
 //起初在静态时，可以猜测一个北向。
    
    float comp_heading= Compass_Heading(); 

    //compass 信息回头发送给遥控。
    // char buf[64];
    // sprintf(buf,"cpass heading: %f", comp_heading);
    // Send_remote_message(buf);

    #if defined USE_GPS
    gps_info last_gps;
    float dist, magdec;
    critical_section_enter_blocking(&gps_queue_section);
    //所有GPS不提供磁偏角，自己算。
    last_gps=gps_queue.last();
    critical_section_exit(&gps_queue_section);
    //使用2023年的磁极位置。
    global_gps_dist_angle(last_gps.latitude, last_gps.longitude, 86.146, 146.826, dist, magdec);
    //_heading_yaw = comp_heading - magdec;
    //纠正磁偏角，给出磁场指示的世界系方向。
    comp_heading-= magdec;
    if(comp_heading<0) comp_heading+=360;
    if(comp_heading>360) comp_heading-=360;
    #else
    //_heading_yaw = comp_heading - 2.8f;
    comp_heading-=2.8f;
    if(comp_heading<0) comp_heading+=360;
    if(comp_heading>360) comp_heading-=360;
    #endif

    _heading_yaw = cur_yaw + comp_heading;
    if(_heading_yaw < -180) _heading_yaw+=360;
    if(_heading_yaw > 180) _heading_yaw-=360;

    _heading_yaw_last_update_time=get_time_mark();

#else
    //不使用指南针时，就假设当前机头指向北方。可以观察在无指南针时系统是否可以迅速找到北方。
    _heading_yaw=cur_yaw;
#endif
}

void auto_task_exec()
{
    //记录间隔时间，长距离飞行记录间隔长。500米方形转圈可能3秒比较合适。
    //uint32_t log_gap_time_ms=2000;  //记录间隔，毫秒。
#define log_gap_time_ms (2000)  //记录间隔，毫秒。

    uint32_t now = get_time_mark();
    uint32_t last_log_tmark=now; //本地信息记录。

    Send_remote_message(24);

    on_remote=false;

    while(!macro_exec.empty_task()) 
    {
        bool finish;
        macro_exec.do_front_task(finish);
        if(finish) {
            logmessage("one task finish.\n");
            Send_remote_message(42);
        }

        now=get_time_mark();

#ifdef USE_REMOTE_CTRL
        //检查遥控器接管。
        //自动任务采用轮询遥控数据的方式监控接管。
        // if(now - last_remote_info.tmark < 1000 && 
        // (last_remote_info.pos1!=0||last_remote_info.pos2!=0||last_remote_info.pos3!=0||last_remote_info.pos4!=0))
        //摇杆的接管必须打杆比较大，以避免摇杆漂移带来的非主动接管问题。摇杆漂移不易察觉，容易造成误操作。1225飞丢的机器大概率
        //就是摇杆低温下漂移造成，20231227修改。
        //20240102 取消偏航角摇杆的接管，只保留高度和方向角度的接管

        //20240712 现在修改为预定义接管命令接管，摇杆不接管，因摇杆稳定性差。
        // if(now - last_remote_info.tmark < 1000 && 
        // (abs(last_remote_info.pos1)>8 || abs(last_remote_info.pos2)>8 || abs(last_remote_info.pos4)>8))
        // {
        //     //通知遥控器接管了。
        //     Send_remote_message(31);
        //     logmessage("remote takeover\n");
        //     break;
        // }

        //预定义命令32号是接管命令。
        if(last_predefined_task!=0)
        {
            uint8_t cmd= last_predefined_task;
            last_predefined_task=0;

            if(cmd== _predefined_cmd_takeover)
            {
                last_task_content.clear(); //在自动飞行期间可能有其他任务发送过来会被保存，是无效的任务，这里清理掉
                Send_remote_message(31);
                logmessage("remote takeover\n");
                break;
            }
            else if(cmd==_predefined_cmd_break_and_return_home)
            {
                //当前使用这个命令未必能正常执行，和后面的处理有关，所以暂时不使用。
                //目前先发送接管，后发送"LH"任务是正常操作。
                last_task_content="LH"; //返回起点命令
                remote_task_signal=true; 
                Send_remote_message(55);
                //Send_remote_message("break task and return home");
                break;  
            }
            else if(cmd==_predefined_cmd_ps0)
            {
                power_control_solution_adjust(0);
                logmessage("ps set to 0\n");
                Send_remote_message("power control mode set to 0");
            }
            else if(cmd==_predefined_cmd_ps1)
            {
                power_control_solution_adjust(1);
                logmessage("ps set to 1\n");
                Send_remote_message("power control mode set to 1");
            }
            else if(cmd==_predefined_cmd_ps2)
            {
                power_control_solution_adjust(2);
                logmessage("ps set to 2\n");
                Send_remote_message("power control mode set to 2");
            }
            else if(cmd==_predefined_cmd_ps3)
            {
                power_control_solution_adjust(3);
                logmessage("ps set to 3\n");
                Send_remote_message("power control mode set to 3");
            }
            else if(cmd==_predefined_cmd_ps4)
            {
                power_control_solution_adjust(4);
                logmessage("ps set to 4\n");
                Send_remote_message("power control mode set to 4");
            }
            else if(cmd==_predefined_cmd_ps5)
            {
                power_control_solution_adjust(5);
                logmessage("ps set to 5\n");
                Send_remote_message("power control mode set to 5");
            }
            else if(cmd==_predefined_cmd_ps6)
            {
                power_control_solution_adjust(6);
                logmessage("ps set to 6\n");
                Send_remote_message("power control mode set to 6");
            }
            else
            {
                Send_remote_message("not acceptable predefined task in auto run.");
            }
        }
        

        Send_remote_proper_info();
#endif  //use remote control

#ifdef USE_GPS
        //自动任务阶段的日志记录，间隔较小，便于找问题。
        if(now - last_log_tmark > log_gap_time_ms)
        {
            //记录必要的飞行日志信息。
            //gps位置，含高度速度方向，pitch,roll,yaw
            //电压信息，电机信息。
            
            gps_info a;
            critical_section_enter_blocking(&gps_queue_section);
            a=gps_queue.last();
            critical_section_exit(&gps_queue_section);
            //a.gpstime

            float roll,pitch,yaw,temp0;
            critical_section_enter_blocking(&imu_queue_section);
            pitch= imu_queue.last().angle[0];
            roll= imu_queue.last().angle[1];
            yaw= imu_queue.last().angle[2];
            temp0=imu_queue.last().temprature;
            critical_section_exit(&imu_queue_section);

            float press, temp1;

            critical_section_enter_blocking(&press_queue_section);
            press=press_queue.last().press;
            temp1=press_queue.last().temprature;
            critical_section_exit(&press_queue_section);

            float sumb=motor_ctrl.GetSumBurden();

            char buf[256];
            //增加水平定位精度记录，怀疑快速运动时水平定位精度可能变差，数据变大。
            //20240324,新增气压记录，需要考察为什么平飞时GPS高度波动大，是否是气压波动大造成。
            //20241127,新增温度记录，两个温度同时记录。
            //20250121,末尾新增vspeed记录，用来调试降落卡住的问题。后又删除。
            sprintf(buf, "%7.1f,%3.1f,%f,%f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%3.2f,%6.1f\n", press, temp1,
                a.latitude,a.longitude,a.height,a.speed,a.direction,pitch,roll,yaw,_heading_yaw,system_power_voltage,system_power_current,sumb,system_power_used);
            logmessage(buf);

            last_log_tmark=now;
        }
#endif

        //检查和执行警卫任务。
        if(guard_energy_thres>0.1 && system_power_used > guard_energy_thres*1000) //后者单位是KJ
        {
            //退出所有自动任务，自动返航。

            macro_exec.clear_task();
            macro_exec.addtask_landing_to_corrdinate(initial_gps_info.longitude, initial_gps_info.latitude, initial_gps_info.height);
            //关闭所有警卫，不然会再次反复执行到这里。
            guard_energy_thres=0;
            guard_voltage_thres=0;
            logmessage("power guard trigered\n");
            Send_remote_message("power guard trigered");
        }

        if(guard_voltage_thres>0.1 && system_power_voltage < guard_voltage_thres)
        {
            macro_exec.clear_task();
            macro_exec.addtask_landing_to_corrdinate(initial_gps_info.longitude, initial_gps_info.latitude, initial_gps_info.height);
            //关闭所有警卫，不然会再次反复执行到这里。
            guard_energy_thres=0;
            guard_voltage_thres=0;
            logmessage("voltage guard trigered\n");
            Send_remote_message("voltage guard trigered");
        }

        sleep_ms(50); //这个速度影响自动指南针图的采样密度。
       
    }

    //避免再次进入自动任务时保留能量限制。
    guard_energy_thres=0;
    guard_voltage_thres=0;

//可能是被中断的，清除所有任务。最后的微任务以及状态得以保留。遥控接管需要分辨。
    macro_exec.clear_task();
    Send_remote_message(25);
 
}

void get_current_gps_coordinate(double& lati, double& longi, float& height)
{
    critical_section_enter_blocking(&gps_queue_section);
    lati=gps_queue.last().latitude;
    longi=gps_queue.last().longitude;
    height=gps_queue.last().height;
    critical_section_exit(&gps_queue_section);
}

void get_forward_coordinate(double lati_base, double longi_base, float dist, double& lati, double& longi)
{
    //根据当前的yaw角，配合-heading_yaw计算出机身的指向角，然后计算目标的坐标点。
    float cur_yaw;
    critical_section_enter_blocking(&imu_queue_section);
    cur_yaw=imu_queue.last().angle[2];
    critical_section_exit(&imu_queue_section);
    float head_angle= _heading_yaw - cur_yaw;
    if(head_angle<0) head_angle+=360;
    else if(head_angle>360) head_angle-=360;

    //CGPS_Ublox::gps_by_dist_angle(lati_base, longi_base, dist, head_angle, lati, longi);
    global_dist_angle_to_gps(lati_base, longi_base, dist, head_angle, lati, longi);
}

void get_east_coordinate(double lati_base, double longi_base, float dist, double& lati, double& longi)
{
    //CGPS_Ublox::gps_by_dist_angle(lati_base, longi_base, dist, 90, lati, longi);
    global_dist_angle_to_gps(lati_base, longi_base, dist, 90, lati, longi);
}

void get_west_coordinate(double lati_base, double longi_base, float dist, double& lati, double& longi)
{
    //CGPS_Ublox::gps_by_dist_angle(lati_base, longi_base, dist, 270, lati, longi);
    global_dist_angle_to_gps(lati_base, longi_base, dist, 270, lati, longi);
}

void get_south_coordinate(double lati_base, double longi_base, float dist, double& lati, double& longi)
{
    //CGPS_Ublox::gps_by_dist_angle(lati_base, longi_base, dist, 180, lati, longi);
    global_dist_angle_to_gps(lati_base, longi_base, dist, 180, lati, longi);
}

void get_north_coordinate(double lati_base, double longi_base, float dist, double& lati, double& longi)
{
    //CGPS_Ublox::gps_by_dist_angle(lati_base, longi_base, dist, 0, lati, longi);
    global_dist_angle_to_gps(lati_base, longi_base, dist, 0, lati, longi);
}

//解析任务字符串，建立任务序列。不特定的任务序列从遥控发过来，有别于预定义任务。
//return num of tasks.
//total_route:全程水平飞行距离。
//B,标记基点，R,返回基点并删除一个基点，RB,返回基点，不删除基点
//H,高度调节，A,绝对高度
//M(方向角，距离),以当前位置移动到一个角度和距离上
//B(方向角，距离),以基点为参考移动到某个位置。
//P0-P9，执行平飞到预定义点
//X0-X9,预定义点
//F,向前飞行
//G(经度,纬度,高度)，执行平飞
//RH,返回起飞点不降落
//LH,降落到起飞点
//K,维持位置
int Parse_Task_String(std::string ts, float& total_route)
{
    //任务以分号分割
    //样本 "B;N+500;S+1000;N+1000;R"
    //记录基点，北飞500米，南飞1000米然后北飞1000，返回基点。
    //样本 "B;E+500;S+500;W+500;N+500;R"
    //记录基点，东飞500米，南飞500米，西飞500米，北飞500米，返回基点。
    //样本 "H(100);F+300"
    //高度提高100米，向前飞300米
    //样本 "M(0,300)" 从当前位置飞行到航向角0，距离300米的点。
    //样本 "G(116.222,39.222)", 飞行到绝对GPS坐标点
    //样本 "H(100);L(116.222,39.222,20.0)", 提高100米，降落到绝对坐标点。
    //样本 "M(0,300);L2B"; 从当前位置飞行到航向角0，距离300米的点，然后返回基点并降落。
    //样本 "B;B(0,200);B(144,200);R", 记录基点，移动到基点方位角0，距离200位置，移动到基点方位角144，距离200位置，返回。
    //B移动的位置基于基点位置计算，M基于当前坐标，这是差别。东南西北的位移也是基于当前坐标。
 
    
    int num_tasks=0;
    total_route=0;

    //首先按照分号分割成段落。然后每个段落再分析。
    std::vector<std::string> splits;
    ParseTaskList(ts, splits);

    //依次按顺序解释分段的意思。
    //第一字母B,H,F,E,N,W,S,M,G,R,L,K,数字，分别有不同的含义。
    //20240320 新增一个字符A, 如A1200，代表调节高度到海拔1200米，以gps为准。
    //A后面只接受一个正数。不支持负海拔高度。
    //A,H用来调节高度，之前只在后面跟上数字，现在为了支持在过程中旋转机身，需要多一个参数
    //这样，A,H两个指令需要跟上括号，修改为 A(1200,2), H(-12.1,0) 这种形式
    //第一个参数是高度控制参数，第二个是旋转控制参数。
    //K用来悬停，也是2个参数，K(100,2) 第一个是时间，第二个是旋转。
    //B命令可带括号，也可不带，不同的意思。带括号的是移动到相对基点的位置。
    //L命令可带括号，也可不带，不带就是就地降落，带就是到目标降落
    //G命令带括号，有2个或3个参数

    //20240606, 新增p0-p9为全局定义点，P0(logi,lati,HEIGHT), 带括号为定义，不带为执行。
    //20241003, 新增X0-X9为全局定义点，X0就是把P0定义为当前点。这样可以在重复路线中简化定义。
    //20241008, 尝试增加PG命令，即power-guard，来防止电量耗尽坠机。同时增加电压警卫VG
    //PG(120),在后续任务中持续有效。当电量消耗达到120kj时，终止所有任务返回起点。PG(0)关闭能量守卫
    //VG(12.0),在后续任务中持续有效。在电压低于12时，终止所有任务返回起点。VG(0)关闭电压守卫
    //目前后续任务仅为终止所有任务返航，以后或可扩展。

    double lati, longi; //最新位置, 随动
    float height;

    get_current_gps_coordinate(lati, longi, height);


    double lati_base=lati; //基点位置，由基点命令设置。
    double longi_base=longi;
    //float height_base=height;

    std::stack<std::tuple<double,double>> bases; //坐标基点序列。
    _predefined_location PredefinedLocations[10]; //10个预定义的位置点。


    for(size_t i=0;i<splits.size();i++)
    {
        if(splits[i].empty()) continue; //20240320+ 防止误输入空字符串造成崩溃。后面的判断都需要至少1个字符。

        if(splits[i][0]=='B')
        {
            if(splits[i]=="B") {
                bases.push(std::tuple<double,double>(lati, longi));
                lati_base=lati;
                longi_base=longi;
            }else{
                //B开头带参数，如B(0,200)则为基于基点的（方位角，距离）
                std::vector<float> vals;
                ParseFloatList(splits[i], vals);
                
                if(vals.size()==2 && vals[0]>=0.0 && vals[0]<=360.0 && vals[1]>=0.0)
                {
                    double nlati, nlongi;
                    global_dist_angle_to_gps(lati_base, longi_base, vals[1], vals[0], nlati, nlongi);
                    macro_exec.addtask_pin_to_corrdinate(nlongi, nlati, 3.0);
                    num_tasks++;
                    lati=nlati;
                    longi=nlongi;
                    total_route+= vals[1];
                }
                else{
                    logmessage("wrong B\n");
                    return -1;
                }
            }
        }
        else if(splits[i][0]=='F') {
            //forward
            //20250126修改，F后面要加括号，只接收一个参数，并大于0
            std::vector<float> vals;
            ParseFloatList(splits[i], vals);
            if(vals.size()==1 && vals[0]>0)
            {
                double nlati, nlongi;
                get_forward_coordinate(lati, longi, vals[0], nlati, nlongi);
                macro_exec.addtask_pin_to_corrdinate(nlongi, nlati, 5.0);
                num_tasks++;
                lati=nlati;
                longi=nlongi;
                total_route+= vals[0];
            }
            else
            {
                logmessage("wrong F\n");
                return -1;
            }

            // splits[i].erase(0,1);
            // float dist= atof(splits[i].c_str());
            // if(dist>0) {
            //     double nlati, nlongi;
            //     get_forward_coordinate(lati, longi, dist, nlati, nlongi);
            //     macro_exec.addtask_pin_to_corrdinate(nlongi, nlati, 5.0);
            //     num_tasks++;
            //     lati=nlati;
            //     longi=nlongi;
            //     total_route+= dist;
            // }else{
            //     logmessage("wrong F\n");
            //     return -1;
            // }
        }
        else if(splits[i][0]=='H') {
            //20240320+ 相对高度调节命令。带括号带参数，第一个参数调节高度，第二个旋转速度
            //20250131 修改相对高度调节处理。
            std::vector<float> vals;
            ParseFloatList(splits[i], vals);

            if(vals.size()==1) {
                height+= vals[0];
                macro_exec.addtask_vertical_absolute_height_adjust(height);
                num_tasks++;
            }else if(vals.size()==2) {
                height+= vals[0];
                macro_exec.addtask_vertical_absolute_height_adjust(height, int8_t(vals[1]));
                num_tasks++;
            }
            else{
                logmessage("wrong H\n");
                return -1;
            }
        }
        
        else if(splits[i][0]=='M') {
            //M(0,200) 方向角，距离
            std::vector<float> vals;
            ParseFloatList(splits[i], vals);
            if(vals.size()==2 && vals[0]>=0.0 && vals[0]<=360.0 && vals[1]>=0.0)
            {
                double nlati, nlongi;
                global_dist_angle_to_gps(lati, longi, vals[1], vals[0], nlati, nlongi);
                macro_exec.addtask_pin_to_corrdinate(nlongi, nlati, 5.0);
                num_tasks++;
                lati=nlati;
                longi=nlongi;
                total_route+= vals[1];
            }else{
                logmessage("wrong M\n");
                return -1;
            }
        }
        else if(splits[i][0]=='G') {
            //G(longi, lati) 经度，纬度
            //G(longi, lati, height) 经度,纬度, 高度
            std::vector<float> vals;
            ParseFloatList(splits[i], vals);
            //经度范围西部73.6，东部134.6，纬度范围南部18.1，北部53.5
            //限制经纬度在中国范围内是防止输入的数据搞错经纬度。
            if(vals.size()==2 ) {
                float dist, angle;
                global_gps_dist_angle(lati, longi, vals[1], vals[0],dist, angle);
                macro_exec.addtask_pin_to_corrdinate(vals[0], vals[1], 5.0);
                num_tasks++;
                longi= vals[0];
                lati= vals[1];
                total_route+= dist;
            }
            else if(vals.size()==3 )
            {
                float dist, angle;
                global_gps_dist_angle(lati, longi, vals[1], vals[0], dist, angle);

                total_route+=dist;

                //带高度的位置，需要在水平位移时伴随调节高度。
                if(vals[2]>height) {
                    //目标高度大于当前高度的，先提高高度。
                    macro_exec.addtask_vertical_absolute_height_adjust(vals[2]);
                    num_tasks++;
                    height=vals[2];

                    macro_exec.addtask_pin_to_corrdinate(vals[0], vals[1], 5.0);
                    num_tasks++;
                    longi= vals[0];
                    lati= vals[1];

                }else{
                    //目标高度小于当前高度的，后降低高度。
                    macro_exec.addtask_pin_to_corrdinate(vals[0], vals[1], 5.0);
                    num_tasks++;
                    longi= vals[0];
                    lati= vals[1];

                    macro_exec.addtask_vertical_absolute_height_adjust(vals[2]);
                    num_tasks++;
                    height=vals[2];
                }

            }
            else{
                logmessage("wrong G\n");
                return -1;
            }
        }
        else if(splits[i][0]=='L') {

            if(splits[i]=="L")
            {
                //只有一个L，就地降落。没有参考高度，尽量少用。
                macro_exec.addtask_landing_to_corrdinate(longi, lati, 0, 3.0);
                num_tasks++;
            }
            else if(splits[i]=="LH")
            {
                float dist, angle;
                global_gps_dist_angle(lati, longi, initial_gps_info.latitude, initial_gps_info.longitude, dist, angle);
                total_route+=dist;

                //降落到出发点。
                macro_exec.addtask_landing_to_corrdinate(initial_gps_info.longitude, initial_gps_info.latitude, initial_gps_info.height);
                num_tasks++;
                height= initial_gps_info.height;
            }
            else
            {
                //L(longi,lati,height) 经度，纬度，高度
                std::vector<float> vals;
                ParseFloatList(splits[i], vals);
                //经度范围西部73.6，东部134.6，纬度范围南部18.1，北部53.5
                //限制经纬度在中国范围内是防止输入的数据搞错经纬度。
                if(vals.size()==3) {
                    float dist, angle;
                    global_gps_dist_angle(lati, longi, vals[1], vals[0], dist, angle);
                    macro_exec.addtask_landing_to_corrdinate(vals[0], vals[1], vals[2], 3.0);
                    num_tasks++;
                    total_route+= dist;
                    height=vals[2];
                }
                else
                {
                    logmessage("wrong L\n");
                    return -1;
                }
            }
        }
        else if(splits[i]=="R") {
            //返回基点，并删除基点
            if(!bases.empty())
            {
                double lati_b= std::get<0>(bases.top());
                double longi_b= std::get<1>(bases.top());
                bases.pop();

                float dist, angle;
                global_gps_dist_angle(lati, longi, lati_b, longi_b, dist, angle);
                total_route+= dist;

                macro_exec.addtask_pin_to_corrdinate(longi_b, lati_b, 5.0);
                num_tasks++;
                lati=lati_b;
                longi=longi_b;

            }else{
                logmessage("wrong R, no base\n");
                return -1; //wrong!
            }

        }
        else if(splits[i]=="RB") {
            //返回基点，并保留基点不删除
            if(!bases.empty())
            {
                double lati_b= std::get<0>(bases.top());
                double longi_b= std::get<1>(bases.top());
                
                float dist, angle;
                global_gps_dist_angle(lati, longi, lati_b, longi_b, dist, angle);
                total_route+= dist;

                macro_exec.addtask_pin_to_corrdinate(longi_b, lati_b, 5.0);
                num_tasks++;
                lati=lati_b;
                longi=longi_b;
            }else{
                logmessage("wrong RB, no base\n");
                return -1; //wrong!
            }
        }
        else if(splits[i]=="RH") {
            //return to home. no landing.
            float dist, angle;
            global_gps_dist_angle(lati, longi, initial_gps_info.latitude, initial_gps_info.longitude, dist, angle);
            total_route+= dist;
            macro_exec.addtask_pin_to_corrdinate(initial_gps_info.longitude, initial_gps_info.latitude, 5.0);
            num_tasks++;
            longi= initial_gps_info.longitude;
            lati= initial_gps_info.latitude;
        }
        else if(splits[i][0]=='K') {
            //停留保持。带一个参数，K(100)=保持100秒
            //带2个参数，K(100, 2)=保持100秒且机身旋转速度2，这样方便拍摄环绕景色。
            std::vector<float> vals;
            ParseFloatList(splits[i], vals);
            if(vals.size()==1 && vals[0]>0) {
                //限制最大停留6000秒。
                float stay= vals[0];
                if(stay>5000) stay=5000;
                macro_exec.addtask_keepstable(stay);
                num_tasks++;
            }else if(vals.size()==2 && vals[0]>0) {
                float stay= vals[0];
                if(stay>5000) stay=5000;
                int8_t yawspd= vals[1];
                macro_exec.addtask_keepstable(stay, yawspd);
                num_tasks++;
            }
            else
            {
                logmessage("wrong K\n");
                return -1;
            }
        }
        else if(splits[i][0]=='A') {
            //20240320+ 海拔高度调节命令。带括号带参数，第一个参数海拔高度，第二个旋转速度
            std::vector<float> vals;
            ParseFloatList(splits[i], vals);

            if(vals.size()==1 && vals[0]>0) {
                macro_exec.addtask_vertical_absolute_height_adjust(vals[0]);
                height= vals[0];
                num_tasks++;
            }else if(vals.size()==2 && vals[0]>0) {
                macro_exec.addtask_vertical_absolute_height_adjust(vals[0], int8_t(vals[1]));
                height= vals[0];
                num_tasks++;
            }else{
                logmessage("wrong A\n");
                return -1;
            }
        }
        else if(splits[i].size()>2 && splits[i][0]=='P' && splits[i][1]=='G') {
            //power guard!
            //PG(float)
            std::vector<float> vals;
            ParseFloatList(splits[i], vals);
            if(vals.size()==1) {
                macro_exec.add_enegy_guard(vals[0]);
            }else{
                //目前只接受一个参数。
                logmessage("wrong PG\n");
                return -1;
            }
        }
        else if(splits[i].size()>2 && splits[i][0]=='V' && splits[i][1]=='G') {
            //voltage guard!
            //VG(float)
            std::vector<float> vals;
            ParseFloatList(splits[i], vals);
            if(vals.size()==1) {
                macro_exec.add_voltage_guard(vals[0]);
            }else{
                //目前只接受一个参数。
                logmessage("wrong VG\n");
                return -1;
            }
        }
        else if(splits[i][0]=='P') //这个必须放在PG后面判断，否则PG无法被执行。
        {
            //P0-P9序列
            if(splits[i].size()<2) {
                logmessage("wrong P\n");
                return -1;
            }

            uint8_t idx;
            switch (splits[i][1])
            {
            case '0':
                idx=0;
                break;
            case '1':
                idx=1;
                break;
            case '2':
                idx=2;
                break;
            case '3':
                idx=3;
                break;
            case '4':
                idx=4;
                break;
            case '5':
                idx=5;
                break;
            case '6':
                idx=6;
                break;
            case '7':
                idx=7;
                break;
            case '8':
                idx=8;
                break;
            case '9':
                idx=9;
                break;
            default:
                {
                    logmessage("wrong P\n");
                    return -1;
                }
            }//switch

            std::vector<float> vals;
            ParseFloatList(splits[i], vals);
            if(vals.size()==0)
            {
                //这是一个执行任务。
                if(!PredefinedLocations[idx].defined) {
                    //未定义该点。
                    logmessage("wrong p\n");
                    return -1;
                }

                float dist, angle;
                global_gps_dist_angle(lati, longi, PredefinedLocations[idx].lati, PredefinedLocations[idx].logi, dist, angle);
                total_route+= dist;

                if(PredefinedLocations[idx].height>0) {
                    //该点定义了高度。
                    if(PredefinedLocations[idx].height>height) {
                        //目标高度大于当前高度的，先提高高度。
                        macro_exec.addtask_vertical_absolute_height_adjust(PredefinedLocations[idx].height);
                        height=PredefinedLocations[idx].height;
                        macro_exec.addtask_pin_to_corrdinate(PredefinedLocations[idx].logi, PredefinedLocations[idx].lati, 3.0);
                     }else{
                        //目标高度小于当前高度的，后降低高度。
                        macro_exec.addtask_pin_to_corrdinate(PredefinedLocations[idx].logi, PredefinedLocations[idx].lati, 3.0);
                        macro_exec.addtask_vertical_absolute_height_adjust(PredefinedLocations[idx].height);
                        height=PredefinedLocations[idx].height;
                    }

                    num_tasks++;
                    longi= PredefinedLocations[idx].logi;
                    lati= PredefinedLocations[idx].lati;

                }else{
                    //该点未定义高度。
                    macro_exec.addtask_pin_to_corrdinate(PredefinedLocations[idx].logi, PredefinedLocations[idx].lati, 3.0);
                    num_tasks++;
                    longi= PredefinedLocations[idx].logi;
                    lati= PredefinedLocations[idx].lati;
                }
            }
            else if(vals.size()==2)
            {
                //这个是定义点。
                PredefinedLocations[idx].logi=vals[0];
                PredefinedLocations[idx].lati=vals[1];
                PredefinedLocations[idx].height=0;
                PredefinedLocations[idx].defined=true;
            }
            else if(vals.size()==3)
            {
                //这个是定义点。
                PredefinedLocations[idx].logi=vals[0];
                PredefinedLocations[idx].lati=vals[1];
                PredefinedLocations[idx].height=vals[2];
                PredefinedLocations[idx].defined=true;
            }
            else 
            {
                logmessage("wrong P\n");
                return -1;
            }
            
        }
        else if(splits[i][0]=='X')
        {
            //X0-X9序列
            if(splits[i].size()<2) {
                logmessage("wrong X\n");
                return -1;
            }

            uint8_t idx;
            switch (splits[i][1])
            {
            case '0':
                idx=0;
                break;
            case '1':
                idx=1;
                break;
            case '2':
                idx=2;
                break;
            case '3':
                idx=3;
                break;
            case '4':
                idx=4;
                break;
            case '5':
                idx=5;
                break;
            case '6':
                idx=6;
                break;
            case '7':
                idx=7;
                break;
            case '8':
                idx=8;
                break;
            case '9':
                idx=9;
                break;
            default:
                {
                    logmessage("wrong X\n");
                    return -1;
                }
            }//switch

            PredefinedLocations[idx].logi=longi;
            PredefinedLocations[idx].lati=lati;
            PredefinedLocations[idx].height=0; //不定义高度，仅水平运动，不调节高度
            PredefinedLocations[idx].defined=true;
        }
        
    }

    return num_tasks;
}


//这里面同时监控是否有陀螺仪矫正的遥控信号，如果有则执行。这样可以利用GPS启动时间。
void basic_gps_check()
{
    uint32_t last_wait_tip=0;

#if defined USE_GPS
    while(1) {
        gps_info a;
        critical_section_enter_blocking(&gps_queue_section);
        a=gps_queue.last();
        critical_section_exit(&gps_queue_section);
        uint32_t now = get_time_mark();
        if(a.tmark==0)
        {
            //数据不满足要求，很可能是GPS等待。回传GPS数据。
            if(now - last_wait_tip > 20000) {
                Send_remote_message(30);
                last_wait_tip=now;
            }
        }
        else if(a.hacc < 8.0 && a.ns >= 5) { //for debug 10.0, for run 2.0
            break;
        }
        
        Send_remote_proper_info();

        sleep_ms(20); //休眠较短，以便捕捉遥控器信号。

        //检查是不是imu校准命令。yaw摇杆左打或右打，按钮1按下。
        #if defined USE_REMOTE_CTRL
        if(last_predefined_task!=0)
        {
            uint8_t cmd= last_predefined_task;
            last_predefined_task=0;

                if(cmd==_predefined_cmd_imu_calib)
                {
                    //imu出场校准
                    Send_remote_message("calib comd accept");
                    Do_imu_calibration();
                }
                else if(cmd==_predefined_cmd_user_imu_calib)
                {
                    //imu用户校准
                    Send_remote_message("user calib accept");
                    Do_user_calibration();
                }
                else if(cmd==_predefined_cmd_compass_calib)
                {
                    //指南针校准
                    Send_remote_message("compass calib accept");
                    Do_magnet_calibration();
                }
                else if(cmd==_predefined_cmd_check_motor)
                {
                    Send_remote_message("check motor accept");
                    basic_motor_check();
                }
                else if(cmd==_predefined_cmd_force_stable_baro_ctrl)
                {
                    _force_stable_baro_height_control=true;
                    Send_remote_message("force stable baro ctrl");
                }
                else if(cmd==45)
                {
                    Send_remote_message("about to test motor by remote");
                    //burn_motor_test();
                    motor_test_by_remote();
                }
                else if(cmd==_predifined_cmd_check_tof)
                {
                    Send_remote_message("check tof module.");
                    Check_tof_data();
                }
                else if(cmd==_predefined_cmd_ps0)
                {
                    power_control_solution_adjust(0);
                    logmessage("ps set to 0\n");
                    Send_remote_message("power control mode set to 0");
                }
                else if(cmd==_predefined_cmd_ps1)
                {
                    power_control_solution_adjust(1);
                    logmessage("ps set to 1\n");
                    Send_remote_message("power control mode set to 1");
                }
                else if(cmd==_predefined_cmd_ps2)
                {
                    power_control_solution_adjust(2);
                    logmessage("ps set to 2\n");
                    Send_remote_message("power control mode set to 2");
                }
                else if(cmd==_predefined_cmd_ps3)
                {
                    power_control_solution_adjust(3);
                    logmessage("ps set to 3\n");
                    Send_remote_message("power control mode set to 3");
                }
                else if(cmd==_predefined_cmd_ps4)
                {
                    power_control_solution_adjust(4);
                    logmessage("ps set to 4\n");
                    Send_remote_message("power control mode set to 4");
                }
                else if(cmd==_predefined_cmd_ps5)
                {
                    power_control_solution_adjust(5);
                    logmessage("ps set to 5\n");
                    Send_remote_message("power control mode set to 5");
                }
                else if(cmd==_predefined_cmd_ps6)
                {
                    power_control_solution_adjust(6);
                    logmessage("ps set to 6\n");
                    Send_remote_message("power control mode set to 6");
                }
                else if(cmd==_predefined_cmd_output_log)
                {
                    //输出日志信息。
                    Send_remote_message("output log");
                    read_log_from_flash();
                }
                else{
                    char dbg[64];
                    sprintf(dbg, "can't accept predefined %d cmd\n", cmd);
                    Send_remote_message(dbg);
                }

        }
        else if(remote_task_signal)
        {
            //在这个时候并不解析任务，但为了调试问题，这里解析。
            remote_task_signal=false;
            //printf("t=%s\n", last_task_content.c_str());
            Send_remote_message("got task:");
            Send_remote_message(last_task_content.c_str());
            float total_route;
            int n= Parse_Task_String(last_task_content, total_route);
            //printf("parse got %d tasks\n", n);
            if(n>0) {
                char buf[128];
                sprintf(buf, "parse got %d tasks, route %5.1f", n, total_route);
                Send_remote_message(buf);
            }
            else{
                macro_exec.clear_task();
                Send_remote_message("parse cmd failed");
            }
            last_task_content.clear();
        }
        #endif
    }


#endif

}

//宏观任务控制循环，跑在主核心上。
static void macro_task_exec()
{

    float now=get_time_mark();

    icm426xx.ResetYaw(); //重置yaw角度为0

    //暂时未区分两个指南针，目前没有同时打开两个指南针。
    if(_initial_heading_update_method==head_compass0)
    {
        Update_Heading_Yaw(0);
    }
    else if(_initial_heading_update_method==head_compass1)
    {
        Update_Heading_Yaw(0);
    }
    else if(_initial_heading_update_method==head_north)
    {
        _heading_yaw=0;
    }
    else if(_initial_heading_update_method==head_south)
    {
        _heading_yaw=179.9;
    }
    else if(_initial_heading_update_method==head_east)
    {
        _heading_yaw=90.0;
    }
    else if(_initial_heading_update_method==head_west)
    {
        _heading_yaw=-90.0;
    }
    else if(_initial_heading_update_method==head_east_south)
    {
        _heading_yaw=135.0;
    }
    else if(_initial_heading_update_method==head_east_north)
    {
        _heading_yaw=45.0;
    }
    else if(_initial_heading_update_method==head_west_north)
    {
        _heading_yaw=-45.0;
    }
    else if(_initial_heading_update_method==head_west_south)
    {
        _heading_yaw=-135.0;
    }
    else if(_initial_heading_update_method==head_none)
    {
        //盲目起飞，方向固定为北。
        _heading_yaw=0.0;
    }


    char buf[64];
    sprintf(buf, "heading yaw=%f", _heading_yaw);
    Send_remote_message(buf);

    //记录间隔时间，长距离飞行记录间隔长。500米方形转圈可能3秒比较合适。
    //uint32_t log_gap_time_ms=3000; 

    //uint32_t last_log_tmark=now; //本地信息记录。

    //on_remote=true;

    macro_exec.addtask_ground_up();

    //全自动任务安排放在这里，运行全自动任务首先要取消最初等待遥控发出起飞命令。
#if defined FULL_AUTO_RUN
    on_remote=false;
    macro_exec.addtask_vertical_height_adjust(160); //升高到净空
    //macro_exec.addtask_pin_to_corrdinate();//去往某地。
    //macro_exec.addtask_pin_to_corrdinate();//去往某地。
    //macro_exec.addtask_pin_to_corrdinate();//去往某地。
    //macro_exec.addtask_landing_to_corrdinate();//在某地降落。至此结束自动运行。
#endif


nosleep:

    auto_task_exec();

#ifdef FULL_AUTO_RUN
    //如果完全自动运行，且此时已经降落，退出系统停机。
    if(!_inair) {
        logmessage("quit sys\n");
        Send_remote_message("quit sys");
        micro_task.taskid=_micro_task_id_nop;
        return;
    }

#endif

#ifdef USE_REMOTE_CTRL
//接下来是遥控器控制代码。

    //这个遥控控制状态里，即便是飞机没有起飞在地面，也可以通过手工推高来起飞。
    //不过目前，遥控推高起飞时，没有设置为在空中的状态。可能会使一键降落无法执行。
    //要使手工推高或推低实现起飞和降落，可以设置在推低时允许低于某个零界点直接
    //关闭电机实现停机。目前设置了最低的电机推力以防止在空中关机。防止在空中
    //追求一个很低的高度设定时，电机的推力过低关机坠落。这种限制目前是合理的，
    //但在降落时会带来无法关机的问题。可以考虑搭配一个按钮并推低油门解除推力最小限制。

    //20240208新发现，这里的处理会使得自动降落失败。需要区分是否是自动降落任务。
    // if(micro_task.taskid==_micro_task_id_vertical_landing_location && micro_task.status== _micro_task_status_done && _inair==false)
    // {
    //     //这是自动任务的正常退出，那么，_inair标记有效。如果_inair==false, 那么已经降落。
    //     goto quit;
    // }

    //分析从自动任务里退出的原因，有些需要遥控接管，有些就真的退出了。
    

    
    now=get_time_mark(); //经过了自动任务，now已经变化很大了，需要更新。
  

    if(macro_exec._running_task_id == _macro_ground_up)
    {
        if(macro_exec._running_task_status==_macro_task_running && !_inair)
        {
            //正在进行起飞途中。
            _inair=true;
            micro_task.help_takeoff=false; //20231215+ 在起飞时接管，可能这个标记还在，清除。
            logmessage("groundup takeover not inair, mark inair\n");
        }
        else if(macro_exec._running_task_status==_macro_task_failed) 
        {
            //起飞失败。
            logmessage("groundup failed, quit\n");
            goto quit;
        }
    }
    else if(macro_exec._running_task_id == _macro_landing_coordinate)
    {
        //降落任务标记完成状态。
        if(macro_exec._running_task_status == _macro_task_succeed)
        {
            logmessage("landing succeed, quit\n");
            Send_remote_message("landing succeed,quit");
            if(_inair) logmessage("oops! landing succeed but inair?\n");
            goto quit;
        }
        else if(macro_exec._running_task_status == _macro_task_running)
        {
            //任务被途中终止，取消可能的锚定标记。
            micro_task.gps_anchor=false;
            logmessage("landing target canceled, mark unanchor\n");
        }
        else
        {
            micro_task.gps_anchor=false;
            logmessage("landing task quit in other status\n");
        }
    }
    else if(macro_exec._running_task_id == _macro_landing_uncondition)
    {
        if(macro_exec._running_task_status == _macro_task_succeed)
        {
            //降落任务标记完成状态。
            logmessage("landing succeed, quit\n");
            Send_remote_message("landing succeed,quit");
            if(_inair) logmessage("oops! landing succeed but inair?\n");
            goto quit;
        }
        else
        {
            micro_task.gps_anchor=false;
            logmessage("landing task quit in other status\n");
        }
    }
    else if(macro_exec._running_task_id == _macro_pin_to_coordinate)
    {
        //任务完成了。目标点坐标保留，辅助后面的on_remote控制锚定目标点。pin_to_location并不使用这个锚定标记。
        //如果不辅助锚定而进入遥控飞行，则会导致刹车后再定点，
        //之前不标记这个锚定标记时，曾有两次坠机发生在这个时间点，原因不明，没有日志记录。像是小概率事件。
        //需要检查过渡代码。
        //现在首次调用pin_to_location 也会标记锚定做到兼容。
        if(macro_exec._running_task_status == _macro_task_succeed)
        {
            micro_task.gps_anchor=true; //维持盯住之前的坐标点。
            logmessage("pintoloc succeed, turn to remote control, mark anchor\n");
        }
        else if(macro_exec._running_task_status == _macro_task_running)
        {
            micro_task.gps_anchor=false; //现在需要反标记，否则接管后还会在水平方向移动到目标点。
            logmessage("pintoloc running, turn to remote control, mark unanchor\n");
        }
        else
        {
            micro_task.gps_anchor=false;
        }
    }
    else
    {
        micro_task.gps_anchor=false; //其他任务的退出，放弃锚点，遥控接管后重新刹车锚定。
        logmessage("other autotask quit, turn to remote control, mark unanchor\n");
    }

    //进入遥控接管
    micro_task.status = _micro_task_status_none; //20240413 注释，即便是这个状态被微任务的标记所覆盖，现在也不会影响遥控器过程
    micro_task.taskid = _micro_task_id_on_remote; //遥控接管。
    on_remote=true;

    Send_remote_status(_inair, micro_task.gps_bad_mark, on_remote, (now - last_remote_info.tmark)/100); //20240121修改


    while(1)
    {

        bool b=sem_acquire_timeout_ms(&remote_info_to_action, 200);
        if(b)  
        {
            //获得了新的遥控信号。
            sem_reset(&remote_info_to_action, 0); //清理信号标记

            //检查是不是任务信号，预定义或是普通任务。
            //如果是任务，要清除任务标记。
            if(last_predefined_task && _inair)
            {
                uint8_t cmd= last_predefined_task;
                last_predefined_task=0;

                if(cmd==_predefined_cmd_landing_home ) //20240814 修改为16号命令
                {
                    //LH命令，降落到起飞点
                    Send_remote_message(44);
                    logmessage("do landing home on command\n");
                    macro_exec.addtask_landing_to_corrdinate(initial_gps_info.longitude, initial_gps_info.latitude, initial_gps_info.height);
                    goto nosleep;
                }
                else if(cmd==_predefined_cmd_return_home)
                {
                    //RH 不降落返回起飞点
                    Send_remote_message(44);
                    logmessage("do auto return on command\n");
                    macro_exec.addtask_pin_to_corrdinate(initial_gps_info.longitude, initial_gps_info.latitude, initial_gps_info.height);
                    goto nosleep;
                }
                else if(cmd==_predefined_cmd_force_stable_baro_ctrl)
                {
                    _force_stable_baro_height_control=true;
                    Send_remote_message("force stable baro ctrl");
                }
                else if(cmd==_predefined_cmd_ps0)
                {
                    power_control_solution_adjust(0);
                    logmessage("ps set to 0\n");
                    Send_remote_message("power control mode set to 0");
                }
                else if(cmd==_predefined_cmd_ps1)
                {
                    power_control_solution_adjust(1);
                    logmessage("ps set to 1\n");
                    Send_remote_message("power control mode set to 1");
                }
                else if(cmd==_predefined_cmd_ps2)
                {
                    power_control_solution_adjust(2);
                    logmessage("ps set to 2\n");
                    Send_remote_message("power control mode set to 2");
                }
                else if(cmd==_predefined_cmd_ps3)
                {
                    power_control_solution_adjust(3);
                    logmessage("ps set to 3\n");
                    Send_remote_message("power control mode set to 3");
                }
                else if(cmd==_predefined_cmd_ps4)
                {
                    power_control_solution_adjust(4);
                    logmessage("ps set to 4\n");
                    Send_remote_message("power control mode set to 4");
                }
                else if(cmd==_predefined_cmd_ps5)
                {
                    power_control_solution_adjust(5);
                    logmessage("ps set to 5\n");
                    Send_remote_message("power control mode set to 5");
                }
                else if(cmd==_predefined_cmd_ps6)
                {
                    power_control_solution_adjust(6);
                    logmessage("ps set to 6\n");
                    Send_remote_message("power control mode set to 6");
                }
                else if(cmd==_predefined_cmd_takeover)
                {
                    //任务接管，在这里已经是处于遥控控制下了。不需要其他操作。
                    Send_remote_message(31);
                }
                else
                {
                    Send_remote_message("can't do predefined cmd in air");
                }

            }
            else if(remote_task_signal) //这里还有一类是任务序列，
            {
                //标记了预定义任务，执行任务，清除标记。
                remote_task_signal=false;
                //last_task_content;
                float total_route;
                int n=Parse_Task_String(last_task_content, total_route);
                last_task_content.clear();
                if(n>0) {
                    //Send_remote_message("do remote tasks");
                    char buf[128];
                    sprintf(buf, "got %d tasks, route %5.1f", n, total_route);
                    Send_remote_message(buf);
                    goto nosleep;
                }else{
                    macro_exec.clear_task(); //20240602+
                    Send_remote_message("parse cmd failed");
                }

            }
            else
            {

                //新设计使用2个摇杆，设计方向有修改，对应关系有修改，26=pitch, 27=roll, 28=height, 29=yaw
                //关于摇杆的数据顺序，新设计下双方重新约定顺序为 height, yaw, pitch, roll. 不以某一端为主。需要飞控端也做同样修改。
                //方向为 height<0 降低高度，反之升高
                //yaw<0 机器顺时针旋转
                //pitch<0 机器前倾低头 >0 机器抬头
                //roll<0 机器左侧低右侧高

                int8_t yspd= last_remote_info.pos2/4; //保持范围在【-16，16】
                micro_task.yawspeed = yspd;


                float rol= float(last_remote_info.pos4)/3.0; //最大角度放松到21.3度。
                micro_task.roll= rol;

                float pit= float(last_remote_info.pos3)/3.0;
                micro_task.pitch= pit;

                //因遥控更改，现在有约定，pos1是高度控制，>0升高
                int8_t vs= -last_remote_info.pos1; //这里取反
                //范围检查防止越界。
                if(vs>64) vs=64;
                else if(vs<-64) vs=-64;
                //刹车标记。
                if(vs!=0) {
                    micro_task.height_break_tmark=0;
                }
                else if(micro_task.vspeed!=0) {
                    micro_task.height_break_tmark=now;
                }
                
                micro_task.vspeed=vs;


                //检查是否已经降落。
                if(micro_task.taskid==_micro_task_id_on_remote && micro_task.status== _micro_task_status_shutdown)
                {
                    Send_remote_message("landed detected.quit.");
                    Send_remote_status(false, false, on_remote, (now - last_remote_info.tmark)/100); //已经落地就不管gps_bad_mark了。
                    gps_info a;
                    critical_section_enter_blocking(&gps_queue_section);
                    a=gps_queue.last();
                    critical_section_exit(&gps_queue_section);
                    Send_remote_gpsinfo(a);
                    //全部电量消耗。20231112+
                    char dbg[128];
                    sprintf(dbg, "total power used %6.1f w", system_power_used);
                    Send_remote_message(dbg);
                    break; //quit task.
                }

            }


        }else{
            //超时退出。不用考虑执行新的遥控指令。但需要观察是否丢失连接。

            if(_inair)
            {
                //遥控信号丢失，在空中才检查，不在空中不检查。
                uint32_t now=get_time_mark();
                if(now - last_remote_info.tmark > 1000)
                {
                    //遥控断连1秒，清理所有摇杆信息，防止之前的打杆持续起作用。
                    //20240416 增加micro_task.taskid== _micro_task_id_on_remote判断
                    if(micro_task.vspeed!=0 && micro_task.taskid== _micro_task_id_on_remote) { //20240223+
                        micro_task.height_break_tmark=now;
                    }

                    micro_task.vspeed=0;
                    micro_task.pitch=0;
                    micro_task.roll=0;
                    micro_task.yawspeed=0;
                }

                if(now - last_remote_info.tmark > 5000)
                {
                    //remote signal timeout, lost remote control, wait, or return to start point if have gps.
    #if defined USE_GPS
                //如果已起飞并在空中，才可以执行这个任务。
                    Send_remote_message(43);
                    macro_exec.addtask_landing_to_corrdinate(initial_gps_info.longitude, initial_gps_info.latitude, initial_gps_info.height, 3.0);
                    goto nosleep;
    #endif 
                }
            }
        }

        //在循环空闲时，读取GPS数据/姿态数据给遥控器。
        //反馈遥控的信息可以按照数据的变化去发送，比如推高高度，则高度数据就及时高频发送。

        //修改为统一的信息发送。
        Send_remote_proper_info();

    }
#endif

quit:

    logmessage("quit macro_task_exec.\n");
    Send_remote_message("quit macro_task_exec");
    micro_task.taskid=_micro_task_id_nop;
    micro_task.status= _micro_task_status_none;
    Send_remote_proper_info(); //主要是更新inair
}




#if defined USE_TOF
//20230705 发现先前对tof_valid, oflow_valid可能理解有误，这里无效不代表测量数据问题，而是它两个数据分开传递
static void on_pio1_tof_rx()
{

#if defined USE_OFLOW
    //目前的处理方式下，机身静止，左侧抬高，右侧压低时，oflowx读数变小，趋于负值，反之增大，趋于正值，调教好了应该是都为0.

    //左高右低时，地面看起来是向右移动的。此时正常x输出负值，说明陀螺仪的补偿不够大。希望补偿到0

    //如果tof数据连续不间断的送进来，这里可能会卡死整个系统。

    char c;
 
    //初始化时，指定了sm=0
    while(uart_rx_program_getc(pio1, 0, &c))
    {

        
        tof_oflow_info info; //读取的数据是无修正的
        bool b= tof_oflow.Pio_PutChar(c, info);
        if(b) {
            //目前光流的安装方向是，机器向前y为负数，向后y为正数。
            //向左x为负值，向右x为正值。即x轴和系统相同，y轴和系统相反。
            //即地面相对机身向左侧移动时，x为正。gyroy>0时，地面相对机身是向右移动
            //此时oflowx为负值，所以oflowx和gyroy是相加关系。

            //地面相对机器向后移动时，oflowy为负数，gyrox>0时，oflowy为负，所以oflowy和gyrox也是相加关系。

            float gyrox=0; //度/秒 影响光流Y方向读数
            float gyroy=0; //度/秒 影响光流X方向读数
            float gyroz=0;
            float zcos=0;
            float angle_roll=0;
            float angle_pitch=0;
            float yaw=0;
            //取一小段平均也许较好点。现在会摇晃。
            //调节这个前推值，使得就地摇晃的速度接近于0.
            critical_section_enter_blocking(&imu_queue_section);
            yaw= imu_queue.last(3).angle[2];
            //可能有时差
            for(size_t i=4; i<7; i++) 
            {
                imu_info& ref= imu_queue.last(i);
                gyrox += ref.gyro[0];
                gyroy += ref.gyro[1];
                gyroz += ref.gyro[2];
                angle_pitch += ref.angle[0];
                angle_roll += ref.angle[1];
                zcos+= ref.zaxis;
            }
            critical_section_exit(&imu_queue_section);

            //求平均，转弧度。
            gyrox/=3*57.2958;
            gyroy/=3*57.2958;
            gyroz/=3*57.2958;
            angle_pitch/=3*57.2958;
            angle_roll/=3*57.2958;
            zcos/=3; //这个不是角度

            float cospitch= cos(angle_pitch);
            float cosroll= cos(angle_roll);

            //zcos是换为对地面的水平角速度（弧度）。要换为对地水平速度还需要乘以高度。
            
            //画图推算，应该是这样修正数据。陀螺仪的速度和zcos的平方成反比。
            //旋转补偿。
            info.fixed_oflow_spdx +=  gyroy/cosroll; //也许这个对 20230617测试表现摆动较小。
            info.fixed_oflow_spdy +=  gyrox/cospitch;
 
            //假如是偏心安装，yaw轴旋转会影响光流数据，也需要矫正。
            //目前装在Y轴上，yaw轴旋转应该影响x方向的读数。如果旋转不是很快，应该影响不大。
            //gyroz如果大于0，目前光流安装在头部的情况下，光流会认为是机身在向左漂移(光流输出负值)。需要修正的是oflow_spdx
            //info.fixed_oflow_spdx += gyroz * 0.15; //光流装在正前方0.15的距离。似乎影响不大

            //在进行了旋转调节后，把光流的y向速度反向。这样机器向前数据为正，向右也为正，方便点。
            //这个光流模块不可能通过旋转把光流方向调节为符合机身的坐标系。
            info.fixed_oflow_spdy = - info.fixed_oflow_spdy; //20230701 修改，后面光流处理也要改。

            //新增数据以记录当前角度。
            info.yaw=yaw;

            if(info.tof_valid) {
                //用于求时差
                //uint32_t last_tmark= tof_oflow_queue.last().tmark;
                //既然都修正了oflow，顺便修正一下距离，方便后期使用
                //目前这种方法在侧倾时，测得的距离偏小了。原因不清楚。暂时不动。
                info.fixed_tof_distance *= zcos;

                //测距滤波，必须保证上次测距有效 20231112+
                if(tof_oflow_queue.last().tof_valid) {
                    info.fixed_tof_distance = (tof_oflow_queue.last().fixed_tof_distance*7.0 + info.fixed_tof_distance)/8.0f;
                }

                //这是机身坐标下的累计偏量。
                if(info.oflow_valid)
                {
                    //光流有效则累计偏移
                    info.fixed_oflow_spdx *= info.fixed_tof_distance; //光流高度调节
                    info.fixed_oflow_spdy *= info.fixed_tof_distance;
                    info.sumx = tof_oflow_queue.last().sumx + info.fixed_oflow_spdx*0.01;
                    info.sumy = tof_oflow_queue.last().sumy + info.fixed_oflow_spdy*0.01;
                }
                else
                {
                    //如果没有测距距离，光流数据*8.0，即当它高度是8米，再高都是8米，不清零，这样或许高空也可以用到光流做参考定位。
                    //但这样处理，如果在低空位置且测距无效，会放大光流的偏离。
                    info.fixed_oflow_spdx=0;
                    info.fixed_oflow_spdy=0;
                }

            }
            else{
                //测距无效，一律按8米计算。这样或许高空也可以利用光流定点。

                if(info.oflow_valid)
                {
                    //测距无效光流有效时，应乘以最后一个有效高度，但这高度可能早就丢失，所以乘一个固定高度调节
                    //多数时候可能已经在高空，测距无效。时间上按说应该乘以光流的数据间隔，这里固定为0.01秒
                    //当然可以参考GPS，但目前高空不太用光流定点。简化处理。
                    info.fixed_oflow_spdx *= 8.0; //光流高度调节
                    info.fixed_oflow_spdy *= 8.0;
                    info.sumx = tof_oflow_queue.last().sumx + info.fixed_oflow_spdx*0.01;
                    info.sumy = tof_oflow_queue.last().sumy + info.fixed_oflow_spdy*0.01;                    
                }
            }

            critical_section_enter_blocking(&tof_oflow_queue_section);
            tof_oflow_queue.push(info);
            critical_section_exit(&tof_oflow_queue_section);
        }
    }

#else
    //未定义光流模块，简化为只处理测距数据。
    char c;
 
    //初始化时，指定了sm=0
    while(uart_rx_program_getc(pio1, 0, &c))
    {
        
        tof_oflow_info info; //读取的数据是无修正的
        bool b= tof_oflow.Pio_PutChar(c, info);
        if(b) 
        {

            if(info.tof_valid) {

                float zcos=0;
                critical_section_enter_blocking(&imu_queue_section);
                zcos= imu_queue.last().zaxis;
                critical_section_exit(&imu_queue_section);

                info.fixed_tof_distance= info.tof_distance * zcos;

                critical_section_enter_blocking(&tof_oflow_queue_section);
                tof_oflow_queue.push(info);
                critical_section_exit(&tof_oflow_queue_section);
            }
            else
            {
                //测距无效，不用读取Imu数据
                critical_section_enter_blocking(&tof_oflow_queue_section);
                tof_oflow_queue.push(info);
                critical_section_exit(&tof_oflow_queue_section);
            }

        }
    }
#endif    
}
#endif


//core0中断，接收GPS信号，只管存储数据到位
#if defined USE_GPS

#if defined USE_GPS_UART
static void on_gps_rx()
{

    gps_info gi;
    bool b=uart_gps.Read(gi);
    if(b) {

        //获得了有效的gps信息,且数据完整。
        //这里可以加入适度的滤波，主要是定位坐标滤波，速度滤波。
        critical_section_enter_blocking(&gps_queue_section);
        uint32_t t= gi.tmark - gps_queue.last().tmark; //时间差。
        total_horizon_move+= float(t)*gi.speed/1000.0; //水平移动距离积分
        gps_queue.push(gi);
        critical_section_exit(&gps_queue_section);
        //sem_release(&gps_data_to_action);

    }

}
#endif

#if defined USE_GPS_PIO
static void on_pio1_gps_rx()
{

    //很有可能pio在接收数据时偶发乱码。
    char c;
    //初始化时，指定了sm=1
    while(uart_rx_program_getc(pio1, 1, &c))
    {
        if(c>127) printf("possible bad char %02X\n", c);

        gps_info gi;
        bool b= uart_gps.Pio_PutChar(c, gi);
        if(b) {
            gi.tmark=get_time_mark();
            critical_section_enter_blocking(&gps_queue_section);
            gps_queue.push(gi);
            critical_section_exit(&gps_queue_section);
            
            sem_release(&gps_data_to_action);
        }
    }
}
#endif

#endif


#if defined USE_REMOTE_CTRL
static void on_remote_ctrl_rx_tx()
{
    //printf("receive\n");
    //io_ro_32 ris= uart_get_hw(uart0)->ris; //这里的标志位可以用来判断是哪个中断被激活，是读还是写。
    //if(ris & UART_UARTRIS_RXRIS_BITS) //这是读中断
    //if(ris & UART_UARTRIS_TXRIS_BITS) //这是写中断

    // //由于固定使用uart1来处理通信所以这里暂时就这么写。
    // io_ro_32 ris= uart_get_hw(uart1)->ris;
    // if(ris & UART_UARTRIS_RXRIS_BITS)
    // {
    //     //接收到信号自动放入指定位置
    //     remote_info rem_info;
    //     bool b=remote_ctrl.OnReceive(rem_info);
    //     if(!b) return;

    //     //printf("got one remote info\n");
    //     last_remote_info = rem_info;
    //     //释放信号。
    //     sem_release(&remote_info_to_action);
    // }

    // if(ris & UART_UARTRIS_TXRIS_BITS)
    // {
    //     //写中断，可以写数据了。
    //     remote_ctrl.Send();
    // }

    //printf("on_remote_rx\n");
    //接收到信号自动放入指定位置
    uint8_t nret=remote_ctrl.OnReceive(last_remote_info, last_predefined_task, last_task_content);
    if(nret==0) {
        return;
    }

    //如果不是标记了任务，那信号就是摇杆信号。
    // if(nret & 2) {
    //     remote_predefined_task_signal=true; //标记释放的信号类型是预定义任务。
    // }else if(nret & 4) {
    //     remote_task_signal=true; //标记任务信号 
    // }

    if(nret & 4)
    {
        remote_task_signal=true;
    }
    //释放信号。现在这个信号可能由预定义任务产生。这里还没有处理好。
    sem_release(&remote_info_to_action);
}
#endif



//传感器中断统一处理, bmp390, icm426xx
void __not_in_flash_func(on_gpio_irq_callback)(uint gpio, uint32_t mask)
{
    if(gpio==ICM426XX_PIN_INT && mask == GPIO_IRQ_EDGE_RISE) {

        imu_info one;
        one.tmark=get_time_mark();
        one.us_signal=get_time_us_mark();

        //这个读取比6050快20倍，23微秒左右，通信频率快了60倍。
        icm426xx.Read(one.acce, one.gyro, one.temprature); 
        
        //未来温度值去比较校准的温度值，然后调节三个陀螺仪漂移，这样更准确。

        //陀螺仪零飘矫正
        //gyrox,gyroy如果有零漂移，会受加速度方向抑制，较大的偏移会引起yaw角长周期摆动。但没有明显的发散趋势。
        //gyroz的零漂移直接导致yaw角漂移和发散。必须精确校准及进行温度调节。
        //裸数据调节
        one.gyro[0]-= imu_gyro_raw_bias0;
        one.gyro[1]-= imu_gyro_raw_bias1;
        one.gyro[2]-= imu_gyro_raw_bias2;

        one.acce[0]/= imu_acce_raw_ratio;
        one.acce[1]/= imu_acce_raw_ratio;
        one.acce[2]/= imu_acce_raw_ratio;

        //前面的修正参数是基础修正参数，分两阶段完成。飞控每次安装做一次。可以看成是厂家出场修正设置。
        //后期修正。这个后期修正是每次起飞时进行的。是用户修正，每次起飞温度不同，可能造成偏差。
        one.gyro[0]-= imu_gyro_user_bias0; //用户校准参数，上电归零
        one.gyro[1]-= imu_gyro_user_bias1; //用户校准参数，上电归零
        one.gyro[2]-= imu_gyro_user_bias2; //用户校准参数，上电归零

        //温飘调节，仅yaw 20241026+
        //之前这里搞反了，低温下yaw是朝大方向漂移，所以低温要减去一个值去补偿偏大的值。

        if(!_in_imu_calib) //在校准时不应用温补
        {
            float temp_dif= one.temprature - imu_calib_temprature; //多数时候是负值，因高处温度低。
            FLOAT_LIMIT(temp_dif, -35.0, 15.0); //高温补15度，低温补-35度。之前这里搞反了上下限。
            one.gyro[2] += temp_dif*imu_gyroz_temp_comp;  //42605补偿可以较低。文档里是0.02º/s/ºC  0.025大了。0.015好像大了。升温yaw会增大。
        }

        //将gyroz的旋转度量压缩，在小幅旋转时选择大压缩率，或可避免大量的噪音被积分影响角度稳定性。
        //gyroz由于温度漂移会在温度变化时产生非0噪音，导致角度持续漂移。在多数时候机身都没有旋转。
        // if(!_in_imu_calib)
        // {
        //     float absgyro= fabs(one.gyro[2]);
        //     if(absgyro<3.0) one.gyro[2]*=0.1;
        //     else if(absgyro<5.0) one.gyro[2]*=0.2;
        //     else if(absgyro<8.0) one.gyro[2]*=0.3;
        //     else if(absgyro<12.0) one.gyro[2]*=0.4;
        //     else if(absgyro<16.0) one.gyro[2]*=0.5;
        //     else if(absgyro<20.0) one.gyro[2]*=0.6;
        //     else if(absgyro<25.0) one.gyro[2]*=0.7;
        //     else if(absgyro<30.0) one.gyro[2]*=0.8;
        //     else if(absgyro<35.0) one.gyro[2]*=0.9;
        // }

        float agyro[3];
        for(size_t i=0;i<3;i++) agyro[i]=one.gyro[i]/57.2958;

        //微秒时差，这里可以换为固定值。
        uint64_t us= one.us_signal - imu_queue.last().us_signal;
        //前期数据准备，后期数据转换耗时124微秒，主要是这个函数自身消耗
        icm426xx.calculate_euler_angles(agyro, one.acce, one.angle, one.acc_gnd, one.zaxis, float(us)/2000000.0f);

        ////累计yaw角度。测试数据，用来比对水平旋转360产生的偏差是来源于数据，还是来源于算法。
        //实验表明，如果是左旋一圈，这个累积值就偏大3度左右，右旋一圈就偏小3度左右。
        //左旋gyro是正数，右旋gyro是负数，说明累计的时长可能存在偏大的情况，这样积分就偏大了。
        //0.993这个数字是将机身旋转一周得到的积分偏差计算来的。旋转一周，差出约2-3度。这个数字可能在每个传感器上都不同。
        // one.igyro2 = imu_queue.last().igyro2 + (float(us)/1000000.0f)*one.gyro[2]; //累计yaw角度。测试数据


        // //把这个数限制在0-360范围。
        // if(one.igyro2 <0) one.igyro2+=360;
        // if(one.igyro2 >360) one.igyro2-=360;

        //算法对角速度有修正。这种修正有点可疑，它使得即使是gyro bias校准后，yaw角计算出来依旧是漂移。
        //是否复制回来似乎对yaw角计算没有影响。
        //for(size_t i=0;i<3;i++) one.gyro[i]=agyro[i]*57.2958;
        

        //水平安装偏差后期修正。水平面的偏差影响到水平加速度，如果水平度不够，则水平面x,y方向有加速度，则导致计算四元数有偏，
        //最后可能导致yaw角有偏，或许可以利用这个安装偏差角先行纠正三个方向的加速度？
        one.angle[0] -= install_bias_angle0;
        one.angle[1] -= install_bias_angle1;
       
        one.acc_gnd[0] -= install_bias_gnd_acc0;
        one.acc_gnd[1] -= install_bias_gnd_acc1;
        one.acc_gnd[2] -= install_bias_gnd_acc2;


// //用来做温度的校准。
//         imu_data_counter++; //总数据计数。

//         if(imu_data_counter%100==0) {
//             Yaw_info yi;
//             yi.us_signal=one.us_signal;
//             yi.yaw=one.angle[2];
            
//             //期间100个数据的平均温度。
//             //期间100个数据的累计陀螺数据，这个数据扣除了校准偏离数据。
//             //比较温度值和校准的温度值的偏差。同陀螺仪的三向去比较。不同的温度差造成不同的陀螺偏差。
//             //因为飞行时不搞校准。所以不加锁读取历史数据。
//             yi.temprature=0;

//             for(size_t i=0;i<99;i++)
//             {
//                 yi.gyro[0]+=imu_queue.last(i).gyro[0];
//                 yi.gyro[1]+=imu_queue.last(i).gyro[1];
//                 yi.gyro[2]+=imu_queue.last(i).gyro[2];
//                 yi.temprature += imu_queue.last(i).temprature;
//             }
            
//             yi.gyro[0]+=one.gyro[0];
//             yi.gyro[1]+=one.gyro[1];
//             yi.gyro[2]+=one.gyro[2];
//             yi.temprature += one.temprature;
//             yi.temprature /=100.0; //和校准温差的比较，高于校准温差则>0
//             yi.temprature -=imu_calib_temprature;
//             yi.gyro[0]/=100; //实际是扣除校准偏差后余下的偏差。
//             yi.gyro[1]/=100; //实际是扣除校准偏差后余下的偏差。
//             yi.gyro[2]/=100; //实际是扣除校准偏差后余下的偏差。
//             yaw_queue.push(yi);
//        }

        critical_section_enter_blocking(&imu_queue_section);
        imu_queue.push(one);
        critical_section_exit(&imu_queue_section);
        //接收了一个新数据，释放一个信号去执行微动作，不在中断内执行微动作。
        sem_release(&imu_data_to_action);
    }
    else   if(gpio==BMP390_PIN_INT && mask == GPIO_IRQ_EDGE_RISE) {

        //为了清理中断
        uint8_t status;
        bmp390.Read_Interrupt_Status(status);

        press_info one;
        one.tmark= get_time_mark();
        bmp390.Read(one.press, one.temprature);
        //气压高度，随后用来控制测距有效性。tof测距模型在光亮的地面或阳光下的雪地常有严重幻觉
        _baro_height = (initial_press_info.press - one.press)/11.7;

        critical_section_enter_blocking(&press_queue_section);
        press_queue.push(one);
        critical_section_exit(&press_queue_section);

    }
    
}



static void readlog()
{
    // for(int i=0;i<12;i++) {
    //     sleep_ms(1000);
    //     printf("wait for readlog.\n");
    // }

    read_log_from_flash();
    
}



//电调油门校准，先主板上电，然后给电调上电。然后按下user key (gp23)
//上了电路板后发现个电机别油门似乎需要校准。
void correct_ESC()
{
    //先给油门打到最大（卸掉螺旋桨），听到哔哔两声后，3秒内，按下GP23按键，将油门打到最小即可。
    motor_ctrl.Initialize(18,19,20,21,1.0);
    sleep_ms(2000);

    gpio_init(23);
    gpio_set_dir(23, GPIO_IN);
    //中断处理设置。
    gpio_set_irq_enabled_with_callback(23, GPIO_IRQ_EDGE_RISE, true, on_gpio_irq_callback); 

    while(1) {
        sleep_ms(1000);
    }

}

//连续收集陀螺仪漂移数据，结合温度变化，得到一个随温度变化的陀螺仪漂移记录。
//用热风枪吹热陀螺仪，然后放入冰箱里，平稳放置，上电，3秒后开始记录温度和陀螺仪的漂移，
//每5秒做一个记录，持续记录10分钟，这样可以采集各个温度段的漂移情况。
//20240330 在42605上测试，采用249hz带宽，温度22.5度对应yaw漂移0.64，温度40度对应0.82飘移，其他位置按直线推算。
//如果每个Imu都这么定标，则不需要后来的校准。
//20240330 在42688上测试，采用258hz带宽，温度22.7度对应-0.2624，温度43.7对应-0.4696，其他位置直线推算。
//按照这个推算，在当前的机架上采用这个线性拟合，然后取消各种校准，观察机器的漂移。
void raw_gyro_bias_with_temp_collect()
{
    printf("try to find gyro bias with temp\n");
    sleep_ms(5000);

    //关闭所有校准数据。
    imu_gyro_raw_bias0=0;
    imu_gyro_raw_bias1=0;
    imu_gyro_raw_bias2=0;
    imu_gyro_user_bias0=0;
    imu_gyro_user_bias1=0;
    imu_gyro_user_bias2=0;

    for(int times=0; times<240; times++)
    {
        //printf("times=%d\n", times);
        uint32_t st= get_time_mark();

        float gyro_raw_bias0=0;
        float gyro_raw_bias1=0;
        float gyro_raw_bias2=0;
        float temp=0;
        int num=0;

        while(1)
        {
            //执行数据监听，统计数据
            sem_acquire_blocking(&imu_data_to_action);
            sem_reset(&imu_data_to_action, 0); //清理信号标记
            imu_info &last=imu_queue.last();
            gyro_raw_bias0+= last.gyro[0];
            gyro_raw_bias1+= last.gyro[1];
            gyro_raw_bias2+= last.gyro[2];
            temp+= last.temprature;
            num++;
            if(get_time_mark() - st > 3000) break;
        }

        //3秒一段数据，大约有1500个。
        gyro_raw_bias0/=num;
        gyro_raw_bias1/=num;
        gyro_raw_bias2/=num;
        temp/=num;

        char dbg[128];
        sprintf(dbg, "%d,%5.3f,%5.4f,%5.4f,%5.4f\n", times,temp, gyro_raw_bias0, gyro_raw_bias1, gyro_raw_bias2);
        printf(dbg);
        logmessage(dbg);
    }
}
//初始数据校准，90秒左右的时间。
//42688的陀螺仪漂移对于温度变化很敏感，
//记录的这些数据可能和滤波方式也有关，滤波修改后这些数据可能会变化很大。
//如果每次上电测试都比较稳定，可以考虑用温度表格来确定漂移量。
//平均温度（imu）, x轴裸漂移，y轴裸漂移，z轴裸漂移
//20.06，0.407，0.015，1.17712
//19.48，0.398，0.017，1.17639
//18.28，0.363，0.030，1.17571

//13度左右 z轴裸漂移1.10附近

bool Initial_Calib_Stage1()
{
    
//偏移值归零。
    imu_gyro_raw_bias0=0;
    imu_gyro_raw_bias1=0;
    imu_gyro_raw_bias2=0;
    imu_acce_raw_ratio= 1.0;

//不能遗漏这些归零，否则不对。
    install_bias_angle0=0;
    install_bias_angle1=0;
    install_bias_gnd_acc0=0;
    install_bias_gnd_acc1=0;
    install_bias_gnd_acc2=0;

    imu_calib_temprature=25;

//用户级校准参数，出厂校准时归零。
    imu_gyro_user_bias0=0;
    imu_gyro_user_bias1=0;
    imu_gyro_user_bias2=0;

    icm426xx.ResetYaw();

    sleep_ms(50); //等待偏移归零后的有效数据。

    //静态接收一段数据，用来校准陀螺仪，加速度计等参数。
    uint32_t listen_start= get_time_mark();
    double sumgyros[3]={0}; //陀螺仪求和。
    double sumaccs[3]={0}; //三个方向加速度和, 这个可以用来调节水平度。
    double sumaccnorm=0; //加速度和
    int counter=0;

    double sumangle0=0;
    double sumangle1=0;
    double sumtemp=0; //温度

    while (1) {
        //执行数据监听，统计数据
        sem_acquire_blocking(&imu_data_to_action);
        sem_reset(&imu_data_to_action, 0); //清理信号标记
        //统计数据偏离。
        //三个陀螺仪的原始数据零漂移。
        //加计的向量和偏移。
        imu_info &last=imu_queue.last();
        sumgyros[0]+= last.gyro[0];
        sumgyros[1]+= last.gyro[1];
        sumgyros[2]+= last.gyro[2];
        sumaccs[0]+= last.acce[0];
        sumaccs[1]+= last.acce[1];
        sumaccs[2]+= last.acce[2];

        sumangle0+= last.angle[0];
        sumangle1+= last.angle[1];

        sumtemp+= last.temprature;
        double anorm= last.acce[0]*last.acce[0]+last.acce[1]*last.acce[1]+last.acce[2]*last.acce[2];
        anorm=sqrt(anorm);
        sumaccnorm+= anorm;

        counter++;
        if(get_time_mark() - listen_start > 20000) break;
    } 

    
    sumangle0/=counter;
    sumangle1/=counter;

    if(fabs(sumangle0)>8.0 || fabs(sumangle1)>8.0)
    {
        printf("angle too big to calibrate\n");
        char buf[64];
        sprintf(buf, "roll bias=%4.3f, pitch bias=%4.3f, too big to calib!", sumangle0, sumangle1);
        Send_remote_message(buf);
        return false; //不保存数据。
    } 

    imu_gyro_raw_bias0=sumgyros[0]/counter; //-0.194535 附近
    imu_gyro_raw_bias1=sumgyros[1]/counter; //-0.079576 附近
    imu_gyro_raw_bias2=sumgyros[2]/counter; //0.753387 附近
    imu_acce_raw_ratio=sumaccnorm/counter; //静态下加速度计调节到1.0的比例因子。
    imu_calib_temprature = sumtemp/counter;

    char buf[128];
    sprintf(buf, "raw gbias:%4.3f,%4.3f,%4.5f", imu_gyro_raw_bias0, imu_gyro_raw_bias1, imu_gyro_raw_bias2);
    Send_remote_message(buf);
    sprintf(buf, "temprature:%f", imu_calib_temprature);
    Send_remote_message(buf);
    sprintf(buf, "acce raw ratio=%5.5f", imu_acce_raw_ratio);
    Send_remote_message(buf);

    if(imu_acce_raw_ratio >1.1 || imu_acce_raw_ratio <0.9) {
        Send_remote_message("bad acce norm! calib failed");
        return false;
    }
    
    //Send_remote_message("basic calib finish.");
    return true;
    //sprintf(buf, "raw ang bias:%3.2f,%3.2f", sumangle0, sumangle1);
    //Send_remote_message(buf);
    //printf("raw gyro bias: %f, %f, %f; raw acc ratio: %f\n", imu_gyro_raw_bias0, imu_gyro_raw_bias1, imu_gyro_raw_bias2, imu_acce_raw_ratio);
    //printf("raw ang0 bias: %f, raw ang1 bias:%f\n", sumangle0, sumangle1);
}

//2阶段校准。在裸数据校准之后。校准加工后的数据，如大地坐标下的净加速度。
void Initial_Calib_Stage2()
{
    install_bias_angle0=0;
    install_bias_angle1=0;

    install_bias_gnd_acc0=0;
    install_bias_gnd_acc1=0;
    install_bias_gnd_acc2=0;

    icm426xx.ResetYaw();
    sleep_ms(50);

    //静态接收一段数据，用来校准陀螺仪，加速度计等参数。
    uint32_t listen_start= get_time_mark();
    double sumacc_gnd[3]={0};  //经过计算后的垂直方向加速度之和。
    int counter=0;
    float yaw_init=imu_queue.last().angle[2]; //最初yaw.

    float sumangle0=0;
    float sumangle1=0;

    while (1) {
        //执行数据监听，统计数据
        sem_acquire_blocking(&imu_data_to_action);
        sem_reset(&imu_data_to_action, 0); //清理信号标记
        //统计数据偏离。
        //三个陀螺仪的原始数据零漂移。
        //加计的向量和偏移。
        imu_info &last=imu_queue.last();

        sumacc_gnd[0]+= last.acc_gnd[0];
        sumacc_gnd[1]+= last.acc_gnd[1];
        sumacc_gnd[2]+= last.acc_gnd[2];
        sumangle0+= last.angle[0];
        sumangle1+= last.angle[1];

        counter++;
        if(get_time_mark() - listen_start > 20000) break;

    } 

    float yaw_end=imu_queue.last().angle[2]; //最后yaw

    install_bias_angle0= sumangle0/counter;
    install_bias_angle1= sumangle1/counter;
    install_bias_gnd_acc0=sumacc_gnd[0]/counter;
    install_bias_gnd_acc1=sumacc_gnd[1]/counter;
    install_bias_gnd_acc2=sumacc_gnd[2]/counter;

    char buf[128];
    sprintf(buf, "ang0 bias %f, ang1 bias %f", install_bias_angle0, install_bias_angle1);
    Send_remote_message(buf);
    std::cout<<buf<<std::endl;
    sprintf(buf, "accgnd bias: %f,%f,%f",install_bias_gnd_acc0,install_bias_gnd_acc1, install_bias_gnd_acc2);
    Send_remote_message(buf);
    std::cout<<buf<<std::endl;
    sprintf(buf,"yaw init=%5.3f, yaw end=%5.3f, dif=%5.3f", yaw_init, yaw_end, fabs(yaw_end - yaw_init));
    Send_remote_message(buf);
    std::cout<<buf<<std::endl;
}

//用户日常校准，仅做yaw角偏差补偿。
bool User_Calib()
{

    imu_gyro_user_bias0=0;
    imu_gyro_user_bias1=0;
    imu_gyro_user_bias2=0;
    
    _in_imu_calib=true;

    icm426xx.ResetYaw();
    sleep_ms(50);

    //检测校准时水平情况。
    double sumangle0=0;
    double sumangle1=0;
    double sumaccnorm=0;
    //总漂移。
    double sumbias[3]={0};
    float first_yaw=0;
    float last_yaw=0;

    uint32_t listen_start=get_time_mark();
    int counter=0;
    
    first_yaw= imu_queue.last().angle[2];

    float temp_mean=0;

    while (1) {
        //执行数据监听，统计数据
        sem_acquire_blocking(&imu_data_to_action);
        sem_reset(&imu_data_to_action, 0); //清理信号标记
        //统计数据偏离。
        //三个陀螺仪的原始数据零漂移。
        //加计的向量和偏移。
        imu_info &last=imu_queue.last();
        sumbias[0]+= last.gyro[0];
        sumbias[1]+= last.gyro[1];
        sumbias[2]+= last.gyro[2];
        sumangle0+= last.angle[0];
        sumangle1+= last.angle[1];
        temp_mean+= last.temprature;

        double anorm= last.acce[0]*last.acce[0]+last.acce[1]*last.acce[1]+last.acce[2]*last.acce[2];
        anorm=sqrt(anorm);
        sumaccnorm+= anorm;

        counter++;
        if(counter%1000==0) {
            //每1000个数据检查一下角度，如果水平角度太大，直接停止矫正。
            if(fabs(sumangle0/counter)>3.0 || fabs(sumangle1/counter)>3.0)
            {
                Send_remote_message("fail to calibrate data");
                //place it on a horizontal surface for correction
                Send_remote_message("place it on a horizontal surface for correction");
                _in_imu_calib=false;
                return false; //不保存数据。
            } 
        }
        if(get_time_mark() - listen_start > 15000) break;
    } 

    last_yaw= imu_queue.last().angle[2];
    
    sumangle0/=counter;
    sumangle1/=counter;
    temp_mean/=counter;

    if(fabs(sumangle0)>3.0 || fabs(sumangle1)>3.0)
    {
        printf("angle too big to calib\n");
        Send_remote_message("fail to calibrate data");
        _in_imu_calib=false;
        return false; //不保存数据。
    } 

    imu_calib_temprature=temp_mean; //记录校准温度。
    imu_acce_raw_ratio=sumaccnorm/counter;

    imu_gyro_user_bias0 = sumbias[0]/counter;
    imu_gyro_user_bias1 = sumbias[1]/counter;
    imu_gyro_user_bias2 = sumbias[2]/counter;

    char buf[128];
    sprintf(buf, "gbias:%5.4f,%5.4f,%5.4f", imu_gyro_user_bias0, imu_gyro_user_bias0, imu_gyro_user_bias2);
    Send_remote_message(buf);
    sprintf(buf, "first yaw %5.3f, last yaw %5.3f, temp %4.2f", first_yaw, last_yaw, temp_mean);
    Send_remote_message(buf);
    _in_imu_calib=false;

    return true;
}



//显示imu数据，并计算推算数据，可以查看数据噪音情况。
void Check_imu_data()
{

    _in_imu_calib=false;

    char log[256];

    int num=0;

    while(1)
    {
        imu_info last= imu_queue.last();

        if(last.tmark!=0)
        {
            

            // printf("angles=%f,%f,%f, gyro=%f,%f, temp=%f, gap=%d\n", 
            //     imu_queue.last().angle[0],imu_queue.last().angle[1],imu_queue.last().angle[2] , 
            //     imu_queue.last().gyro[0], imu_queue.last().gyro[1], imu_queue.last().temprature, 
            //     imu_queue.last().tmark - imu_queue.last(1).tmark);


            sprintf(log, "t=%d,g=%4.3f,%4.3f,%4.3f,a=%4.3f,%4.3f,%4.3f,an=%4.3f,%4.3f,%4.3f,zx=%4.3f\n",
                last.tmark,last.gyro[0],last.gyro[1],last.gyro[2],
                last.acce[0], last.acce[1], last.acce[2], 
                last.angle[0], last.angle[1], last.angle[2],
                last.zaxis
            );

            logmessage(log);

            num++;
            
        }

        uint32_t now = get_time_mark();
        if(now > last.tmark && now - last.tmark > 5) {
            printf("signal break, now=%d, sig=%d\n", now, last.tmark);
        }

        if(last.tmark - imu_queue.last(180).tmark >364)
        {
            uint32_t dif= last.tmark - imu_queue.last(180).tmark;
            printf("signal break, last=%d, 180=%d, dif=%d\n", last.tmark, imu_queue.last(180).tmark, dif);
        }

        sleep_ms(20);
        printf("%d\n", num);
        if(num>500) break;
    }

    write_log_to_flash();
}




#ifdef USE_BMP390
void Check_baro_data()
{

    //主动读。
    // while(1)
    // {
    //     press_info one;
    //     one.tmark= get_time_mark();
    //     bmp390.Read(one.press, one.temprature);
    //     printf("press=%f, temp=%f\n", 
    //             one.press, 
    //             one.temprature);
    //     sleep_ms(500);
    // }

    static float s0=0;
    static float s1=0;
    static float s2=0;
    static float acc=0;

    while(1)
    {
        if(press_queue.last().tmark!=0)
        {
            printf("press=%f, temp=%f, gap=%d\n", 
                press_queue.last().press, 
                press_queue.last().temprature, 
                press_queue.last().tmark - press_queue.last(1).tmark );

            //     press_info pr0, pr1, pr2, pr3, pr4; //阶段气压数据
            //     //uint32_t dtime; //数据时间
            //     //uint32_t tgap; //20230619+ 一个气压数据时间间隔。毫秒 >0
            //     critical_section_enter_blocking(&press_queue_section);
            //     //dtime=press_queue.last().tmark;
            //     //tgap= dtime - press_queue.last(1).tmark;
            //     pr0= press_queue.history_mean(0, 3);  //当前气压值，3均值。
            //     pr1= press_queue.history_mean(1, 3);  //搭配pr0做气压微分阻尼。
            //     pr2= press_queue.history_mean(5, 3);   //0.1s
            //     pr3= press_queue.history_mean(10, 3);  //0.2s
            //     pr4= press_queue.history_mean(20, 3);  //0.4s
            //     critical_section_exit(&press_queue_section);

            //     //三阶段气压速度数据。
            //     float pspd0 = (pr0.press - pr2.press)*10.0/12.0; //0.1s //m/s 下为正
            //     float pspd1 = (pr0.press - pr3.press)*5.0/12.0; //0.2s  //m/s 下为正
            //     float pspd2 = (pr0.press - pr4.press)*2.5/12.0; //0.4s  //m/s 下为正

            //     float pspd = pspd2*0.55 + pspd1*0.3 + pspd0*0.15; //综合气压指示速度，m/s 向下为正
            //     //都间隔0.1秒, *10换算为1秒差，/12.0换算为标准单位，正值为加速度向下，负值加速度向上。
            //     float pacc = ((pr0.press - pr2.press) - (pr2.press - pr3.press))*10.0/12.0; 

            //     printf("pspd0,1,2=%3.2f,%3.2f,%3.2f, pspd=%3.2f, pacc=%3.2f\n", pspd0, pspd1, pspd2, pspd, pacc);



                press_info pr0, pr1, pr2, pr3;//, pr4, pr5; //采样2阶段气压数据，可求出气压速度。
                uint32_t dtime; //数据时间
                uint32_t tgap; //20230619+ 一个气压数据时间间隔。毫秒 >0
                critical_section_enter_blocking(&press_queue_section);
                dtime=press_queue.last().tmark;
                tgap= dtime - press_queue.last(1).tmark;
                pr0= press_queue.history_mean(0, 5);  //当前气压值，5均值。
                pr1= press_queue.history_mean(1, 5);  //搭配pr0做气压阻尼。
                pr2= press_queue.history_mean(5, 5); //0.1s
                pr3= press_queue.history_mean(25,5); //0.5s
                critical_section_exit(&press_queue_section);

                float pspd0 = (pr0.press - pr2.press)*(200.0/tgap)/12.0; //m/s 下为正
                float pspd1 = (pr2.press - pr3.press)*(50.0/tgap)/12.0; //m/s 下为正
                float pspd2 = (pr0.press - pr3.press)*(40.0f/tgap)/12.0; //m/s 下为正 25个数据时差，0.5秒, 用来设定速度的调节
                s0=0.9*s0+0.1*pspd0;
                s1=0.9*s1+0.1*pspd1;
                s2=0.9*s2+0.1*pspd2;

                //气压指示的加速度。这个可以去和imu加速度比较，如果是近似的就相信，如果差别很大，就否定。
                //float pacc =  (pspd0 - pspd1)*(200.0/tgap); //m/s^2 下为正
                float pacc = (s0 - s1)*(200.0/tgap);
                acc= 0.9*acc+0.1*pacc;

                printf("pspd0,1,2=%3.2f,%3.2f,%3.2f, pacc=%3.2f\n", pspd0, pspd1, pspd2, pacc);
                
                
                //另一个方法计算速度
                float time[5]={0.0,0.02,0.04,0.06,0.08};
                float press[5];
                critical_section_enter_blocking(&press_queue_section);
                for(size_t i=0;i<5;i++)
                {
                    press[i]=press_queue.last(i).press;
                }
                critical_section_exit(&press_queue_section);
                
            
                // 最小二乘拟合
                float sum_t = 0.2, sum_at = 0.0, sum_a = 0.0;
                for (size_t i=0; i<5; ++i) {
                    //sum_t += time[i];
                    //sum_t2 += time[i] * time[i];
                    sum_at += press[i] * time[i];
                    sum_a += press[i];
                }

                float nspd= (5 * sum_at - sum_t * sum_a) / 0.02 /11.7;
                printf("nspd=%3.2f\n", nspd);
        }
        else
        {
            printf("no baro data\n");
        }

        sleep_ms(200);
    }

}
#endif

#ifdef USE_GPS
void Check_gps_data()
{

    //gps数据上下间隔非常不稳定，1秒10次数据，间隔常在50多毫秒-160多毫秒波动。使用上要小心。
    while(1)
    {
        if(gps_queue.last().tmark!=0)
        {
            uint32_t now=get_time_mark();

            printf("lon=%f, lat=%f, hei=%f, acc=%f,  gap=%d, gap4=%d\n", 
            gps_queue.last().longitude, gps_queue.last().latitude, gps_queue.last().height, gps_queue.last().hacc,
            
            gps_queue.last().tmark - gps_queue.last(1).tmark, now - gps_queue.last(4).tmark);

            char buf[64];
            sprintf(buf, "ns=%d, vacc=%f",  gps_queue.last().ns, gps_queue.last().vacc);
            Send_remote_message(buf);
            sprintf(buf, "%f,%f", gps_queue.last().longitude, gps_queue.last().latitude, gps_queue.last().height);
            Send_remote_message(buf);
        }
        else
        {
            printf("wait for gps\n");
        }

        sleep_ms(1000);
    }

    

}
#endif

#if defined (USE_QMC5883) || defined (USE_HMC5883)

//检查指南针和电压，这两者在兼容芯片上有兼容性问题。
void Check_compass_and_power()
{
    for(;;)
    {
       // printf("read compass\n");
        int16_t x,y,z;
        // i2c_compass.read(x,y,z);
        // float heading= atan2(y,x);
        // //heading-=0.04; //磁偏角？
        // if(heading<0) heading+= 2*3.1415926;
        printf("read compass\n");
        Send_remote_message("read compass");
        
        Read_Compass_Data(x,y,z);
        printf("read compass finish\n");
        Send_remote_message("read compass finish");
        
        //float hd=Compass_Heading();

        Send_remote_message("check power");
        printf("check power\n");
        float pv= check_system_power_voltage();
        Send_remote_message("check power finish");
        printf("check power finish");
        char msg[128];
        sprintf(msg, "x=%d,y=%d,z=%d, p=%4.1f\n", x,y,z, pv);
        Send_remote_message(msg);
        printf(msg);
        printf("\n");

        sleep_ms(1000);
    }
}

void Check_compass_data()
{
    for(;;)
    {
       // printf("read compass\n");
        int16_t x,y,z;
        // i2c_compass.read(x,y,z);
        // float heading= atan2(y,x);
        // //heading-=0.04; //磁偏角？
        // if(heading<0) heading+= 2*3.1415926;

        Read_Compass_Data(x,y,z);
        float hd=Compass_Heading();

        char msg[128];
        sprintf(msg, "x=%d,y=%d, z=%d, hd=%4.1f", x,y,z, hd);
        Send_remote_message(msg);
        printf(msg);
        printf("\n");
        //std::cout<<msg<<std::endl;
        //std::cout<<"x="<<x<<",y="<<y<<",heading="<<heading<<",deg="<<heading*180/3.14159326<<std::endl;

        //float hd=Compass_Heading();

        sleep_ms(500);
    }
}
#endif

//执行Imu校准。平地静止状态下。
//不够水平的情况下，yaw角的角速度会有大偏差。尽量在比较平的地面做。
//全面校准。
void Do_imu_calibration()
{
    printf("calib stage1\n");
    _in_imu_calib=true;
    Send_remote_message(27);
    bool b1=Initial_Calib_Stage1();
    if(!b1) {
        _in_imu_calib=false;
        return;
    }

    printf("calib stage2\n");
    Send_remote_message(28);
    Initial_Calib_Stage2();

    bool b2= write_arguements_to_flash();
    if(b2) {
        printf("write calibration data to flash\n");
    }else{
        printf("write calibration data fail\n");
    }

    Send_remote_message(29);// 校准结束
    _in_imu_calib=false;
}

//没有考虑z轴方向。只能水平旋转校准。
void Do_magnet_calibration()
{
   #if defined USE_QMC5883 || defined (USE_HMC5883)
    Send_remote_message("start 30s magnet calibration");
    printf("start 30s magnet calibration\n");

    uint32_t st= get_time_mark();

    float max_cx=-90000;
    float min_cx= 90000;
    float max_cy=-90000;
    float min_cy= 90000;
    //float max_cz=-90000;
    //float min_cz= 90000;
    do
    {
        float curpitch, curroll;
        critical_section_enter_blocking(&imu_queue_section);
        curpitch=imu_queue.last().angle[0];
        curroll=imu_queue.last().angle[1];
        critical_section_exit(&imu_queue_section); 

        int16_t cx,cy,cz;
        Read_Compass_Data(cx,cy,cz);

    // //做一个转化，全部调节为水平。参考ymfc-32
    // //http://www.brokking.net/ymfc-32_downloads.html
    // //http://www.brokking.net/YMFC-32/YMFC-32_document_1.pdf

        float compXH= cx*cos(curpitch*-0.0174533)
                    +cy*sin(curroll*0.0174533)*sin(curpitch*-0.0174533)
                    -cz*cos(curroll*0.0174533)*sin(curpitch*-0.0174533);

        float compYH= cy*cos(curroll*0.0174533)
                            +cz*sin(curroll*0.0174533); 

        if(compXH>max_cx) max_cx=compXH;
        if(compXH<min_cx) min_cx=compXH;
        if(compYH>max_cy) max_cy=compYH;
        if(compYH<min_cy) min_cy=compYH;

        // if(cx > max_cx) max_cx=cx;
        // if(cx < min_cx) min_cx=cx;
        // if(cy > max_cy) max_cy=cy;
        // if(cy < min_cy) min_cy=cy;
        // if(cz > max_cz) max_cz=cz;
        // if(cz < min_cz) min_cz=cz;

        sleep_ms(15);
        uint32_t now=get_time_mark();
        if(now - st > 30000) break;
    }while(1);


    char buf[64];
    printf("mincx=%f,maxcx=%f\n", min_cx, max_cx);
    printf("mincy=%f,maxcy=%f\n", min_cy, max_cy);
    //printf("mincz=%f,maxcz=%f\n", min_cz, max_cz);

    sprintf(buf, "cx=%f,%f", min_cx, max_cx);
    Send_remote_message(buf);
    sprintf(buf, "cy=%f,%f", min_cy, max_cy);
    Send_remote_message(buf);
    //sprintf(buf, "cz=%f,%f", min_cz, max_cz);
    //Send_remote_message(buf);

    magnet_midx= (min_cx + max_cx)/2.0;
    magnet_midy= (min_cy + max_cy)/2.0;
    magnet_midz= 0;
    printf("midx=%f,midy=%f\n", magnet_midx, magnet_midy);

    sprintf(buf, "midx=%f, midy=%f", magnet_midx, magnet_midy);
    Send_remote_message(buf);


    bool b2=write_arguements_to_flash();
    if(b2) {
        Send_remote_message("save calib data ok");
    }else{
        Send_remote_message("save calib data fail");
    }

   #else
    Send_remote_message("no magnet device");
   #endif
}


//日常使用校准,用户级校准，遥控器控制。不保存数据。
void Do_user_calibration()
{
    Send_remote_message(27);
    bool b=User_Calib();
    if(b)
    {
        Send_remote_message(29);// 校准结束   
    }
    else
    {
        Send_remote_message("user calib fail.");
    }
}

bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

void i2c0_scan()
{
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    gpio_pull_up(0);
    gpio_pull_up(1);

    for(int i=0;i<10;i++)
    {
        printf("wait for scan\n");
        sleep_ms(1000);
    }
    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(i2c0, addr, &rxdata, 1, false);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
        sleep_ms(500);
    }
    printf("Done.\n");    
}

//模拟电机工作。测试稳定性。总拉力在1.6-2.0之间波动


//遥控器信号测试，测量遥控距离多远会失效。这里面修改为全包信息发送
#ifdef USE_GPS
void remote_signal_test()
{
    int cnt=0;

    char buf[128];

    double start_lati=0;
    double start_logi=0;
    bool fixed=false;

    while(1)
    {
        gps_info a;
        critical_section_enter_blocking(&gps_queue_section);
        a=gps_queue.last();
        critical_section_exit(&gps_queue_section);

        if(a.tmark==0)
        {
            //数据不满足要求，很可能是GPS等待。回传GPS数据。
            sprintf(buf, "no gps fix, %d", cnt++);
            Send_remote_message(buf);
        }
        else if(a.hacc < 1.2) {

            //collect_and_send_skyinfo();
 
            if(!fixed) {
                start_lati=a.latitude;
                start_logi=a.longitude;
                fixed=true;

            }


            float dist, angle;
            global_gps_dist_angle(a.latitude, a.longitude, start_lati, start_logi, dist, angle);
            sprintf(buf, "gps fixed, %d, dist=%f", cnt++, dist);
            Send_remote_message(buf);

        }

        sleep_ms(1000);
    }

}
#endif

//校准电流表。连接可调电源，设置不同的马达动力，观察对比电流
void calib_current()
{
    

    motor_ctrl.Initialize();
    power_prepare();

    sleep_ms(1000);
    float burs[4];
    burs[0]=burs[1]=burs[2]=burs[3]=0.6;
    motor_ctrl.SetBurdens(burs,0);
    //采集电压，电流数据发送到遥控器。
    for(int i=0;i<10;i++) {
        float v=check_system_power_voltage();
        float a=check_system_power_current();
        //Send_remote_powerinfo(v, a);
        char log[64];
        sprintf(log, "bur=0.6, v=%f,a=%f\n", v, a);
        logmessage(log);
        sleep_ms(500);
    }

    burs[0]=burs[1]=burs[2]=burs[3]=0.7;
    motor_ctrl.SetBurdens(burs, 0);
    //采集电压，电流数据发送到遥控器。
    for(int i=0;i<10;i++) {
        float v=check_system_power_voltage();
        float a=check_system_power_current();
        char log[64];
        sprintf(log, "bur=0.7, v=%f,a=%f\n", v, a);
        logmessage(log);
        sleep_ms(500);
    }

    burs[0]=burs[1]=burs[2]=burs[3]=0.8;
    motor_ctrl.SetBurdens(burs,0);
    //采集电压，电流数据发送到遥控器。
    for(int i=0;i<10;i++) {
        float v=check_system_power_voltage();
        float a=check_system_power_current();
        char log[64];
        sprintf(log, "bur=0.8, v=%f,a=%f\n", v, a);
        logmessage(log);
        sleep_ms(500);
    }

    burs[0]=burs[1]=burs[2]=burs[3]=0.9;
    motor_ctrl.SetBurdens(burs, 0);
    //采集电压，电流数据发送到遥控器。
    for(int i=0;i<10;i++) {
        float v=check_system_power_voltage();
        float a=check_system_power_current();
        char log[64];
        sprintf(log, "bur=0.9, v=%f,a=%f\n", v, a);
        logmessage(log);
        sleep_ms(500);
    }

    for(int i=8;i>=0;i--)
    {
        burs[0]=burs[1]=burs[2]=burs[3]=0.1*i;
        motor_ctrl.SetBurdens(burs, 0);
        sleep_ms(300);
    }

    write_log_to_flash();
    while(1) sleep_ms(100000);
}

//津航4合1电调测试，这个电调似乎在pwm协议下有点稳定性问题。在dshot300协议下还没发现有问题。
void JHEMCU_ESC_STABLE_TEST()
{
    bool b=motor_ctrl.Initialize();
    if(!b) {
        printf("bad init\n");
        logmessage("bad init\n");
        led_blink(100, 500);
        return;
    }

    sleep_ms(2000);
    
    float burs[4];

    //先跑一遍按顺序递增递减。
    led_blink(10, 0.5);
    for(int i=15;i<100;i++)
    {
        burs[0]= float(i)/100.0f;
        burs[1]= burs[0];
        burs[2]= burs[0];
        burs[3]= burs[0];
        motor_ctrl.SetBurdens(burs,0);
        sleep_ms(100);
    }

    led_blink(10, 0.5);

    for(int i=99;i>20;i--)
    {
        burs[0]= float(i)/100.0f;
        burs[1]= burs[0];
        burs[2]= burs[0];
        burs[3]= burs[0];
        motor_ctrl.SetBurdens(burs,0);
        sleep_ms(100);
    }

    led_blink(10, 0.5);

    // while(1)
    // {
    //     burs[0]= test_jhemcu(rgen);
    //     if(burs[0]<0.15) burs[0]=0.15;
    //     if(burs[0]>0.998) burs[0]=0.998;

    //     burs[1]= burs[0];
    //     burs[2]= burs[0];
    //     burs[3]= burs[0];
    //     motor_ctrl.SetBurdens(burs);
    //     sleep_ms(2);
    // }

}

//定时执行，20毫秒一次，以便滤波传送给遥控器。
bool power_check(repeating_timer_t* rtt)
{
    //新增电压电流采样任务，20231013
    float pv=check_system_power_voltage();
    float pc=check_system_power_current();
    //printf("%f\n", pc); //debug
    //可以积分总电量消耗。电压*电流*时间。20231112+
    uint32_t now  = get_time_mark();

    //使用无滤波的测量数据来积分。得到的是标准单位，焦耳。
    system_power_used+= pv*pc*(now - system_power_last_check_time)/1000.0f;
    system_power_last_check_time=now;

    //滤波，是给数据传输和飞行控制使用的，避免太大波动，20241025 增加滤波窗口
    system_power_voltage = (system_power_voltage * 63.0 + pv)/64.0;
    system_power_current = (system_power_current * 63.0 + pc)/64.0;
    //recursive_mutex_enter_blocking(&current_queue_mutex);
    current_queue.push(system_power_current); //电流序列记录
    //recursive_mutex_exit(&current_queue_mutex);
    return true;
}



//配置模式，插上usb进入这个模式。20241211+
void config_mode()
{

        read_arguements_from_flash();
        sleep_ms(1000);
        printf("config mode\n");
        printf("arg lists:\n");
        for(auto it=adjustable_running_arguements.begin();it!=adjustable_running_arguements.end();it++)
        {
            char name[5];
            name[0]=it->first.name[0];
            name[1]=it->first.name[1];
            name[2]=it->first.name[2];
            name[3]=it->first.name[3];
            name[4]=0;
            printf("%s:%f\n", name, it->second);
        }

        printf("waiting for command\n");

        while(1){
            std::string comd;
            std::getline(std::cin, comd);
            size_t pr=comd.find_first_of(0x0d); //回车符号
            if(pr!=std::string::npos) comd.erase(pr);

            TrimString(comd);

            std::vector<std::string> splits;
            int line= ParseLine(comd, splits);
            if(line==1 ) {
                TrimString(splits[0]);

                if(splits[0].compare("save")==0)
                {
                    printf("do save to flash\n");
                    write_arguements_to_flash();
                }
                else if(splits[0].compare("load")==0)
                {
                    printf("do load from flash\n");
                    read_arguements_from_flash();
                }
                else if(splits[0].compare("dump")==0)
                {
                    readlog();
                }
                else if(splits[0].compare("quit")==0)
                {
                    return;
                }
                else
                {
                    printf("unknow command: %s\n", splits[0].c_str());
                    continue;
                }
            }
            else if(line==2) {
                TrimString(splits[0]);
                TrimString(splits[1]);
                if(splits[0].size()!=4) {
                    printf("name len!=4\n");
                    continue;
                }
                if(!arguement_name::isLeagleName(splits[0].c_str())) {
                    printf("name illegal\n");
                    continue;
                }

                float v= atof(splits[1].c_str());
                arguement_name na;
                na=splits[0];
                adjustable_running_arguements[na]=v;
                Spread_arguements();

                printf("set %s to %f\n", splits[0].c_str(), v);

            }
            else{
                printf("invalid command, args=%d\n", line);
                continue;
            }

            printf("arg list:\n");
            for(auto it=adjustable_running_arguements.begin();it!=adjustable_running_arguements.end();it++)
            {
                char name[5];
                name[0]=it->first.name[0];
                name[1]=it->first.name[1];
                name[2]=it->first.name[2];
                name[3]=it->first.name[3];
                name[4]=0;
                printf("%s:%f\n", name, it->second);
            }
        }
}

//论单纯计算，pico的125mhz对比x86的3.0Ghz，速度是1/50
//pico没有标准文件系统，不能简单记录日志
int main() {

    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);


    if(tud_cdc_connected()) {
        printf("usb connected\n");
        led_blink(6, 200);
        config_mode();
    }

    power_control_solution_adjust(1); //设置默认电流控制方案。

    //用来测试系统受水淋的稳定性。
    // gpio_init(16);
    // gpio_pull_down(16);
    // led_blink(10, 200);

    // while(1) {
    //     led_blink(1, 1000);
    // }

    //led_fade_in();
    //led_fade_out();
    led_blink(3, 200);

    //critical_section_init(&log_section); //20240906+
    mutex_init(&log_mutex); //20250313 修改。
    //mutex_init(&current_queue_mutex); //20250315+

#ifdef USE_ICM426XX
    critical_section_init(&imu_queue_section);
#endif

#ifdef USE_TOF
    critical_section_init(&tof_oflow_queue_section);
#endif

#ifdef USE_GPS
    critical_section_init(&gps_queue_section);
#endif

#ifdef USE_BMP390
    critical_section_init(&press_queue_section);
#endif

    power_prepare(); //电压电流测量准备。

    sleep_ms(50);
    repeating_timer_t rtt;
    add_repeating_timer_ms(20, power_check, 0, &rtt); //20240301 修改为20毫秒一个采样，这样在电流敏感的运动中调节反应就快速。

    sleep_ms(10);

    last_task_content.reserve(256); //为避免反复内存分配释放导致内存碎片，保留256字节。20240707+
    

#ifdef USE_REMOTE_CTRL
#if defined (USE_DUAL_LORA)
    sem_init(&remote_info_to_action, 0, 1);

    remote_ctrl.Initialize();
    
    //设置中断接收数据，
    if(DUAL_LORA_UART_PORT==0)
    {
        irq_set_exclusive_handler(UART0_IRQ, on_remote_ctrl_rx_tx);
        irq_set_enabled(UART0_IRQ, true);
        uart_set_irq_enables(uart0, true, false); //读中断开
    }
    else
    {
        irq_set_exclusive_handler(UART1_IRQ, on_remote_ctrl_rx_tx);
        irq_set_enabled(UART1_IRQ, true);
        uart_set_irq_enables(uart1, true, false);  //读中断开
    }

#endif
#endif //remote control

    sleep_ms(10);

    Send_remote_message(1); //remote control initialized.

    Send_remote_powerinfo(system_power_voltage, system_power_current, system_power_used);

    //sleep_ms(5000);
    //v8版本四个电机接口连线顺序与之前完全反过来，21，20，19，18，这样连线方便。
    //26/27还是GPS的指南针，但是板子上已经有指南针了。目前优选板上的指南针hmc5883。
    motor_ctrl.Initialize(); 

    Send_remote_message(2); //motor initialized.

    sem_init(&imu_data_to_action, 0, 1);
    
    bool b;

#if defined USE_ICM426XX
    //以下icm426xx初始化代码VV
    
    icm426xx.Initialize(ICM426XX_SPI_PORT, ICM426XX_PIN_SCK, ICM426XX_PIN_MISO, ICM426XX_PIN_MOSI, ICM426XX_PIN_CS, 24000000, 0);
    b=icm426xx.Reset(); 

    if(!b){
        printf("imu failed\n");
        led_blink(6, 100);
        Send_remote_message(3);
        return 0; //不能进行下去。
    }else{
        printf("imu init ok\n");
    }


    //中断接收脚初始化。
    gpio_init(ICM426XX_PIN_INT);
    gpio_set_dir(ICM426XX_PIN_INT, GPIO_IN);

    //icm426xx中断处理设置。
    gpio_set_irq_enabled_with_callback(ICM426XX_PIN_INT, GPIO_IRQ_EDGE_RISE, true, on_gpio_irq_callback);      
    
    //printf("imu init\n");
    Send_remote_message(4);
    Send_remote_powerinfo(system_power_voltage, system_power_current, system_power_used);

    //^^以上icm426xx初始化代码^^

    //Check_imu_data(); //仅检查imu的工作。
#endif


#ifdef USE_TOF
    {
        //uint sm = 0;  //State machine index 0,1,2,3
        //PIO pio = pio0; //pio设备，共有2个, pio0, pio1
        uint offset_tof = pio_add_program(pio1, &uart_rx_program);
        //TOF_TX_PIN就是本机的RX_PIN
        //uint sm = pio_claim_unused_sm(pio1, true); //20230929修改，自动选择sm, 下面原来sm填写0，现在改为sm
        //sm依旧指定为0，因为读取数据那边需要指定sm，所以这里固定配置sm=0
        //所有设备都使用pio1，将来所有的dshot输出口都使用pio0,那边正好4个状态机。
        uart_rx_program_init(pio1, 0, offset_tof, TOF_OFLOW_RX_PIN, TOF_OFLOW_BUADRATE); //使用0号状态机
        //完美的pio中断设置, 每个pio块上有两个中断号，分别是PIO0_IRQ_0, PIO0_IRQ_1,PIO1_IRQ_0, PIO1_IRQ_1
        pio_set_irq0_source_enabled(pio1, pis_sm0_rx_fifo_not_empty, true);
        irq_set_exclusive_handler(PIO1_IRQ_0, on_pio1_tof_rx);
        irq_set_enabled(PIO1_IRQ_0, true);
    }

    //printf("tof init\n");
    Send_remote_message(5);
    Send_remote_powerinfo(system_power_voltage, system_power_current, system_power_used);

#endif
    

#ifdef USE_GPS

    #if defined USE_GPS_PIO
    {
        //20230929修改，设备只使用pio1,pio0用来输出dshot
        //PIO pio = pio0; //pio设备，共有2个, pio0, pio1
        uint offset_gps = pio_add_program(pio1, &uart_rx_program); //20230929,修改为pio1
        //uint sm = pio_claim_unused_sm(pio1, true); //20230929修改，自动选择sm, 下面原来sm填写0，现在改为sm
        //sm依旧指定为1，因为读取数据那边需要指定sm，所以这里固定配置sm=1
        //GPS_RX_PIN就是主机的RX_PIN
        uart_rx_program_init(pio1, 1, offset_gps, GPS_RX_PIN, GPS_BUADRATE); //使用0号状态机
        pio_set_irq0_source_enabled(pio1, pis_sm0_rx_fifo_not_empty, true);
        irq_set_exclusive_handler(PIO1_IRQ_0, on_pio1_gps_rx);
        irq_set_enabled(PIO1_IRQ_0, true);
    } 
    #endif

    #if defined USE_GPS_UART
    
    uart_gps.Initialize(GPS_UBLOX_UART_PORT, GPS_UBLOX_RX_PIN ,GPS_UBLOX_TX_PIN);

    //设置中断接收数据，
    if(GPS_UBLOX_UART_PORT==0)
    {
        irq_set_exclusive_handler(UART0_IRQ, on_gps_rx);
        irq_set_enabled(UART0_IRQ, true);
        uart_set_irq_enables(uart0, true, false);
    }
    else
    {
        irq_set_exclusive_handler(UART1_IRQ, on_gps_rx);
        irq_set_enabled(UART1_IRQ, true);
        uart_set_irq_enables(uart1, true, false); 
    }
    #endif

    printf("gps init\n");
    Send_remote_message(6);
    Send_remote_powerinfo(system_power_voltage, system_power_current, system_power_used);
#endif


#ifdef USE_QMC5883

    printf("compass init\n");
    Send_remote_message(38); //try to find compass

    //新购入的小gps的指南针，在这个100k速度配置下，读取数据偶发问题会失败。无法保证转一圈都能够读到数据，原因不明。
    //小gps带的指南针在400k速度下找不到。几次都是失败，只能用100，在100的速度下读数也不太稳定，失败率较高。
    //i2c_cpass_qmc5883.Initialize(1, QMC5883_SCL, QMC5883_SDA, 100*1000); //400的速度偶尔失败，试试100k

    i2c_cpass_qmc5883.Initialize(0, QMC5883_SCL, QMC5883_SDA, 400*1000); 

    b=i2c_cpass_qmc5883.init();
    if(!b){
        printf("compass failed\n");
        //led_blink(30, 50);
        led_blink(9, 100);
        Send_remote_message(36); //compass failed.
        return 0; //不能进行下去。
    }

    Send_remote_message(39); //compass found

#endif

//v8版本版上新增hmc5883,端口0/1
#ifdef USE_HMC5883
    printf("compass hmc5883 init\n");
    Send_remote_message(38); //try to find compass

    //新购入的小gps的指南针，在这个100k速度配置下，读取数据偶发问题会失败。无法保证转一圈都能够读到数据，原因不明。
    //小gps带的指南针在400k速度下找不到。几次都是失败，只能用100，在100的速度下读数也不太稳定，失败率较高。
    i2c_cpass_hmc5883.Initialize(0, HMC5883_SCL, HMC5883_SDA, 400*1000); 
    b=i2c_cpass_hmc5883.init();
    if(!b){
        printf("compass failed\n");
        led_blink(30, 50);
        Send_remote_message(36); //compass failed.
        return 0; //不能进行下去。
     }

    Send_remote_message(39); //compass found
    Send_remote_powerinfo(system_power_voltage, system_power_current, system_power_used);
#endif

#if defined USE_BMP390
//上面是i2c的板子, 现在是spi，速度更快
	//1M速度下，138微秒。10M速度下52微秒。i2c下要400多微秒。后来用pio优化的i2c要大概200微秒。
     bmp390.Initialize(BMP390_SPI_PORT, BMP390_PIN_SCK, BMP390_PIN_MISO, BMP390_PIN_MOSI, BMP390_PIN_CS, 10000000, 1);
     b=bmp390.Reset(); 

     if(!b){
        printf("bmp390 failed\n");
        //led_blink(30, 50);
        led_blink(12, 100);
        Send_remote_message(7);
        return 0; //不能进行下去。
     }
    //中断接收脚初始化。
     gpio_init(BMP390_PIN_INT);
     gpio_set_dir(BMP390_PIN_INT, GPIO_IN);

    //中断处理设置。
     gpio_set_irq_enabled_with_callback(BMP390_PIN_INT, GPIO_IRQ_EDGE_RISE, true, on_gpio_irq_callback);      
    //^^以上初始化代码^^
    //printf("bmp init\n");
    Send_remote_message(8);
    Send_remote_powerinfo(system_power_voltage, system_power_current, system_power_used);
#endif

    //数据准备期。主要是校准Imu的陀螺仪，加速度计的偏离。
    //led_blink(2, 1000); //跳过前2秒的数据
    //sleep_ms(300);
    
    //数据校准应该在静态下做，最好是水平面上。不必每次起飞都校准。
    //可以依据指令去校准，校准后的数据写入flash特定位置，不校准时直接读取数据。
    read_arguements_from_flash();
    printf("read arg from flash, balm=%f\n", balance_main_ratio);

    Send_remote_message(10);

    char dbg[64];
    sprintf(dbg,"read calib temp=%3.2f, balm=%3.2f", imu_calib_temprature, balance_main_ratio);
    Send_remote_message(dbg);

    Send_remote_proper_info(); //老方法推送


    //Check_imu_data();
    //Check_tof_data();
    //Check_gps_data();
    //Check_baro_data();
    //basic_motor_check();

    //Do_magnet_calibration();
    //Check_compass_data();

    //printf("get in check\n");
    //Check_compass_and_power();


    //uart_gps.SendPGKC("$PGKC101,200*31<CR><LF>"); //国科微的GPS不能保存配置，每次启动后需要设置

    //寻找温度漂移曲线。大约耗时10分钟。将热风机吹热imu后放入冰箱，平置通电。等待10分钟。
    // raw_gyro_bias_with_temp_collect();
    // printf("trying write log\n");
    // write_log_to_flash();
    // printf("write log finish\n");
    // led_blink(10000, 1000);
    // return 0;


//应在这里处理数据初始化，等待GPS信号定位完成，然后等待遥控器，这里有两个问题，一个是校准命令处理，一个是起飞命令。
//因为校准命令需要监听信号量，所以如果要执行校准则不能启动微任务。所以部分宏任务需要放在这里完成。

    //Send_remote_proper_info(); //老方法推送
    sleep_ms(1000); //等待一秒，让传感器数据充满队列。

    //测试运行时间
    // multicore_launch_core1(micro_task_exec); //运行微任务
    // while (1)
    // {
    //     sleep_ms(1000);
    //     micro_task.taskid= _micro_task_id_on_remote; //进入遥控器控制态
    //     continue;
    // }
    

    
    basic_sensor_check(); //基础传感器数据检查。

    printf("pass sensor check\n");
    Send_remote_message("pass sensor check.");
    Send_remote_proper_info(); //老方法推送

    //remote_signal_test(); //test,检查遥控距离。

    led_blink(3, 200);
    
#if defined USE_GPS
    basic_gps_check(); //这里面同样监控了遥控器的校准命令。定位精度达到后即退出。
#if defined FULL_AUTO_RUN
    //全自动飞行没有电机确认，GPS准备好以后直接跳过去执行任务。
    goto full_auto_run;
#endif
#endif

    //电机交叉运行确认数据正常。
    //到这里连同GPS数据都已经准备好了。开始接受遥控命令。

getready:

#if defined USE_REMOTE_CTRL
    //如果有遥控，这里监听校准命令。
    //如果收到了起飞命令，则传递这个信号到宏任务直接起飞。
    Send_remote_message(26); //等待遥控命令。


    while(1) {
        
        bool b=sem_acquire_timeout_ms(&remote_info_to_action, 200);

        if(b) 
        {

            sem_reset(&remote_info_to_action, 0); //清理信号标记


            //这里主要是监听预定义任务信号。
            //之前这里还有定点方式设置，控高方式设置，暂时取消了。
            if(last_predefined_task!=0)
            {
                uint8_t cmd= last_predefined_task;
                last_predefined_task=0;

                if(cmd==_predefined_cmd_takeoff_compass)
                {
                    //起飞命令。
                    #if defined USE_HMC5883 ||defined USE_QMC5883
                    _initial_heading_update_method = head_compass0;
                    Send_remote_message("take off compass0");
                    break;
                    #else
                    Send_remote_message("no compass supoort");
                    #endif
                }
                else if(cmd==_predefined_cmd_takeoff_north)
                {
                    //起飞命令。
                    _initial_heading_update_method = head_north;
                    Send_remote_message("take off head north");
                    break;
                }
                else if(cmd==_predefined_cmd_takeoff_south)
                {
                    //起飞命令。
                    _initial_heading_update_method = head_south;
                    Send_remote_message("take off head south");
                    break;
                }
                else if(cmd==_predefined_cmd_takeoff_east)
                {
                    //起飞命令。
                    _initial_heading_update_method = head_east;
                    Send_remote_message("take off head east");
                    break;
                }
                else if(cmd==_predefined_cmd_takeoff_west)
                {
                    //起飞命令。
                    _initial_heading_update_method = head_west;
                    Send_remote_message("take off head west");
                    break;
                }
                else if(cmd==_predefined_cmd_takeoff_east_south)
                {
                    //起飞命令。
                    _initial_heading_update_method = head_east_south;
                    Send_remote_message("take off head east south");
                    break;
                }
                else if(cmd==_predefined_cmd_takeoff_east_north)
                {
                    //起飞命令。
                    _initial_heading_update_method = head_east_north;
                    Send_remote_message("take off head east north");
                    break;
                }
                else if(cmd==_predefined_cmd_takeoff_west_north)
                {
                    //起飞命令。
                    _initial_heading_update_method = head_west_north;
                    Send_remote_message("take off head west north");
                    break;
                }
                else if(cmd==_predefined_cmd_takeoff_west_south)
                {
                    //起飞命令。
                    _initial_heading_update_method = head_west_south;
                    Send_remote_message("take off head west south");
                    break;
                }
                else if(cmd==45)
                {
                    Send_remote_message("about to test motor by remote!");
                    //burn_motor_test();
                    motor_test_by_remote();
                }
                else if(cmd==_predifined_cmd_check_tof)
                {
                    Send_remote_message("check tof module");
                    Check_tof_data();
                }
                else if(cmd==_predefined_cmd_imu_calib)
                {
                    //imu出场校准
                    Send_remote_message("calib comd accept");
                    Do_imu_calibration();
                }
                else if(cmd==_predefined_cmd_user_imu_calib)
                {
                    //imu用户校准
                    Send_remote_message("user calib accept");
                    Do_user_calibration();
                }
                else if(cmd==_predefined_cmd_compass_calib)
                {
                    //指南针校准
                    #if defined USE_HMC5883 ||defined USE_QMC5883
                    Send_remote_message("compass calib accept");
                    Do_magnet_calibration();
                    //break; //外地测试发现指南针校准后自动起飞，就是这里的问题。
                    #else
                    Send_remote_message("no compass supoort");
                    #endif
                }
                else if(cmd==_predefined_cmd_check_motor)
                {
                    Send_remote_message("check motor accept");
                    basic_motor_check();
                }
                else if(cmd==_predefined_cmd_force_stable_baro_ctrl)
                {
                    _force_stable_baro_height_control=true;
                    Send_remote_message("force stable baro ctrl");

                }
                else if(cmd==_predefined_cmd_ps0)
                {
                    power_control_solution_adjust(0);
                    logmessage("ps set to 0\n");
                    Send_remote_message("power control mode set to 0");
                }
                else if(cmd==_predefined_cmd_ps1)
                {
                    power_control_solution_adjust(1);
                    logmessage("ps set to 1\n");
                    Send_remote_message("power control mode set to 1");
                }
                else if(cmd==_predefined_cmd_ps2)
                {
                    power_control_solution_adjust(2);
                    logmessage("ps set to 2\n");
                    Send_remote_message("power control mode set to 2");
                }
                else if(cmd==_predefined_cmd_ps3)
                {
                    power_control_solution_adjust(3);
                    logmessage("ps set to 3\n");
                    Send_remote_message("power control mode set to 3");
                }
                else if(cmd==_predefined_cmd_ps4)
                {
                    power_control_solution_adjust(4);
                    logmessage("ps set to 4\n");
                    Send_remote_message("power control mode set to 4");
                }
                else if(cmd==_predefined_cmd_ps5)
                {
                    power_control_solution_adjust(5);
                    logmessage("ps set to 5\n");
                    Send_remote_message("power control mode set to 5");
                }
                else if(cmd==_predefined_cmd_ps6)
                {
                    power_control_solution_adjust(6);
                    logmessage("ps set to 6\n");
                    Send_remote_message("power control mode set to 6");
                }
                else if(cmd==_predefined_cmd_output_log)
                {
                    //输出日志信息。
                    Send_remote_message("output log");
                    read_log_from_flash();
                }
                else if(cmd==_predefined_cmd_takeover)
                {
                    //接管命令，这里不处理。
                    Send_remote_message(31);
                }
                else{
                    char dbg[64];
                    sprintf(dbg, "can't accept predefined %d cmd\n", cmd);
                    Send_remote_message(dbg);
                }

            }
            else if(remote_task_signal)
            {
                //在这个时候并不解析任务，但为了调试问题，这里解析。
                remote_task_signal=false;
                //printf("t=%s\n", last_task_content.c_str());
                Send_remote_message("got task:");
                Send_remote_message(last_task_content.c_str());
                float total_route;
                int n= Parse_Task_String(last_task_content, total_route);

                if(n>0) {
                    char buf[128];
                    sprintf(buf, "parse got %d tasks, route %5.1f", n, total_route);
                    Send_remote_message(buf);
                }
                else{
                    macro_exec.clear_task();
                    Send_remote_message("parse cmd failed");
                }

                last_task_content.clear();
            }

        }

        //在循环空闲时，读取GPS数据/姿态数据给遥控器。
        Send_remote_proper_info();
    }

#endif

full_auto_run:

    //让它执行微任务，主线执行宏任务
    Send_remote_message(11);
    multicore_launch_core1(micro_task_exec);

    sleep_ms(50); //确保微任务启动。

    Send_remote_message(12);
    macro_task_exec(); //进入这里面就执行起飞，要么就执行预定任务。

    //测试方向判断，打开GPS, 关闭宏任务，保持微任务运行，维持一段时间退出，保存日志。
    //等待GPS达到精度。
    //uart_gps.SendPGKC("$PGKC101,200*31<CR><LF>"); //国科微的GPS不能保存配置，每次启动后需要设置
 
    Send_remote_message(13);

    multicore_reset_core1(); //停止微任务运行，以便下次校准。

    Send_remote_message(14);

    //全部电量消耗。20231112+
    sprintf(dbg, "power used %6.1f\n", system_power_used);
    logmessage(dbg);
    //sprintf(dbg, "total distance=%f,%f\n", _total_gps_distance, _gps_distance_since_last_reset);
    //logmessage(dbg);

    printf("trying write log\n");
    write_log_to_flash();
 
    printf("write log finish\n");

    if(update_magnet)
    {
        write_arguements_to_flash();
    }

    led_blink(3, 500);

    Send_remote_message(15);
    Send_remote_proper_info(); //再次更新inair

    char tm[128];
    sprintf(tm, "total move=%6.1f", total_horizon_move/1000.0);
    Send_remote_message(tm);

    //考虑在这里将记录数据发送给遥控器。
    //这样读取日志不需要从飞控上做。

#if defined FULL_AUTO_RUN
    //如果是全自主飞行，这里就不再返回到等待遥控起飞命令上去。因为任务都执行完成了。
#else
    goto getready;
#endif
    return 0;
}



