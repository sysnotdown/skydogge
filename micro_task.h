
//微观任务定义。
#pragma once

enum _micro_task_id
{
    _micro_task_id_nop=0,  //不执行任何动作
    _micro_task_id_ground_up=1, //起飞
    _micro_task_id_keep_stable_withyaw=2, //保持姿态稳定，保持航向角度稳定。
    _micro_task_id_vertical_landing=3, //垂直降落，
    _micro_task_id_move_around=5, //前后左右平移， 四个操作主要为了微调位置
    _micro_task_id_move_ahead_withyaw=6, //前飞，盯住某航向角，如果有偏自动纠偏
    _micro_task_id_on_remote=7, //听从遥控指挥。灵活控制。
    _micro_task_id_idle=9, //微任务没有目的性，仅保持基本的平衡，水平定点，操作主要由宏任务来执行
    //_micro_task_id_board_test=12, //板载测试
    //_micro_task_id_time_test=13, //用来测试计算时间。
    _micro_task_id_pin_to_location=15, //这个微任务用来盯住gps目标值，也可用来移动到目标值。
    _micro_task_id_vertical_landing_location=17, //带目标位置的降落任务。

};

enum _micro_task_status
{
    _micro_task_status_none=0,  //无信息
    _micro_task_status_inprogress=1, //正在进行
    _micro_task_status_done=2,  //完成
    _micro_task_status_fail=3,  //失败
    _micro_task_status_shutdown=4, //遥控器控制专用状态，为防止自动任务退出时标记done传递到遥控控制里导致遥控流程不工作而坠机。
};

class _micro_task
{
    public:
    _micro_task() {
        taskid=_micro_task_id_nop;
        yaw=0;
        pitch=0;
        roll=0;
        tof_height=0;
        vspeed=0;
        yawspeed=0;
        help_takeoff=false;
        tof_dominate=true;
        status=_micro_task_status_none;

        oflow_anchor_x=false; //光流锚点
        oflow_anchor_y=false;
        gps_anchor=false;

        height_keep_switch=0; //高度控制开关0，1，
        height_test_switch=0; //高度测试切换开关，用于切换不同的气压控高。0为老的，1为测试的。
        position_keep_switch=1; //位置保持开关,1=gps,0=光流，2关闭定点
        gps_height_press_bias=0; //gps/气压偏差。
        gps_height_press_bias_last_update=0;
        gps_bad_mark=false;
        height_break_tmark=0;
        smooth_landing=0;
        smooth_landing_triger_reason=0;
        gps_ref_height=0;
        gps_ref_height_press_bias=0; 
        air_press_height_relation_last_update=0; //20241007+
    }

    public:

    _micro_task_id taskid;
    //航向角, degree，指定这个角度可以旋转机身到指向这个角度
    float yaw;  

    //前倾角, degree，前飞时，指定一个负角度[-1,-15]应该都可以，太大的角度飞机不容易保持高度
    //后飞时，指定一个正角度[1,15]
    int8_t pitch;  //直接存放遥控数据[-64-64]，应用时/3.0换算为浮点角度

    //滚转角, degree，右飞或左飞需要指定这个角度，飞机会盯住这个角度飞行。
    int8_t roll;  //直接存放遥控数据[-64-64]，应用时/3.0换算为浮点角度

    //每隔20秒或30秒，对比press和初始press的差异，换算为高度，再对比当前gps高度和初始gps高度，也换算为高差
    //然后推算出气压的误差值，去调节这个误差值。在这里，以气压的高度为预期设定高度，通过gps高度去矫正长期偏差。
    float press; //气压 pa，用来盯住高度稳定高度, 如果是ground_up任务，这里是设定的目标高度压力

    float tof_height; //tof高度设定

    //20250122 修改为int16类型，这样在降落时可以允许设定超过12.7m/s的速度，否则会出错反向运行。
    int16_t vspeed; //垂直速度指示，负值是向上，正值是向下，0保持高度不动。1个整数差代表0.1m/s的速度差。

    //yaw角的调节速度，来源于遥控器控制。>0 逆时针旋转yaw角。之前的yaw角调节在宏任务里处理。
    //现在下沉到微任务处理上来，balance里自动根据这个速度参数调节yaw角度。
    //这个参数指示了旋转速度，并没有特定的目标角度。如果需要一个特定的目标角度，可以直接设置yaw.
    int8_t yawspeed; //正负各有128级，够用了。
    
    uint32_t timemark; //时间标签，辅助控制垂直速度。记录开始速度调节的时间

    //20230428+ 新增两个标记对于于遥控按钮，目的是方便遥控穿透控制起飞和降落，而不是抛给自动任务。
    bool  help_takeoff; //起飞辅助标记，有这个标记后台自动在一定高度内增加垂直力量。达到高度后标记被清除。
    bool  tof_dominate; //true=tof控高，false=气压控高。

    //gps高度的气压偏离。首先以气压为高度准绳，（设定气压-起飞气压）/12.0当作是设定高度。
    //随着时间和位置的变化，（设定气压-起飞气压）/12.0得到的高度数据和预期的发生偏差。
    //这个偏差值就记录在这里。每隔一段时间，依据GPS传来的高度数据对比气压数据去计算这个偏差。
    //然后气压控高里面单独附加这个偏差上去处理，并不去修改设定的气压数据。
    //在GPS高差小于20米时，这个设置为0，不使用偏差纠正。高于20米时才使用。
    float gps_height_press_bias; 
    uint32_t gps_height_press_bias_last_update;

    float gps_ref_height; //gps参考高度，辅助气压定高，气压高度有偏差时用于调节高度。
    float gps_ref_height_press_bias; //由gps高度差计算出合适的气压调节差。

    //位置稳定辅助偏角, 由balance函数参考光流的数据来设定调节。使水平面位置稳定。
    bool oflow_anchor_x; //锚点调节
    bool oflow_anchor_y; //锚点调节

    bool gps_anchor; //GPS锚定开关，机头方向确定且速度低于2m/s时打开。

    double target_gps_latitude; //锚定位置/目标位置，遥控锚定点和自动任务目标共用
    double target_gps_logitude; 
    float  target_refer_gps_height; //目标点参考高度，用于控制降落速度，高差大于100米时，降落速度可以比较快。
    float  target_distance_tolerance; //水平位置距离容差，小于这个值代表已经到达位置。总是大于0，高精度可以设置为1.5左右。
    double start_gps_latitude; //pin任务的起点位置 20240720+
    double start_gps_logitude; //pin任务的起点位置 20240720+

    //辅助起飞时同时设定两个参数。
    float help_takeoff_target_press; //辅助起飞的目标气压，高于这个气压时，持续增加升力，到达后不起作用。
    float help_takeoff_target_height; //辅助起飞tof目标高度。


    uint8_t position_keep_switch; //水平位置保持开关。1,GPS，0，光流， 2, 关闭定点。
    uint8_t height_keep_switch; //控高开关，0: tof to baro, 1: only baro
    uint8_t height_test_switch; //高度测试开关，0是老的控高。
    _micro_task_status status;  //进行状态，回传

    bool gps_bad_mark; //GPS信号问题标记。true表示GPS信号中断。

    float power_limit; //功率限制，瓦，用于定点航行时或自动爬高时，避免过度消耗能量。

    //20240223+
    uint32_t height_break_tmark; //时间记录，防止因机身震荡始终杀不住车。
    uint32_t smooth_landing; //平滑降落的起始时间戳。 20240226+ 由带降落检测的控高函数使用。
    uint8_t smooth_landing_triger_reason; //激活自动降落的原因，0，无原因无记录，1，tof测距很小，2，垂直震动很大，3，机身侧翻，4，推力持续偏低并无速度。

    bool heading_forward; //是否需要在移动时，将头部指向目标点，通常出于摄影需要可以设置这个标记。
    uint32_t air_press_height_relation_last_update;
};




