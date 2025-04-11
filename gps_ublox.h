#pragma once
#include <string>
#include "gps_info.h"

//小方板
//主板对应的接收口，gps发送口
#define GPS_UBLOX_UART_PORT (0)
#define GPS_UBLOX_TX_PIN 16
#define GPS_UBLOX_RX_PIN 17
#define GPS_UBLOX_BUADRATE (115200)

//新长条板，和通信接口做了交换
// #define GPS_UBLOX_UART_PORT (1)
// #define GPS_UBLOX_TX_PIN 8
// #define GPS_UBLOX_RX_PIN 9
// #define GPS_UBLOX_BUADRATE (115200)
//UBX Protocol

//ublox的问题：1.数据间隔不均匀，不可靠，每秒10次应该间隔100毫秒，这个波动很大。
//2.定位精度和其他的不同，很难达到1米以内，可能是标准不同。不能指望等到精度达到1米以内再起飞。
//室内靠窗长时间可以做到1.5米以内精度。多数都在2米。

class CGPS_Ublox
{
public:
    CGPS_Ublox() {
        _rbuf_tail=0;
    }
    //uart初始化。
    bool Initialize(int uart_port=GPS_UBLOX_UART_PORT, int pin_rx=GPS_UBLOX_RX_PIN, int pin_tx=GPS_UBLOX_TX_PIN);
    //uart读。
    bool Read(gps_info& info);//返回false无有效信息读取
    //pio推送数据
    bool Pio_PutChar(char c, gps_info& info); //pio调用，输入一个字符数据，如果有完整的信息则返回

    private:
    bool ParseBuffer(gps_info& info);
    bool NavPacket(uint8_t msgid, const char* payload, size_t payload_len, gps_info& info);
    bool Nav_pvt_Packet(uint8_t msgid, const char* payload, size_t payload_len, gps_info& info);
    //void Nav_odo_Packet(uint8_t msgid, const char* payload, size_t payload_len);
public:
    // static void dist_angle_by_gps(double lati_from, double logi_from, double lati_to, double logi_to, float& dist, float& angle)
    // {
    // #define ER 6380000
    // #define cratio  57.29578
    //     double dNorth = ER * ( lati_to - lati_from)/ cratio;
    //     double dEast = ER *(logi_to - logi_from) * cos(lati_from/ cratio)/ cratio;

    //     angle = atan2(dEast, dNorth); //弧度
    //     angle *= cratio; //转360度

    //     if (angle < 0) angle += 360; //转360度

    //     dist = sqrt(dNorth*dNorth + dEast * dEast);

    // #undef ER
    // #undef cratio
    // }

    // //给出基础坐标，目标的航向角和距离，计算目标的GPS坐标点。
    // //angle是航向角，指向北方为0
    // //输入输出角度参数全部为度数。
    // static void gps_by_dist_angle(double lati_base, double logi_base, float dist, float angle, double& lati, double& logi)
    // {
    // #define ER 6380000
    // #define cratio  57.29578
    //     float dNorth = cos(angle/ cratio)*dist;
    //     float dEast = sin(angle / cratio)*dist;
    //     float an = asin(dNorth / ER);
    //     float ae = asin(dEast / (cos(lati_base/ cratio)*ER));
    //     lati = lati_base + an * cratio;
    //     logi = logi_base + ae * cratio;
    // #undef ER
    // #undef cratio
    // }
private:
    char _rbuf[256];
    size_t _rbuf_tail;
    //std::string _rbuf; //长期运行可能导致碎片内存？
    int _uart_port;
};