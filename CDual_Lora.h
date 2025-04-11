#pragma once
#include <string>
#include <vector>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "pico/multicore.h"
#include "remote.h"
#include "gps_info.h"
#include "skydogge.h"

//v8板子使用自制的dual-lora通信板，通过uart发送接收
#define DUAL_LORA_UART_PORT (1)
#define DUAL_LORA_PIN_RX (9)
#define DUAL_LORA_PIN_TX (8)
#define DUAL_LORA_BAUDRATE (115200)

//新长条板和GPS接口做了交换
// #define DUAL_LORA_UART_PORT (0)
// #define DUAL_LORA_PIN_RX (17)
// #define DUAL_LORA_PIN_TX (16)
// #define DUAL_LORA_BAUDRATE (115200)

class CDual_Lora{

    public:
    CDual_Lora() {
        _rbuf_tail=0;
    }
    void Initialize(int8_t uport= DUAL_LORA_UART_PORT, int pin_rx=DUAL_LORA_PIN_RX, int pin_tx=DUAL_LORA_PIN_TX);
    //bool OnReceive(remote_info& info); //由接收数据引发中断调用这个来读数据。
    uint8_t OnReceive(remote_info& info, uint8_t& predefined_task, std::string& taskcont);

    void SendPredefinedMessage(uint8_t id); //发送预定类型的消息。
    void SendPowerInfo(float power, float amp, float consume);
    void SendMessage(std::string msg); //发送消息。
    void SendGpsInfo(gps_info& gps); 
    void SendBaroInfo(float press, float temp);  
    void SendMotorInfo(float m0, float m1, float m2, float m3); 
    void SendImuInfo(float pitch, float roll, float yaw, float heading);
    void SendStatus(bool inair, bool gps_bad, bool on_remote, uint16_t last_sig); 

    protected:
    int8_t _uport;

    critical_section_t _send_critical_section;
    //这个接收缓存可能要改，要避免所有动态内存分配。
    //std::string _rdbuf;

    char _rbuf[256];
    size_t _rbuf_tail;
};