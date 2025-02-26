#ifndef SERIALPORT_H
#define SERIALPORT_H
/**
 *@class  SerialPort
 *@brief  set serialport,recieve and send
 *@param  int fd
 */
#include <atomic>
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/netlink.h>

#include <iostream>
#include <vector>

#include "Content.h"

#include <yaml-cpp/yaml.h>

#include<stdlib.h>


//#include <bitset>
//#include <algorithm>



//using namespace std;

#define TRUE 1
#define FALSE 0

//#pragma pack(1) // 设置结构体按1字节对齐
//
//
//typedef struct _packed {
//
//    uint16_t target_robot_id;
//    float target_position_x;
//    float target_position_y;
//} map_robot_data_t;
//
//#pragma pack() // 恢复默认对齐方式


/**
 *@brief 串口协议格式, 通信方式为串口，配置为：波特率 115200，8 位数据位，1 位停止位，无硬件流控，无校验位
 *
 */

class SerialPort
{
private:
//    int fd;      //串口号
    int last_fd; //上一次串口号
    int speed,dataBits, stopBits, parity;
    char *port_path;

public:
    int fd;      //串口号

    int mode;

    SerialPort() = default;
    bool initSerialPort(const char *port_path);
    bool get_Mode();
    bool withoutSerialPort();

    void set_Brate();
    int  set_Bit();
    void closePort();
//    bool sendData(unsigned char *Tmap_data);



//    atomic_bool need_init = true;
//    int baud;

//    SerialPort(const string ID, const int BUAD);
//    SerialPort(int BUAD){baud = BUAD; }
//    void setBUAD(int BUAD){baud = BUAD; }
//
//    SerialPort(char *);
//    void TransformData(const VisionData &data); //主要方案
//    void Radar_TransformData(const BeforeData &beforeData,const Ext_client_map_command_t &extClientMapCommand);   //主要方案
//    void fly_warning(const BeforeData &beforeData,const graphic_data_struct_t &graphicDataStruct);
//    void sendToOther(const BeforeData &beforeData,const ext_student_interactive_header_data_t &extStudentInteractiveHeaderData);
//
//    void send_U();
//    void send_Y();
//    void send_Z();

};
//
//typedef  struct
//{
//    //6
//    uint16_t_uchar data_cmd_id;
//    uint16_t_uchar sender_ID;
//    uint16_t_uchar receiver_ID;
////    //3
//    unsigned char graphic_name1;
//    unsigned char graphic_name2;
//    unsigned char graphic_name3;
//    //4
//    uint32_t operate_tpye:3;
//    uint32_t graphic_tpye:3;
//    uint32_t layer:4;
//    uint32_t color:4;
//    uint32_t start_angle:9;
//    uint32_t end_angle:9;
//    //4
//    uint32_t width:10;
//    uint32_t start_x:11;
//    uint32_t start_y:11;
////    //4
////    uint32_t radius:10;
////    uint32_t end_x:11;
////    uint32_t end_y:11;
//
//
//} graphic_data_struct_t;
//
//
//
//typedef struct
//{
//    string id;
//    string alias;
//    string path;
//} Device;
//
//
//
//typedef struct
//{
//    uint16_t_uchar data_length;
//    uint16_t_uchar cmd_id;
//
//} BeforeData;
//
//
//

//
//typedef struct
//{
//    uint8_t mark_hero_progress;
//    uint8_t mark_engineer_progress;
//    uint8_t mark_standard_3_progress;
//    uint8_t mark_standard_4_progress;
//    uint8_t mark_standard_5_progress;
//    uint8_t mark_sentry_progress;
//}Radar_mark_data_t;

#endif // SERIALPORT_H
