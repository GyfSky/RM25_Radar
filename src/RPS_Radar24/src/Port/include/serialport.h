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

#define TRUE 1
#define FALSE 0
/**
 *@brief 串口协议格式, 通信方式为串口，配置为：波特率 115200，8 位数据位，1 位停止位，无硬件流控，无校验位
 *
 */

class SerialPort
{
private:
    int last_fd; //上一次串口号
    int speed,dataBits, stopBits, parity;
    char *port_path;

public:
    int fd;      //串口号
    int mode;

    SerialPort() = default;
    bool initSerialPort(const char *port_path,std::string config_path);
    bool get_Mode();
    bool withoutSerialPort();

    void set_Brate();
    int  set_Bit();
    void closePort();
};

#endif // SERIALPORT_H
