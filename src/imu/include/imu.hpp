#include <termios.h>
#include <ros/ros.h>
#include <fcntl.h>
#include <poll.h>
#include <serial/serial.h>
#include <vector>
// #include <Publisher.h>
#include "imu/WT61_IMU.h"
#pragma once

#define g 9.80000000000

class IMU_WT61CTTL
{
public:
    IMU_WT61CTTL(ros::NodeHandle &_nh);
    ~IMU_WT61CTTL();
    void read(void);
    void print_test(void) const;
    bool imu_check(void) const;
    bool data_valid_check(void) const;
    void pub_imu_data(void) const;
    inline const std::vector<uint8_t> sumcrc(const std::vector<uint8_t> &_vec) const;

    void port_init(void);
    void port_open(void);

private:
    bool _flag;

    int _fd; // USB句柄
    char _USB_NAME[13] = "/dev/ttyUSB0";
    int _baudrate = 115200; // 波特率，默认9600

    unsigned char _usb_buf[50]; // 读取缓冲区

    std::vector<uint8_t> _read_buf;
    serial::Serial ser;

    ros::Publisher _pub;
};
