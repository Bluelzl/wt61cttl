#include <termios.h>
#include <ros/ros.h>

#include "imu/WT61_IMU.h"
#include <fcntl.h>
#include <poll.h>
#include <vector>

class POSIX_WT61C_TTL
{
public:
    POSIX_WT61C_TTL(int &argc, char **&argv, ros::NodeHandle &_nh);
    ~POSIX_WT61C_TTL();
    void read(void);
    void print_test(void) const;
    bool imu_check(void) const;
    bool data_valid_check(void) const;
    void pub_imu_data(void) const;
    inline const std::vector<uint8_t> sumcrc(const std::vector<uint8_t> &_vec) const;
    inline const std::vector<uint8_t> sumcrc(const uint8_t *&_arr);

    void port_init(void);
    void port_open(void);

private:
    int _fd; // USB句柄
    char _USB_NAME[13] = "/dev/ttyUSB0";
    int _baudrate = 115200; // 波特率，默认9600

    unsigned char _usb_buf[50]; // 读取缓冲区

    std::vector<uint8_t> _read_buf;

    ros::Publisher _pub;
};