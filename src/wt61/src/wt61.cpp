#include "wt61.hpp"

/**
 * @brief Construct a new posix wt61c ttl::posix wt61c ttl object
 *
 */
POSIX_WT61C_TTL::POSIX_WT61C_TTL(int &argc, char **&argv, ros::NodeHandle &_nh)
{
    // if(argc==2)
    // {
    //     _USB_NAME=argv[1];
    // }
    _pub = _nh.advertise<imu::WT61_IMU>("wt61_imu_posix", 10);
}

/**
 * @brief Destroy the posix wt61c ttl::posix wt61c ttl object
 *
 */
POSIX_WT61C_TTL::~POSIX_WT61C_TTL()
{
    // 关闭串口
    if (_fd > 0)
    {
        ::close(_fd);
    }
}

/**
 * @brief read
 *
 */
const size_t POSIX_WT61C_TTL::read(void)
{
    // _fd = ::open(_USB_NAME, O_RDWR | O_NOCTTY | O_NONBLOCK);
    size_t _n;
    _read_buf.clear();
    _n = ::read(_fd, _usb_buf, sizeof(_usb_buf));
    ROS_INFO("read %ld bytes data.", _n);

    if (_n > 0)
    {
        for (size_t i = 0; i < _n; i++)
        {
            _read_buf.push_back(_usb_buf[i]);
        }
    }
    return _n;
}

/**
 * @brief
 *
 */
void POSIX_WT61C_TTL::print_test(void) const {}

/**
 * @brief 检测串口状态
 *
 * @return true
 * @return false
 */
bool POSIX_WT61C_TTL::imu_check(void) const
{
    ROS_INFO("fd = %d", _fd);
    return _fd > 0;
}

/**
 * @brief 数据检验
 *
 * @return true
 * @return false
 */
bool POSIX_WT61C_TTL::data_valid_check(void) const
{
    // WT61CTTL imu数据长度33bytes,顺序：加速度、角速度、角度（各11bytes）
    // 检查数据长度
    if (_read_buf.size() != 33)
    {
        ROS_ERROR("data length error!");
        return false;
    }
    // 检验数据是否乱码
    else if (_read_buf[0] != 0x55 || _read_buf[1] != 0x51 ||
             _read_buf[11] != 0x55 || _read_buf[12] != 0x52 ||
             _read_buf[22] != 0x55 || _read_buf[23] != 0x53)
    {
        ROS_ERROR("data is gabled!");
        return false;
    }
    // 检验数据和校验
    else if (sumcrc(_read_buf)[0] != _read_buf[10] ||
             sumcrc(_read_buf)[1] != _read_buf[21] ||
             sumcrc(_read_buf)[2] != _read_buf[32])
    {
        // printf("sumcrc = %d, read[11] = %d\n", sumcrc(_read_buf)[0], _read_buf[10]);
        ROS_ERROR("sumcrc error!");
        return false;
    }
    else
    {
        // 是否完全可信？
        return true;
    }
}

/**
 * @brief Publisher发送消息
 *
 */
void POSIX_WT61C_TTL::pub_imu_data(void) const
{
    uint8_t _xl = 2;
    uint8_t _xh = 3;
    uint8_t _yl = 4;
    uint8_t _yh = 5;
    uint8_t _zl = 6;
    uint8_t _zh = 7;
    uint8_t _A = 0;
    uint8_t _B = 11;
    uint8_t _C = 22;

    imu::WT61_IMU data;
    data.Header.stamp = ros::Time::now();

    // ------------------数据转换---------------------
    if (_read_buf[_A] == 0x55 && _read_buf[_A + 1] == 0x51)
    {
        // 加速度信息
        data.linear_acceleration.x = (short)(((short)_read_buf[_A + _xh] << 8) | _read_buf[_A + _xl]) * 16.0 * g / 32768.0;
        data.linear_acceleration.y = (short)(((short)_read_buf[_A + _yh] << 8) | _read_buf[_A + _yl]) * 16.0 * g / 32768.0;
        data.linear_acceleration.z = (short)(((short)_read_buf[_A + _zh] << 8) | _read_buf[_A + _zl]) * 16.0 * g / 32768.0;
    }
    if (_read_buf[_B] == 0x55 && _read_buf[_B + 1] == 0x52)
    {
        // 角速度信息
        data.angular_velocity.x = (short)(((short)_read_buf[_B + _xh] << 8) | _read_buf[_B + _xl]) * 2000.0 / 32768.0;
        data.angular_velocity.y = (short)(((short)_read_buf[_B + _yh] << 8) | _read_buf[_B + _yl]) * 2000.0 / 32768.0;
        data.angular_velocity.z = (short)(((short)_read_buf[_B + _zh] << 8) | _read_buf[_B + _zl]) * 2000.0 / 32768.0;
    }
    if (_read_buf[_C] == 0x55 && _read_buf[_C + 1] == 0x53)
    {
        // 角度信息
        data.angular.x = (short)(((short)_read_buf[_C + _xh] << 8) | _read_buf[_C + _xl]) * 180.0 / 32768.0;
        data.angular.y = (short)(((short)_read_buf[_C + _yh] << 8) | _read_buf[_C + _yl]) * 180.0 / 32768.0;
        data.angular.z = (short)(((short)_read_buf[_C + _zh] << 8) | _read_buf[_C + _zl]) * 180.0 / 32768.0;
    }
    // ------------------数据发送---------------------
    _pub.publish(data);
    ROS_INFO("message published.\n");
}

/**
 * @brief 计算数据校验和
 *
 * @param _vec 数据
 * @return const std::vector<uint8_t> 校验和
 */
inline const std::vector<uint8_t> POSIX_WT61C_TTL::sumcrc(const std::vector<uint8_t> &_vec) const
{
    uint8_t _num = 1;
    std::vector<uint8_t> _suncrc;
    if (!_suncrc.empty())
    {
        _suncrc.clear();
    }
    std::vector<std::vector<uint8_t>> vec;
    if (_vec.size() == 33)
    {
        std::vector<uint8_t>::const_iterator ite1 = _vec.begin();
        std::vector<uint8_t>::const_iterator ite2 = _vec.begin() + 11;
        std::vector<uint8_t>::const_iterator ite3 = _vec.begin() + 22;
        std::vector<uint8_t>::const_iterator ite4 = _vec.end();

        // 分割成三部分指令，方便求数据和校验
        vec = {std::vector<uint8_t>(ite1, ite2),
               std::vector<uint8_t>(ite2, ite3),
               std::vector<uint8_t>(ite3, ite4)};
        _num = 3;
    }
    for (uint _i = 0; _i < _num; _i++)
    {
        uint8_t temp = 0;
        for (auto it = vec[_i].begin(); it < vec[_i].end() - 1; it++)
            temp += *it;

        _suncrc.push_back(temp);
    }
    return _suncrc;
}

/**
 * @brief 计算数据校验和
 *
 * @param _vec 数据
 * @return const uint8_t 校验和
 */
inline const std::vector<uint8_t> POSIX_WT61C_TTL::sumcrc(const uint8_t *_arr)
{
    // uint8_t _num = sizeof(_vec);

    uint8_t _num = 1;
    _read_buf.clear();
    for (uint8_t i = 0; i < sizeof(_arr); i++)
    {
        _read_buf.push_back(*(_arr + i));
    }

    std::vector<uint8_t> _suncrc;
    std::vector<std::vector<uint8_t>> vec;
    if (_read_buf.size() == 33)
    {
        std::vector<uint8_t>::const_iterator ite1 = _read_buf.begin();
        std::vector<uint8_t>::const_iterator ite2 = _read_buf.begin() + 11;
        std::vector<uint8_t>::const_iterator ite3 = _read_buf.begin() + 22;
        std::vector<uint8_t>::const_iterator ite4 = _read_buf.end();

        // 分割成三部分指令，方便求数据和校验
        vec = {std::vector<uint8_t>(ite1, ite2),
               std::vector<uint8_t>(ite2, ite3),
               std::vector<uint8_t>(ite3, ite4)};
        _num = 3;
    }
    for (uint _i = 0; _i < _num; _i++)
    {
        uint8_t temp = 0;
        for (auto it = vec[_i].begin(); it < vec[_i].end() - 1; it++)
            temp += *it;

        _suncrc.push_back(temp);
    }
    return _suncrc;
}

/**
 * @brief 初始化串口POSIX
 *
 */
const bool POSIX_WT61C_TTL::port_init(void)
{
    _fd = ::open(_USB_NAME, O_RDWR | O_NOCTTY | O_NONBLOCK); //

    if (_fd < 0)
    {
        printf("ERROR opening UART %s, aborting..\n", _USB_NAME);
        return false;
    }
    else
    {
        printf("Writing to UART %s\n", _USB_NAME);
        printf("_fd : %d\n", _fd);
    }

    int speed;

    switch (_baudrate)
    {
    case 9600:
        speed = B9600;
        break;

    case 19200:
        speed = B19200;
        break;

    case 38400:
        speed = B38400;
        break;

    case 57600:
        speed = B57600;
        break;

    case 115200:
        speed = B115200;
        break;

    case 460800:
        speed = B460800;
        break;

    case 921600:
        speed = B921600;
        break;

    case 1000000:
        speed = B1000000;
        break;

    default:
        printf("ERR: baudrate: %d\n", _baudrate);
        return false;
    }

    struct termios uart_config;

    int termios_state;

    tcgetattr(_fd, &uart_config);

    uart_config.c_oflag &= ~ONLCR;
    uart_config.c_cflag &= ~(CSTOPB | PARENB);
    uart_config.c_cflag &= ~CSIZE;
    uart_config.c_cflag |= CS8;
    uart_config.c_cflag &= ~CRTSCTS;
    uart_config.c_cflag |= CREAD;
    uart_config.c_cflag |= CLOCAL;

    uart_config.c_oflag &= ~OPOST;
    uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    uart_config.c_cc[VTIME] = 1;
    uart_config.c_cc[VMIN] = 1;

    // tcflush(_fd, TCIFLUSH);

    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0)
    {
        printf("ERR: %d (cfsetispeed)\n", termios_state);
        ::close(_fd);
        return false;
    }

    else if ((termios_state = cfsetospeed(&uart_config, speed)) < 0)
    {
        printf("ERR: %d (cfsetospeed)\n", termios_state);
        ::close(_fd);
        return false;
    }

    else if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0)
    {
        printf("ERR: %d (tcsetattr)\n", termios_state);
        ::close(_fd);
        return false;
    }
    else
    {
        return true;
    }
}

/**
 * @brief 打开串口POSIX
 *
 * @return true
 * @return false
 */
const bool POSIX_WT61C_TTL::port_open(void)
{
    _fd == ::open(_USB_NAME, O_RDWR | O_NOCTTY | O_NONBLOCK);
    return (_fd > 0);
}