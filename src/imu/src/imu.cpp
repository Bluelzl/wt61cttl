#include "imu.hpp"

/**
 * @brief Construct a new imu wt61cttl::imu wt61cttl object
 *
 * @param _nh ros::NodeHandle
 */
IMU_WT61CTTL::IMU_WT61CTTL(ros::NodeHandle &_nh) : _flag(false), _fd(-1)
{
    printf("init start\n");
    // 初始化ros节点
    // ros::init();

    // 初始化publisher
    _pub = _nh.advertise<imu::WT61_IMU>("wt61_imu", 10);

    // serial
    // try
    // {
    //     port_init();
    // }
    // catch (serial::IOException &e)
    // {
    //     ROS_ERROR_STREAM("Unable to open port");
    //     _flag = ser.isOpen();
    // }

    // serial
}

/**
 * @brief Destroy the imu wt61cttl::imu wt61cttl object
 *
 */
IMU_WT61CTTL::~IMU_WT61CTTL()
{
    // 关闭串口
    ser.close();
    _pub.shutdown();
}

/**
 * @brief 读取串口数据
 *
 */
void IMU_WT61CTTL::read(void)
{

    if (ser.isOpen())
    {
        size_t n = ser.available(); // 获取当前缓冲区字节数
        if (n != 0)
        // if (true)
        {
            // 清空上次读取到的数据
            if (!_read_buf.empty())
            {
                _read_buf.clear();
            }
            n = ser.read(_read_buf, n);
            // printf("read %ld bytes\t", n);
            // printf("vector_size = %ld\t", _read_buf.size());
        }
    }
}

/**
 * @brief 测试打印数据
 *
 */
void IMU_WT61CTTL::print_test(void) const
{
    if (ser.isOpen())
    {
        printf("read[0] = %d\n", _read_buf[0]);
    }
}

bool IMU_WT61CTTL::imu_check(void) const
{
    return ser.isOpen();
}

/**
 * @brief 检验数据有效性
 *
 * @return true 当前数据有效
 * @return false 当前数据无效
 */
bool IMU_WT61CTTL::data_valid_check(void) const
{
    // WT61CTTL imu数据长度33bytes,顺序：加速度、角速度、角度（各11bytes）
    // 检查数据长度
    if (_read_buf.size() != 33)
    {
        return false;
    }
    // 检验数据是否乱码
    else if (_read_buf[0] != 0x55 || _read_buf[1] != 0x51 ||
             _read_buf[11] != 0x55 || _read_buf[12] != 0x52 ||
             _read_buf[22] != 0x55 || _read_buf[23] != 0x53)
    {
        return false;
    }
    // 检验数据和校验
    else if (sumcrc(_read_buf)[0] != _read_buf[10] ||
             sumcrc(_read_buf)[1] != _read_buf[21] ||
             sumcrc(_read_buf)[2] != _read_buf[32])
    {
        // printf("sumcrc = %d, read[11] = %d\n", sumcrc(_read_buf)[0], _read_buf[10]);
        ROS_ERROR("sumcrc error");
        return false;
    }
    else
    {
        // 是否完全可信？
        return true;
    }
}

/**
 * @brief publisher发送数据
 *
 */
void IMU_WT61CTTL::pub_imu_data(void) const
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
 * @brief 计算数据和校验
 *
 * @param _vec
 * @return const std::vector<uint8_t>
 */
inline const std::vector<uint8_t> IMU_WT61CTTL::sumcrc(const std::vector<uint8_t> &_vec) const
{
    uint8_t _num = 1;
    std::vector<uint8_t> _suncrc;
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

void IMU_WT61CTTL::port_init(void)
{
    try
    {
        ser.setPort(_USB_NAME);
        ser.setBaudrate(_baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);

        ser.open();
        if (_flag = ser.isOpen())
        {
            ROS_INFO("Port open.\n");
            ROS_INFO("Init end.\n");
        }
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port");
        _flag = ser.isOpen();
    }
}

void IMU_WT61CTTL::port_open(void)
{
    if (!ser.isOpen())
        ser.open();
}