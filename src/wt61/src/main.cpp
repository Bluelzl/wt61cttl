#include "wt61.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "IMU_WT61CTTL");
    ros::NodeHandle nh;

    POSIX_WT61C_TTL imu(argc, argv, nh);
    imu.port_init();
    ros::Rate rate(100); // 固定频率运行

    ROS_INFO("开始获取IMU数据");
    while (ros::ok())
    {
        if (!imu.imu_check())
        {
            imu.port_open();
        }
        else
        {
            // 获取imu数据
            imu.read();

            // test
            // imu.print_test();

            // 检验数据有效性
            if (imu.data_valid_check())
            {
                // ROS_INFO("data valid\n");
                // 数据有效进行发送
                imu.pub_imu_data();
            }
            else
            {
                // ROS_INFO("data invalid\n");
                ROS_ERROR("data invalid\n");
            }
        }
        // 睡眠
        rate.sleep();
    }
}