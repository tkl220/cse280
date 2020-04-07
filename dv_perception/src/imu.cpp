#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

#include <sstream>

static const char* topic_name = "imu";
static const int refresh_rate = 10;

int main(int argc, char **argv) {
    ros::init(argc, argv, topic_name);
    ros::NodeHandle handle;
    ros::Publisher imu_pub = handle.advertise<sensor_msgs::Imu>("imu", 1000);
    ros::Rate loop_rate(refresh_rate);

    int count = 0;
    while(ros::ok()) {
        sensor_msgs::Imu msg;
        std::stringstream ss;
        ss << "imu message" << count;
        msg.header.frame_id = ss.str();

        ROS_INFO("%s", msg.header.frame_id.c_str());
        imu_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}