#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <sstream>

static const char* topic_name = "lidar";
static const int refresh_rate = 10;

int main(int argc, char **argv) {
    ros::init(argc, argv, topic_name);
    ros::NodeHandle handle;
    ros::Publisher lidar_pub = handle.advertise<sensor_msgs::LaserScan>("lidar", 1000);
    ros::Rate loop_rate(refresh_rate);

    int count = 0;
    while (ros::ok()) {
        sensor_msgs::LaserScan msg;
        std::stringstream ss;
        ss << "LIDAR message" << count;
        msg.header.frame_id = ss.str();

        ROS_INFO("%s", msg.header.frame_id.c_str());
        lidar_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}