#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"

#include <sstream>

static const char* topic_name = "camera";
static const int refresh_rate = 10; /* 10Hz for the moment */

int main(int argc, char **argv) {
    ros::init(argc, argv, topic_name);
    ros::NodeHandle handle;

    ros::Publisher camera_pub = handle.advertise<sensor_msgs::CameraInfo>("camera_info", 1000); /* TODO: maybe increase the queue size for large messages? */
    ros::Rate loop_rate(refresh_rate);

    int count = 0;
    while(ros::ok()) {
        std_msgs::Header header;
        sensor_msgs::CameraInfo msg;
        std::stringstream ss;

        ss << "CameraInfo message header" << count;
        msg.header.frame_id = ss.str();

        ROS_INFO("%s", msg.header.frame_id.c_str());
        camera_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}