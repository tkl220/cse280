#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include <sstream>

/* Global constants */
static const char* topic_name = "odometer";
static const int refresh_rate = 10; /* in Hz */

/**
 * This file sends dummy odometry messages for now 
 * Adapted from the pub/sub tutorial to use odometry messages
 */
int main(int argc, char** argv) {
    /* Initialization */
    ros::init(argc, argv, topic_name);
    ros::NodeHandle handle;
    // Params: topic name, buffer(message queue) size - experiment with this in simulation?
    ros::Publisher odo_pub = handle.advertise<nav_msgs::Odometry>("odometry", 1000);
    ros::Rate loop_rate(refresh_rate);

    /* Message sending */
    int count = 0;
    while (ros::ok()) {
        nav_msgs::Odometry msg;
        std::stringstream ss;
        ss << "this is an odometry message " << count;
        msg.child_frame_id = ss.str(); /* I guess this is ok but what about the other fields? */

        ROS_INFO("%s", msg.child_frame_id.c_str());
        odo_pub.publish(msg);

        // Waste a little time? or match the loop rate manually?
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}