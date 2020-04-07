#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"

/* Callbacks ensure receipt of perception messages */
void odoCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->child_frame_id.c_str());
}

void cameraCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->header.frame_id.c_str());
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->header.frame_id.c_str());
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->header.frame_id.c_str());
}

/**
 * Subscribes to all sensor messages and passes them to mapping
 * Adapted from the pub/sub tutorial
 */ 
int main(int argc, char **argv) {
    ros::init(argc, argv, "perception_aggregator");
    ros::NodeHandle handle;

    ros::Subscriber odo_sub = handle.subscribe("odometry", 1000, odoCallback);
    ros::Subscriber cam_sub = handle.subscribe("camera_info", 1000, cameraCallback);
    ros::Subscriber imu_sub = handle.subscribe("imu", 1000, imuCallback);
    ros::Subscriber lidar_sub = handle.subscribe("lidar", 1000, lidarCallback);

    ros::spin();

    return 0;
}