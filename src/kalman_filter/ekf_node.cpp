#include "kalman_filter.hpp"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf_node");
    ros::NodeHandle nh;
    KalmanFilter kalmanFilter(&nh);
    ros::spin();
    return 0;
}