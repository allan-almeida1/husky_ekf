/**
 * @file ekf_node.cpp
 *
 * @brief EKF node
 *
 * @details This node runs the EKF in ROS environment by using the KalmanFilter class
 *
 * @version 1.0.0
 *
 * @date 2023-11-29
 *
 * @author Allan Almeida
 *
 * @note This file is part of the `husky_ekf` package
 */

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