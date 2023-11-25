/**
 * @file plot.cpp
 *
 * @brief This file is part of the `plot` node and contains the implementation of the `Robot` class.
 *
 * @details This file contains the implementation of the `Robot` class, which is used to subscribe
 * to the `/gazebo/model_states` and `/dead_reckoning` topics and update the robot's position and orientation.
 *
 * @version 1.0.0
 *
 * @date 2023-10-15
 *
 * @author Allan Souza Almeida
 *
 * @note This file is part of the `allan_husky` package.
 */

#include "ros/ros.h"
#include "Display.hpp"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "tf/tf.h"

class Robot
{
public:
    Robot(ros::NodeHandle *nh, Display *display)
    {
        model_states_sub = nh->subscribe("/gazebo/model_states", 100, &Robot::modelStatesCallback, this);
        estimated_states_sub = nh->subscribe("/dead_reckoning", 100, &Robot::estimatedStatesCallback, this);
        est_ekf_sub = nh->subscribe("/estimated_position", 100, &Robot::estimatedEKFCallback, this);
        this->display = display;
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
    {
        double robot_x = msg->pose[1].position.x;
        double robot_y = msg->pose[1].position.y;
        tf::Pose pose;
        tf::poseMsgToTF(msg->pose[1], pose);
        double robot_theta = tf::getYaw(pose.getRotation());
        display->setStates(robot_x, robot_y, robot_theta);
    }

    void estimatedStatesCallback(const geometry_msgs::Pose::ConstPtr &msg)
    {
        double robot_x = msg->position.x;
        double robot_y = msg->position.y;
        tf::Pose pose;
        tf::poseMsgToTF(*msg, pose);
        double robot_theta = tf::getYaw(pose.getRotation());
        display->setEstimates(robot_x, robot_y, robot_theta);
        display->calculateError();
    }

    void estimatedEKFCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        double robot_x = msg->linear.x;
        double robot_y = msg->linear.y;
        double robot_theta = msg->angular.z;
        display->setEKFEstimates(robot_x, robot_y, robot_theta);
    }

private:
    ros::Subscriber model_states_sub;
    ros::Subscriber estimated_states_sub;
    ros::Subscriber est_ekf_sub;
    Display *display;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "plot");
    ros::NodeHandle nh;
    Display display;
    Robot robot(&nh, &display);
    display.init("Simulação [Plot XY]", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1080, 720, false);
    while (display.running())
    {
        display.handleEvents();
        display.render();
        ros::spinOnce();
    }
    display.clean();
    ros::shutdown();
    return 0;
}