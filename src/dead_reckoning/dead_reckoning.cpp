/**
 * @file dead_reckoning.cpp
 *
 * @brief This file is part of the `dead_reckoning` node and contains the implementation of the `Controller` class.
 *
 * @details This file contains the implementation of the `Controller` class, which is used to subscribe
 * to the `/gazebo/model_states` and `/joint_states` topics and publish to the `/husky_velocity_controller/cmd_vel` topic.
 * This node makes the robot move in a semi-circle by publishing a constant linear and angular velocity to the
 * `/husky_velocity_controller/cmd_vel` topic. The ground truth position and orientation are saved to a json file.
 * The odometry is calculated by integrating the linear and angular velocities and is also saved to a json file.
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
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"
#include "tf/tf.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>

class Controller
{
public:
    Controller(ros::NodeHandle *nh)
    {
        duration = ros::Duration(10); // Run for 10 seconds
        model_states_sub = nh->subscribe("/gazebo/model_states", 100, &Controller::modelStatesCallback, this);
        joint_states_sub = nh->subscribe("/joint_states", 100, &Controller::jointStatesCallback, this);
        cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 100);
        odom_pub = nh->advertise<geometry_msgs::Pose>("/dead_reckoning", 100);
        stop_ekf_pub = nh->advertise<std_msgs::Bool>("/stop_ekf", 100);
        est_vel_pub = nh->advertise<geometry_msgs::Twist>("/estimated_velocity", 100);
        timer = nh->createTimer(ros::Duration(0.1), &Controller::timerCallback, this);
        dt = 0.1;
        odom_timer = nh->createTimer(ros::Duration(dt), &Controller::odomCallback, this);
        while (ros::Time::now().toSec() == 0.0)
            ; // wait for time to be initialized
        start_time = ros::Time::now();
    }

    /**
     * @brief Callback function for the model_states topic. This commands
     *
     * @param msg   Message from the topic
     */
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
    {
        ros::Time current_time = ros::Time::now();
        ros::Duration elapsed_time = current_time - start_time;
        // Run for 10 seconds
        if (elapsed_time.toSec() <= duration.toSec())
        {
            current_gt_pos = msg->pose[1].position; // get ground truth current position
            gt_pos.push_back(current_gt_pos);       // append to gt_pos vector
            gt_time.push_back(ros::Time::now());    // append current simulation time
            // Transform Quaternion to yaw
            tf::Pose pose;
            tf::poseMsgToTF(msg->pose[1], pose);
            current_gt_theta = tf::getYaw(pose.getRotation()); // get ground truth theta
            gt_theta.push_back(current_gt_theta);              // append it to gt_theta cevtorvector
        }
        // Then stop the robot, save the measurements and exit
        else
        {
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub.publish(cmd_vel);
            saveToFile(gt_pos, gt_pos, gt_theta, gt_time, "/home/allan/catkin_ws/src/allan_husky/gt_data.json");
            saveToFile(odom_pos, odom_pos, odom_theta, odom_time, "/home/allan/catkin_ws/src/allan_husky/odom_data.json");
            std_msgs::Bool stop_ekf;
            stop_ekf.data = true;
            stop_ekf_pub.publish(stop_ekf);
            ros::shutdown();
        }
    }

    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
    {
        joint_states.push_back(*msg);
    }

    /**
     * @brief Callback function for the publisher timer
     *
     * @param event Timer event
     */
    void timerCallback(const ros::TimerEvent &event)
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.2;
        cmd_vel.angular.z = 0.2;
        // current_velocities = cmd_vel;
        cmd_vel_pub.publish(cmd_vel);
        double vl = cmd_vel.linear.x - (0.545 * cmd_vel.angular.z) / 2;
        double vr = cmd_vel.linear.x + (0.545 * cmd_vel.angular.z) / 2;
        double alpha = 0.98;
        double xicr = 0.49;
        double v = (alpha / 2) * (vl + vr);
        double w = (alpha * 0.5 / xicr) * (vr - vl);
        current_velocities.linear.x = v;
        current_velocities.angular.z = w;
        est_vel_pub.publish(current_velocities);
    }

    /**
     * @brief Callback function for the odom_timer
     *
     * @param event Timer event
     */
    void odomCallback(const ros::TimerEvent &event)
    {
        calculateOdom();
    }

private:
    ros::Subscriber model_states_sub, joint_states_sub;
    ros::Publisher cmd_vel_pub, odom_pub, est_vel_pub, stop_ekf_pub;
    ros::Time start_time;
    ros::Duration duration;
    ros::Timer timer, odom_timer;
    std::vector<geometry_msgs::Point> gt_pos, odom_pos;
    std::vector<double> gt_theta, odom_theta;
    std::vector<ros::Time> gt_time, odom_time;
    std::vector<sensor_msgs::JointState> joint_states;
    geometry_msgs::Point current_gt_pos;
    geometry_msgs::Twist current_velocities;
    double current_gt_theta;
    double dt;
    bool reset;

    /**
     * @brief Save the ground truth and odometry data to json files
     */
    // void saveToFile()
    // {
    //     std::ofstream gt_file, odom_file;
    //     gt_file.open("/home/allan/catkin_ws/src/allan_husky/gt_data.json");
    //     if (gt_file.is_open())
    //     {
    //         gt_file << "[";
    //         for (int i = 0; i < gt_pos.size(); i++)
    //         {
    //             if (i == gt_pos.size() - 1)
    //                 gt_file << "{\"x\": " << gt_pos[i].x << ", \"y\": "
    //                         << gt_pos[i].y << ", \"theta\": " << gt_theta[i] << ", \"timestamp\": " << gt_time[i].toSec() << "}";
    //             else
    //             {
    //                 gt_file << "{\"x\": " << gt_pos[i].x << ", \"y\": "
    //                         << gt_pos[i].y << ", \"theta\": " << gt_theta[i]
    //                         << ", \"timestamp\": " << gt_time[i].toSec() << "},";
    //             }
    //         }
    //         gt_file << "]";
    //         gt_file.close();
    //     }

    //     odom_file.open("/home/allan/catkin_ws/src/allan_husky/odom_data.json");
    //     if (odom_file.is_open())
    //     {
    //         odom_file << "[";
    //         for (int i = 0; i < odom_pos.size(); i++)
    //         {
    //             if (i == odom_pos.size() - 1)
    //                 odom_file << "{\"x\": " << odom_pos[i].x << ", \"y\": "
    //                           << odom_pos[i].y << ", \"theta\": " << odom_theta[i]
    //                           << ", \"timestamp\": " << odom_time[i].toSec() << "}";
    //             else
    //             {
    //                 odom_file << "{\"x\": " << odom_pos[i].x << ", \"y\": "
    //                           << odom_pos[i].y << ", \"theta\": " << odom_theta[i]
    //                           << ", \"timestamp\": " << odom_time[i].toSec() << "},";
    //             }
    //         }
    //         odom_file << "]";
    //         odom_file.close();
    //     }

    //     std::ofstream joint_file;
    //     joint_file.open("/home/allan/catkin_ws/src/allan_husky/joint_data.json");
    //     if (joint_file.is_open())
    //     {
    //         joint_file << "[";
    //         for (int i = 0; i < joint_states.size(); i++)
    //         {
    //             if (i == joint_states.size() - 1)
    //             {
    //                 joint_file << "{\"velocity\": {\"front_left\": "
    //                            << joint_states[i].velocity[0] << ", \"front_right\": "
    //                            << joint_states[i].velocity[1] << ", \"rear_left\": "
    //                            << joint_states[i].velocity[2]
    //                            << ", \"rear_right\": "
    //                            << joint_states[i].velocity[3]
    //                            << "}, \"position\": {\"front_left\": "
    //                            << joint_states[i].position[0] << ", \"front_right\": "
    //                            << joint_states[i].position[1] << ", \"rear_left\": "
    //                            << joint_states[i].position[2] << ", \"rear_right\": "
    //                            << joint_states[i].position[3] << "}, \"timestamp\": "
    //                            << joint_states[i].header.stamp.toSec() << "}";
    //             }
    //             else
    //             {
    //                 joint_file << "{\"velocity\": {\"front_left\": "
    //                            << joint_states[i].velocity[0] << ", \"front_right\": "
    //                            << joint_states[i].velocity[1] << ", \"rear_left\": "
    //                            << joint_states[i].velocity[2]
    //                            << ", \"rear_right\": "
    //                            << joint_states[i].velocity[3]
    //                            << "}, \"position\": {\"front_left\": "
    //                            << joint_states[i].position[0] << ", \"front_right\": "
    //                            << joint_states[i].position[1] << ", \"rear_left\": "
    //                            << joint_states[i].position[2] << ", \"rear_right\": "
    //                            << joint_states[i].position[3] << "}, \"timestamp\": "
    //                            << joint_states[i].header.stamp.toSec() << "},";
    //             }
    //         }
    //         joint_file << "]";
    //         joint_file.close();
    //     }
    // }

    template <typename T>
    void saveToFile(std::vector<geometry_msgs::Point> x_vec, std::vector<geometry_msgs::Point> y_vec,
                    std::vector<double> theta_vec, std::vector<T> time_vec, const char *filename)
    {
        std::ofstream file;
        file.open(filename);
        if (file.is_open())
        {
            file << "[";

            for (int i = 0; i < x_vec.size(); i++)
            {
                if (i == x_vec.size() - 1)
                    file << "{\"x\": " << x_vec[i].x << ", \"y\": "
                         << y_vec[i].y << ", \"theta\": " << theta_vec[i] << ", \"timestamp\": " << time_vec[i].toSec() << "}";
                else
                {
                    file << "{\"x\": " << x_vec[i].x << ", \"y\": "
                         << y_vec[i].y << ", \"theta\": " << theta_vec[i]
                         << ", \"timestamp\": " << time_vec[i].toSec() << "},";
                }
            }
            file << "]";
            file.close();
        }
    }

    /**
     * @brief Calculate the dead reckoning odometry
     *
     * @details The odometry is calculated by integrating the linear and angular velocities
     */
    void calculateOdom()
    {

        int n = odom_pos.size();
        geometry_msgs::Point c_pos;
        geometry_msgs::Twist c_vel;
        ros::Time c_time;
        double c_theta;
        // Initial position
        if (n == 0)
        {
            c_pos = current_gt_pos;
            c_vel = current_velocities;
            c_theta = current_gt_theta;
            c_time = ros::Time(0);
        }
        // Subsequent positions
        else
        {
            c_pos = odom_pos[n - 1];
            c_vel = current_velocities;
            c_theta = odom_theta[n - 1];
            c_pos.x += c_vel.linear.x * cos(c_theta) * dt;
            c_pos.y += c_vel.linear.x * sin(c_theta) * dt;
            c_theta += c_vel.angular.z * dt;
            c_time = odom_time[n - 1] + ros::Duration(dt);
        }

        ROS_INFO("x: %.3f | y: %.3f | theta: %.3f", c_pos.x, c_pos.y, c_theta);
        odom_pos.push_back(c_pos);
        odom_theta.push_back(c_theta);
        odom_time.push_back(c_time);
        geometry_msgs::Pose odom_pose;
        odom_pose.position = c_pos;
        odom_pose.orientation = tf::createQuaternionMsgFromYaw(c_theta);
        odom_pub.publish(odom_pose);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_node");
    ros::NodeHandle nh;
    Controller controller(&nh);
    ros::spin();
    return 0;
}