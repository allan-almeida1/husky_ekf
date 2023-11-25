#ifndef KalmanFilter_hpp
#define KalmanFilter_hpp
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "Eigen/Dense"
#include "math.h"

using Eigen::Vector2d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using Eigen::Matrix2d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;

class KalmanFilter
{
public:
    KalmanFilter(ros::NodeHandle *nh);
    ~KalmanFilter();
    void setMeasurements(sensor_msgs::Imu::ConstPtr measurements);
    void setStates(VectorXd states);
    void setStatesCovariance(MatrixXd states_covariance);
    void setEstVel(double est_vel);
    VectorXd getStates();
    MatrixXd getStatesCovariance();
    Vector2d getMeasurements();
    Vector2d getMeasurementsCovariance();
    double getEstVel();
    void setInitialized(bool initialized);
    void setRunning(bool running);
    bool isInitialized();
    bool isRunning();
    void predictionStep(double dt);
    void updateStep();

private:
    // Private variables
    ros::Subscriber imu_data_sub;            // Subscriber for the /imu/data topic
    ros::Subscriber est_vel_sub;             // Subscriber for the /estimated_velocity topic
    ros::Subscriber stop_ekf_sub;            // Subscriber for the /stop_ekf topic
    ros::Publisher est_pos_pub;              // Publisher for the /estimated_position topic
    sensor_msgs::Imu::ConstPtr measurements; // The measurements vector
    double est_vel;                          // The estimated velocity (from the /estimated_velocity topic)
    ros::Timer timer;                        // The timer
    ros::Duration dt;                        // The time step
    bool initialized;                        // Is the filter initialized?
    bool running;                            // Is the filter running?
    VectorXd states;                         // The states vector
    MatrixXd states_covariance;              // The states covariance matrix
    const double POS_STD = 0.1;              // The position standard deviation
    const double VEL_STD = 0.05;             // The velocity standard deviation
    const double THETA_STD = 0.1;            // The orientation standard deviation
    // Private methods
    void measurementsCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void estVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void timerCallback(const ros::TimerEvent &event);
    void stopCallback(const std_msgs::Bool::ConstPtr &msg);
    double normalizeAngle(double angle);
};

#endif /* KalmanFilter_hpp */