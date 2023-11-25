#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "kalman_filter.hpp"

/**************************************************/
/***************** Public methods *****************/
/**************************************************/

/**
 * @brief Constructor for the KalmanFilter class.
 *
 * @param nh The ROS node handle.
 */
KalmanFilter::KalmanFilter(ros::NodeHandle *nh)
{
    this->dt = ros::Duration(0.1);
    this->initialized = false;
    this->running = false;
    this->states = VectorXd(5);
    this->states_covariance = MatrixXd(5, 5);
    this->imu_data_sub = nh->subscribe("/imu/data", 100, &KalmanFilter::measurementsCallback, this);
    this->est_vel_sub = nh->subscribe("/estimated_velocity", 100, &KalmanFilter::estVelCallback, this);
    this->stop_ekf_sub = nh->subscribe("/stop_ekf", 100, &KalmanFilter::stopCallback, this);
    this->est_pos_pub = nh->advertise<geometry_msgs::Twist>("/estimated_position", 100);
    this->timer = nh->createTimer(this->dt, &KalmanFilter::timerCallback, this);
}

/**
 * @brief Destructor for the KalmanFilter class.
 */
KalmanFilter::~KalmanFilter()
{
    ros::shutdown();
}

// ------------------- SETTERS ------------------- //

/**
 * @brief Setter for the measurements vector.
 *
 * @param measurements The measurements vector.
 */
void KalmanFilter::setMeasurements(sensor_msgs::Imu::ConstPtr measurements)
{
    this->measurements = measurements;
}

/**
 * @brief Setter for the initialized variable.
 *
 * @param initialized The initialized variable.
 */
void KalmanFilter::setInitialized(bool initialized)
{
    this->initialized = initialized;
}

/**
 * @brief Setter for the running variable.
 *
 * @param running The running variable.
 */
void KalmanFilter::setRunning(bool running)
{
    this->running = running;
}

/**
 * @brief Setter for the states vector.
 *
 * @param states The states vector.
 */
void KalmanFilter::setStates(VectorXd states)
{
    this->states = states;
}

/**
 * @brief Setter for the states covariance matrix.
 *
 * @param states_covariance The states covariance matrix.
 */
void KalmanFilter::setStatesCovariance(MatrixXd states_covariance)
{
    this->states_covariance = states_covariance;
}

// ------------------- GETTERS ------------------- //

/**
 * @brief Getter for the measurements vector.
 *
 * @return Vector2d The measurements vector.
 */
Vector2d KalmanFilter::getMeasurements()
{
    const sensor_msgs::Imu::ConstPtr &measurements = this->measurements;
    // This check is necessary because we're accessing a measurements by reference
    if (measurements == NULL)
    {
        return Vector2d(0.0, 0.0);
    }
    float ax = measurements->linear_acceleration.x;
    float wz = measurements->angular_velocity.z;
    const Vector2d measurements_vector(wz, ax);
    return measurements_vector;
}

/**
 * @brief Getter for the measurements covariance vector.
 *
 * @return Vector2d The measurements covariance vector.
 */
Vector2d KalmanFilter::getMeasurementsCovariance()
{
    const sensor_msgs::Imu::ConstPtr &measurements = this->measurements;
    // This check is necessary because we're accessing a measurements by reference
    if (measurements == NULL)
    {
        return Vector2d(0.0, 0.0);
    }
    float ax = measurements->linear_acceleration_covariance[0];
    float wz = measurements->angular_velocity_covariance[0];
    const Vector2d measurements_covariance_vector(wz, ax);
    return measurements_covariance_vector;
}

/**
 * @brief Getter for the initialized variable.
 *
 * @return bool The initialized variable.
 */
bool KalmanFilter::isInitialized()
{
    return this->initialized;
}

/**
 * @brief Getter for the running variable.
 *
 * @return bool The running variable.
 */
bool KalmanFilter::isRunning()
{
    return this->running;
}

/**
 * @brief Getter for the states vector.
 *
 * @return VectorXd The states vector.
 */
VectorXd KalmanFilter::getStates()
{
    return this->states;
}

/**
 * @brief Getter for the states covariance matrix.
 *
 * @return MatrixXd The states covariance matrix.
 */
MatrixXd KalmanFilter::getStatesCovariance()
{
    return this->states_covariance;
}

/**************************************************/
/***************** Private methods ****************/
/**************************************************/

/**
 * @brief Callback for the /imu/data topic.
 *
 * @param msg The message received from the /imu/data topic.
 */
void KalmanFilter::measurementsCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    setMeasurements(msg);
}

/**
 * @brief Callback for the /estimated_velocity topic.
 *
 * @param msg The message received from the /estimated_velocity topic.
 */
void KalmanFilter::estVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    if (!isRunning())
    {
        ROS_INFO("Starting the filter");
        setRunning(true);
        VectorXd states = getStates();
        ROS_INFO("Getting velocity estimated from the dead reckoning");
        states << 0.0, 0.0, 0.0, msg->linear.x, 0.0;
        setStates(states);
    }
}

/**
 * @brief Callback for the timer.
 *
 * @param event The timer event.
 */
void KalmanFilter::timerCallback(const ros::TimerEvent &event)
{
    if (isRunning())
    {
        predictionStep(dt.toSec());
        updateStep();
    }
}

/**
 * @brief Callback for the /stop_ekf topic.
 *
 * @param msg The message received from the /stop_ekf topic.
 */
void KalmanFilter::stopCallback(const std_msgs::Bool::ConstPtr &msg)
{
    if (msg->data)
    {
        ROS_INFO("Stopping the filter");
        setRunning(false);
        ros::shutdown();
    }
}

/**
 * @brief Normalizes an angle between -pi and pi.
 *
 * @param angle The angle to be normalized.
 * @return double The normalized angle.
 */
double KalmanFilter::normalizeAngle(double angle)
{
    angle = fmod(angle, (2.0 * M_PI));
    if (angle <= -M_PI)
    {
        angle += (2.0 * M_PI);
    }
    else if (angle > M_PI)
    {
        angle -= (2.0 * M_PI);
    }
    return angle;
}

// --------------------------------------------------- //
// ------------------ KALMAN FILTER ------------------ //
// --------------------------------------------------- //

/**
 * @brief Prediction step of the Kalman filter.
 *
 * @param dt The time step.
 */
void KalmanFilter::predictionStep(double dt)
{
    if (isInitialized())
    {
        ROS_INFO("Running the prediction step");
        VectorXd states = getStates();
        MatrixXd states_covariance = getStatesCovariance();
        Vector2d measurements = getMeasurements();
        Vector2d measurements_covariance = getMeasurementsCovariance();

        // Predict states
        VectorXd update_vector(5);
        update_vector << states(3) * cos(states(2)) * dt,
            states(3) * sin(states(2)) * dt,
            normalizeAngle(measurements(0) * dt),
            states(4) * dt,
            0.0;
        states += update_vector;

        // Predict covariance
        MatrixXd F(5, 5); // The Jacobian of the state transition function
        MatrixXd L(5, 1); // The Jacobian of the measurement function
        MatrixXd Q(5, 5); // The process covariance matrix

        F << 1.0, 0.0, -dt * sin(states(2)) * states(3), dt * cos(states(2)), 0.0,
            0.0, 1.0, dt * cos(states(2)) * states(3), dt * sin(states(2)), 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, dt,
            0.0, 0.0, 0.0, 0.0, 1.0;

        L << 0.0, 0.0, 0.0, 0.0, 1.0;

        Q = MatrixXd::Zero(5, 5);
        Q(0, 0) = POS_STD * POS_STD;
        Q(1, 1) = POS_STD * POS_STD;
        Q(2, 2) = THETA_STD * THETA_STD;
        Q(3, 3) = VEL_STD * VEL_STD;
        Q(4, 4) = measurements_covariance(1) * 10;

        // here it's breaking
        states_covariance = F * states_covariance * F.transpose() + Q;
        setStates(states);
        setStatesCovariance(states_covariance);
    }
}

/**
 * @brief Update step of the Kalman filter.
 */
void KalmanFilter::updateStep()
{
    if (!isInitialized())
    {
        ROS_INFO("Initializing the filter");

        // Initialize the states vector
        VectorXd states = getStates();
        states(0) = 0.0;
        states(1) = 0.0;
        states(2) = 0.0;
        states(4) = getMeasurements()(1);
        setStates(states);

        // Initialize the states covariance matrix
        MatrixXd states_covariance(5, 5);
        states_covariance = getStatesCovariance();
        double accel_cov = getMeasurementsCovariance()(1);
        states_covariance << 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, VEL_STD * VEL_STD, 0.0,
            0.0, 0.0, 0.0, 0.0, accel_cov * 10;
        setStatesCovariance(states_covariance);

        setInitialized(true);
    }
    else
    {
        ROS_INFO("Running the update step");
        VectorXd states = getStates();
        MatrixXd states_covariance = getStatesCovariance();
        Vector2d measurements = getMeasurements();
        Vector2d measurements_covariance = getMeasurementsCovariance();

        // Update states
        MatrixXd H(1, 5);
        H << 0.0, 0.0, 0.0, 0.0, 1.0;

        MatrixXd R(1, 1);
        R << measurements_covariance(1); // The measurement covariance matrix

        VectorXd Z(1);
        Z << measurements(1);                                   // The measured acceleration
        VectorXd Y = Z - H * states;                            // The innovation vector
        MatrixXd S = H * states_covariance * H.transpose() + R; // The innovation covariance matrix

        MatrixXd K = states_covariance * H.transpose() * S.inverse(); // The Kalman gain

        // Update states and states covariance
        states += K * Y;
        states_covariance = (MatrixXd::Identity(5, 5) - K * H) * states_covariance;

        setStates(states);
        setStatesCovariance(states_covariance);

        // Publish the estimated position
        geometry_msgs::Twist estimated_position;
        estimated_position.linear.x = states(0);
        estimated_position.linear.y = states(1);
        estimated_position.angular.z = states(2);
        est_pos_pub.publish(estimated_position);
    }
}