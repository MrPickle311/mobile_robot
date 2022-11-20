#include "compass/compass.hpp"
#include <cmath>
#include <iostream>
#include <geometry_msgs/Twist.h>

namespace mobile_robot{

double vectorLength(tf::Vector3 vector) 
{
  return std::sqrt(vector.x()*vector.x() + 
                   vector.y()*vector.y() + 
                   vector.z()*vector.z());
}

double now()
{
  return ros::Time::now().toSec();
}

Compass::Compass(ros::NodeHandle &node_handle) :
    node_{node_handle}, 
    current_imu_reading_{new sensor_msgs::Imu{}},
    sensors_measurement_timestamp_mismatch_{0.5},
    heading_measurement_uncertainlity_{1.0},
    process_noise_{0.01}
{
  imu_sub_ = node_.subscribe("mobile_robot_imu", 1000, &Compass::predictionImuCallback, this);
  mag_sub_ = node_.subscribe("mobile_robot_mag", 1000, &Compass::magUpdateCallback, this);
  compass_publisher_ = node_.advertise<std_msgs::Float32>("compass_heading", 1);

  is_first_mag_reading_occured_ = false;
  is_first_imu_reading_occured_ = false;
  is_imu_update_complete_ = false;
  last_imu_update_timestamp_ = now();
  check_timer_ = node_.createTimer(ros::Duration(1), &Compass::checkCallback, this);

  ROS_INFO("Compass started");
}

void Compass::checkCallback(const ros::TimerEvent&) 
{
  if (!is_first_imu_reading_occured_)
  {
    ROS_WARN("Waiting for IMU data ...");
  }

  if (!is_first_mag_reading_occured_)
  {
    ROS_WARN("Waiting for mag data ...");
  }

  if ((now() - last_imu_update_timestamp_ > sensors_measurement_timestamp_mismatch_) && is_first_imu_reading_occured_) 
  {
    ROS_WARN("IMU data being receieved too slow or not at all");
    is_first_imu_reading_occured_ = false;
  }

  if ((now() - last_magnetometer_update_timestamp_ > sensors_measurement_timestamp_mismatch_) && is_first_mag_reading_occured_) 
  {
    ROS_WARN("Magnetometer data being receieved too slow or not at all");
    is_filter_initialized_ = false;
    is_first_mag_reading_occured_ = false;
  }
}

void Compass::predictionImuCallback(const sensor_msgs::ImuPtr data) 
{
  geometry_msgs::Vector3 gyro_vector;
  gyro_vector = data->angular_velocity;

  if(!is_first_imu_reading_occured_)
    is_first_imu_reading_occured_ = true;

  double dt = now() - last_imu_update_timestamp_;
  last_imu_update_timestamp_ = now();
  double yaw_gyro_velocity =  gyro_vector.z;

  if (is_filter_initialized_) {
    heading_estimation_prognose_ = current_heading_estimation_ + yaw_gyro_velocity * dt;
    heading_estimation_prognose_uncertainlity_ = curr_heading_uncertainlity_ + process_noise_;

    if (heading_estimation_prognose_ > 3.14159)
      heading_estimation_prognose_ -= 2 * 3.14159;
    else if(heading_estimation_prognose_ < -3.14159)
      heading_estimation_prognose_ += 2 * 3.14159;
    is_imu_update_complete_ = true;
  }
  current_imu_reading_ = data;
}

void Compass::magUpdateCallback(const geometry_msgs::Vector3StampedConstPtr& data) 
{
  if ( std::isnan(data->vector.x) ||
       std::isnan(data->vector.y) ||
       std::isnan(data->vector.z) ) 
  {
    ROS_INFO("Incoming mag vector is nan!");
    return;
  }

  geometry_msgs::Vector3 imu_mag = data->vector;
  imu_mag.x = data->vector.x;
  imu_mag.y = data->vector.y;
  imu_mag.z = data->vector.z;

  last_magnetometer_update_timestamp_ = now();

  tf::Vector3 normalized_vector{imu_mag.x, imu_mag.y, imu_mag.z};
  normalized_vector = normalized_vector / vectorLength(normalized_vector);

  double heading_meas = atan2(normalized_vector.x(), normalized_vector.y());

  if (!is_first_mag_reading_occured_) 
  {
    initFilter(heading_meas);
    is_first_mag_reading_occured_ = true;
    return;
  }

  if (is_imu_update_complete_) 
  {
    double kalman_gain = heading_estimation_prognose_uncertainlity_ /
      (heading_estimation_prognose_uncertainlity_ + heading_measurement_uncertainlity_);
    double innovation = heading_meas - heading_estimation_prognose_;
    
    if (innovationIsTooLarge(innovation))
    { 
      current_heading_estimation_ = heading_meas;
    }
    else
    {
      current_heading_estimation_ = heading_estimation_prognose_ + kalman_gain * (innovation);
    }
    curr_heading_uncertainlity_ = (1 - kalman_gain) * heading_estimation_prognose_uncertainlity_;

    publishHeading();
    is_imu_update_complete_ = false;
  }
}

bool Compass::innovationIsTooLarge(double innovation)
{
  return (abs(innovation) > M_PI);
}

void Compass::publishHeading() 
{
  std_msgs::Float32 msg;
  msg.data = std::fmod(current_heading_estimation_ + 2 * M_PI, 2* M_PI);
  compass_publisher_.publish(msg);
}

void Compass::initFilter(double current_heading) 
{
  current_heading_estimation_ = current_heading;
  curr_heading_uncertainlity_ = 1;
  is_filter_initialized_ = true;
  ROS_INFO("Filter initialized");
}

}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "compass");
  ros::NodeHandle node;
  ros::Duration(2).sleep();
  mobile_robot::Compass imu_heading_estimator{node};
  ros::spin();
  return 0;
}
