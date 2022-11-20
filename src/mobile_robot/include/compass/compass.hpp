#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float32.h"

namespace mobile_robot{

class Compass {

private:
  ros::NodeHandle node_;
  ros::Subscriber imu_sub_;
  ros::Subscriber mag_sub_;
  ros::Publisher compass_publisher_;
  ros::Timer check_timer_;
  sensor_msgs::ImuPtr current_imu_reading_;

  bool is_first_mag_reading_occured_;
  bool is_first_imu_reading_occured_;
  bool is_filter_initialized_;
  bool is_imu_update_complete_;

  double last_imu_update_timestamp_;
  double last_magnetometer_update_timestamp_;
  double sensors_measurement_timestamp_mismatch_;

  double current_heading_estimation_;
  double heading_estimation_prognose_;
  double heading_estimation_prognose_uncertainlity_;
  double curr_heading_uncertainlity_;
  double process_noise_;
  double heading_measurement_uncertainlity_;

private:
  void predictionImuCallback(sensor_msgs::ImuPtr data);
  void magUpdateCallback(const geometry_msgs::Vector3StampedConstPtr& data);
  void checkCallback(const ros::TimerEvent&);
  void publishHeading();
  bool innovationIsTooLarge(double innovation);
  void initFilter(double current_heading);

public:
  Compass(ros::NodeHandle &n);
};

}

