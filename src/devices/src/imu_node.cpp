#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "imu/imu_defs.hpp"
#include <memory>

namespace devices
{
    class TestImuSource : public ImuSource
    {
    public:
        virtual Vector3D getAngularVelocity() const override
        {
            Vector3D vec;
            vec.x = 0.000;
            vec.y = 0.000;
            vec.z = 0.000;
            return vec;
        }
        virtual Vector3D getLinearAcceleration() const override
        {
            Vector3D vec;
            vec.x = 0.00;
            vec.y = 0.00;
            vec.z = 9.84;
            return vec;
        }
        virtual Vector3D getMagneticField() const override
        {
            Vector3D vec;
            vec.x = -7.7;
            vec.y = -30.2;
            vec.z = -21.1;
            return vec;
        }
    };

    class OdometryProcessorNode
    {
    public:
        const std::string IMU_RAW_FRAME_ID = std::string{"imu_raw"};
        const std::string MAG_RAW_FRAME_ID = std::string{"mag_raw"};

        OdometryProcessorNode(int argc, char **argv) : _node_handle{nullptr},
                                                  _odometry_source{std::make_unique<TestImuSource>()},
                                                  _current_seq{0}
        {
            ros::init(argc, argv, "imu_node");
            _node_handle = std::make_unique<ros::NodeHandle>();
            _odometry_data_publisher = _node_handle->advertise<sensor_msgs::Imu>("/imu/data_raw", 1);
            _mag_data_publisher = _node_handle->advertise<sensor_msgs::MagneticField>("/imu/mag", 1);
            std::cout << "Created!\n";
        }

    private:
        std::unique_ptr<ros::NodeHandle> _node_handle;
        std::unique_ptr<ImuSource> _odometry_source;
        ros::Publisher _odometry_data_publisher;
        ros::Publisher _mag_data_publisher;
        long long _current_seq;

    public:
        void run()
        {
            ros::Rate _rate{2};
            while (ros::ok())
            {
                loop();
                ros::spinOnce();
                _rate.sleep();
            }
        }

        void loop()
        {
            auto time{getCurrentTime()};
            publishRawImu(time);
            publishRawMag(time);
            incremenentCurrentSeq();
        }

        ros::Time getCurrentTime()
        {
            return ros::Time::now();
        }

        void incremenentCurrentSeq()
        {
            ++_current_seq;
        }

        void publishRawImu(ros::Time time)
        {
            sensor_msgs::Imu msg_raw;
            setMsgHeader(msg_raw.header, time, IMU_RAW_FRAME_ID);
            msg_raw.linear_acceleration = _odometry_source->getLinearAcceleration();
            msg_raw.angular_velocity = _odometry_source->getAngularVelocity();
            msg_raw.orientation.x = 1.0f;
            msg_raw.orientation_covariance[0] = -1;
            _odometry_data_publisher.publish(msg_raw);
        }

        void publishRawMag(ros::Time time)
        {
            sensor_msgs::MagneticField msg_raw;
            setMsgHeader(msg_raw.header, time, MAG_RAW_FRAME_ID);
            msg_raw.magnetic_field = _odometry_source->getMagneticField();
            _mag_data_publisher.publish(msg_raw);
        }

        void setMsgHeader(std_msgs::Header &header, const ros::Time &time, const std::string &frame_id)
        {
            header.stamp = time;
            header.frame_id = frame_id;
            header.seq = _current_seq;
        }
    };

}

int main(int argc, char **argv)
{
    devices::OdometryProcessorNode processor{argc, argv};
    processor.run();
    return 0;
}