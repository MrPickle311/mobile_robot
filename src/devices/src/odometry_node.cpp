#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "odometry/odometry_defs.hpp"
#include <memory>
#include <tf/transform_broadcaster.h>

namespace devices
{
    class TestOdometrySource : public OdometrySource
    {
    public:
        virtual Vector3D getPosition() const override
        {
            Vector3D vec;
            vec.x = 0.000;
            vec.y = 0.000;
            vec.z = 0.000;
            return vec;
        }
        virtual Vector3D getVelocity() const override
        {
            Vector3D vec;
            vec.x = 0.00;
            vec.y = 0.00;
            vec.z = 9.84;
            return vec;
        }
    };

    class OdometryProcessorNode
    {
    public:
        const std::string ODOM_RAW_FRAME_ID = std::string{"odom_raw"};

        OdometryProcessorNode(int argc, char **argv) : _node_handle{nullptr},
                                                  _odometry_source{std::make_unique<TestOdometrySource>()},
                                                  _current_seq{0},
                                                  _previous_time{ros::Time::now()}
        {
            ros::init(argc, argv, "odometry_node");
            _node_handle = std::make_unique<ros::NodeHandle>();
            _odometry_data_publisher = _node_handle->advertise<nav_msgs::Odometry>("/odom", 1);
        }

    private:
        std::unique_ptr<ros::NodeHandle> _node_handle;
        std::unique_ptr<OdometrySource> _odometry_source;
        ros::Publisher _odometry_data_publisher;
        long long _current_seq;
        ros::Time _previous_time;

    public:
        void run()
        {
            ros::Rate _rate{2};
            while (ros::ok())
            {
                ros::spinOnce();
                loop();
                _rate.sleep();
            }
        }

        void loop()
        {
            auto time{getCurrentTime()};
            publishRawOdom(time);
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

        void publishRawOdom(ros::Time time)
        {
            nav_msgs::Odometry msg_raw;
            setMsgHeader(msg_raw.header, time, ODOM_RAW_FRAME_ID);
            msg_raw.child_frame_id = "base_link";

            auto current_pose = _odometry_source->getPosition();
            auto current_velocity = _odometry_source->getVelocity();
            auto theta = current_pose.z;
            auto omega = current_velocity.z;

            double dt = (time - _previous_time).toSec();
            double delta_x = (current_velocity.x * cos(theta) - current_velocity.y * sin(theta)) * dt;
            double delta_y = (current_velocity.x * sin(theta) + current_velocity.y * cos(theta)) * dt;
            double delta_theta = omega * dt;

            current_pose.x += delta_x;
            current_pose.y += delta_y;
            current_pose.z += delta_theta;

            msg_raw.pose.pose.position.x = current_pose.x;
            msg_raw.pose.pose.position.y = current_pose.y;
            msg_raw.pose.pose.position.z = 0.0f;
            msg_raw.pose.pose.orientation =  tf::createQuaternionMsgFromYaw(current_pose.z);

            msg_raw.twist.twist.linear.x = current_velocity.x;
            msg_raw.twist.twist.linear.y = current_velocity.y;
            msg_raw.twist.twist.angular.z = current_velocity.z;

            _odometry_data_publisher.publish(msg_raw);

            _previous_time = time;
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
    ros::Time::init();
    devices::OdometryProcessorNode processor{argc, argv};
    processor.run();
    return 0;
}