#pragma once

#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>

namespace devices
{

    using Vector3D = geometry_msgs::Vector3;

    class OdometrySource
    {
    public:
        virtual Vector3D getPosition() const = 0;
        virtual Vector3D getVelocity() const = 0;
    };
}