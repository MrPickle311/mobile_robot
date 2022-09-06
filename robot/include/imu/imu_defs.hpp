#pragma once

#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

namespace devices
{

    using Vector3D = geometry_msgs::Vector3;

    class ImuSource
    {
    public:
        virtual Vector3D getAngularVelocity() const = 0;
        virtual Vector3D getLinearAcceleration() const = 0;
        virtual Vector3D getMagneticField() const = 0;
    };
}