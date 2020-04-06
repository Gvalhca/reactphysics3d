//
// Created by kirill on 03.04.2020.
//

#ifndef REACTPHYSICS3D_GYROSCOPE_H
#define REACTPHYSICS3D_GYROSCOPE_H

#include <mathematics/Vector3.h>
#include "reactphysics3d.h"
#include "Sensor.h"

namespace drone {

    class Gyroscope : public Sensor {
    private:
        inline rp3d::Vector3 getPRYFromQuaternion(const rp3d::Quaternion& Q) const {
//            double toDegrees = 180 / M_PI;
            double angleYaw = asin(2.0 * (Q.w * Q.y - Q.z * Q.x));
            rp3d::Quaternion qYaw = rp3d::Quaternion::fromEulerAngles(0.0, -angleYaw, 0.0);
            rp3d::Quaternion qZeroYaw = Q * qYaw;
            double pitch = atan2(2.0 * (qZeroYaw.w * qZeroYaw.x + qZeroYaw.y * qZeroYaw.z), 1.0 - 2.0 * (qZeroYaw.x * qZeroYaw.x + qZeroYaw.y * qZeroYaw.y));
            double roll = atan2(2.0 * (qZeroYaw.w * qZeroYaw.z + qZeroYaw.x * qZeroYaw.y), 1.0 - 2.0 * (qZeroYaw.y * qZeroYaw.y + qZeroYaw.z * qZeroYaw.z));

            rp3d::Vector3 angVelocity = _objectToRead->getRigidBody()->getAngularVelocity();
//            std::cout << angVelocity.to_string();
            double yaw = angVelocity.y;
//            double pitch = atan2(2.0 * (Q.w * Q.x + Q.y * Q.z), 1.0 - 2.0 * (Q.x * Q.x + Q.y * Q.y));
//            double roll = atan2(2.0 * (Q.w * Q.z + Q.x * Q.y), 1.0 - 2.0 * (Q.y * Q.y + Q.z * Q.z));
//            yaw *= toDegrees;
//            pitch *= toDegrees;
//            roll *= toDegrees;

            return rp3d::Vector3(pitch, roll, yaw);
        };

    public:
        explicit Gyroscope(PhysicsObject* objectToRead) : Sensor(objectToRead) {};

        inline void getData(QuadAttitudeParameters& quadAttitudeParameters) override {
            quadAttitudeParameters.setAxisPRY(getPRYFromQuaternion(_objectToRead->getTransform().getOrientation()));
        }
    };

}


#endif //REACTPHYSICS3D_GYROSCOPE_H
