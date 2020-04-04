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
        inline static rp3d::Vector3 getPRYFromQuaternion(const rp3d::Quaternion& Q) {
            double yaw = asin(2.0 * (Q.w * Q.y - Q.z * Q.x));
            double pitch = atan2(2.0 * (Q.w * Q.x + Q.y * Q.z), 1.0 - 2.0 * (Q.x * Q.x + Q.y * Q.y));
            double roll = atan2(2.0 * (Q.w * Q.z + Q.x * Q.y), 1.0 - 2.0 * (Q.y * Q.y + Q.z * Q.z));
            return rp3d::Vector3(pitch, roll, yaw);
        };

    public:
        explicit Gyroscope(const PhysicsObject* objectToRead) : Sensor(objectToRead) {};

        inline void getData(QuadAttitudeParameters& quadAttitudeParameters) override {
            quadAttitudeParameters.setAxisPRY(getPRYFromQuaternion(_objectToRead->getTransform().getOrientation()));
        }
    };

}


#endif //REACTPHYSICS3D_GYROSCOPE_H
