//
// Created by kirill on 03.04.2020.
//

#ifndef REACTPHYSICS3D_GYROSCOPE_H
#define REACTPHYSICS3D_GYROSCOPE_H

#include <mathematics/Vector3.h>

#include <utility>
#include "reactphysics3d.h"
#include "Sensor.h"

namespace quad {

    class Gyroscope : public Sensor {
    private:

        rp3d::Vector3 toEulerAngles(const rp3d::Quaternion& q) const;

        rp3d::Vector3 getPRYFromQuaternion(const rp3d::Quaternion& Q) const;

    public:
        explicit Gyroscope(std::shared_ptr<PhysicsObject> objectToRead) : Sensor(std::move(objectToRead)) {};

        inline void getData(QuadAttitudeParameters& quadAttitudeParameters) override {
//            quadAttitudeParameters.setAxisPRY(getPRYFromQuaternion(_objectToRead->getTransform().getOrientation()));
            quadAttitudeParameters.setAxisPRY(toEulerAngles(_objectToRead->getTransform().getOrientation()));
        }
    };

}


#endif //REACTPHYSICS3D_GYROSCOPE_H
