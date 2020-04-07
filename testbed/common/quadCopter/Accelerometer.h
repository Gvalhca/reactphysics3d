//
// Created by kirill on 07.04.2020.
//

#ifndef REACTPHYSICS3D_ACCELEROMETER_H
#define REACTPHYSICS3D_ACCELEROMETER_H

#include "Sensor.h"

namespace quad {

    class Accelerometer : public Sensor {
    public:
        explicit Accelerometer(PhysicsObject*& objectToRead) : Sensor(objectToRead) {};

        inline void getData(QuadAttitudeParameters& quadAttitudeParameters) override {
            quadAttitudeParameters.setAngularVelocity(_objectToRead->getRigidBody()->getAngularVelocity());
        }
    };
}

#endif //REACTPHYSICS3D_ACCELEROMETER_H
