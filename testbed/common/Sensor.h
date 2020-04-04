//
// Created by kirill on 03.04.2020.
//

#ifndef REACTPHYSICS3D_SENSOR_H
#define REACTPHYSICS3D_SENSOR_H

#include "DroneModule.h"
#include "QuadAttitudeParameters.h"

namespace drone {

    class Sensor {
    protected:
        const PhysicsObject* _objectToRead;

    public:
        explicit Sensor(const PhysicsObject* objectToRead) : _objectToRead(objectToRead) {};
        virtual void getData(QuadAttitudeParameters&) = 0;
    };
}

#endif //REACTPHYSICS3D_SENSOR_H
