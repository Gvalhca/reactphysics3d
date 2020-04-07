//
// Created by kirill on 03.04.2020.
//

#ifndef REACTPHYSICS3D_SENSOR_H
#define REACTPHYSICS3D_SENSOR_H

#include "quadCopter/DroneModule.h"
#include "QuadAttitudeParameters.h"

namespace quad {

    class Sensor {
    protected:
        PhysicsObject* _objectToRead;

    public:
        explicit Sensor(PhysicsObject*& objectToRead) : _objectToRead(objectToRead) {};
        virtual void getData(QuadAttitudeParameters&) = 0;
        virtual ~Sensor() = default;;
    };
}

#endif //REACTPHYSICS3D_SENSOR_H
