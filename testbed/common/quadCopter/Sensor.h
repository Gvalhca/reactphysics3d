//
// Created by kirill on 03.04.2020.
//

#ifndef REACTPHYSICS3D_SENSOR_H
#define REACTPHYSICS3D_SENSOR_H

#include <utility>

#include "quadCopter/DroneModule.h"
#include "QuadAttitudeParameters.h"

namespace quad {

    class Sensor {
    protected:
        std::shared_ptr<PhysicsObject> _objectToRead;

    public:
        explicit Sensor(std::shared_ptr<PhysicsObject> objectToRead) : _objectToRead(std::move(objectToRead)) {};
        virtual void getData(QuadAttitudeParameters&) = 0;
        virtual ~Sensor() = default;
    };
}

#endif //REACTPHYSICS3D_SENSOR_H
