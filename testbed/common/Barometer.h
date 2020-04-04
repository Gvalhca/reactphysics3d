
#ifndef REACTPHYSICS3D_BAROMETER_H
#define REACTPHYSICS3D_BAROMETER_H

#include "reactphysics3d.h"
#include "openglframework.h"
#include "Sensor.h"

namespace drone {

    class Barometer : public Sensor {
    public:
        explicit Barometer(const PhysicsObject* objectToRead) : Sensor(objectToRead) {};

        inline void getData(QuadAttitudeParameters& quadAttitudeParameters) override {
            quadAttitudeParameters.setAltitude(_objectToRead->getTransform().getPosition().y);
        }
    };
}

#endif //REACTPHYSICS3D_BAROMETER_H
