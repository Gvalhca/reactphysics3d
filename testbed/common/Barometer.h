
#ifndef REACTPHYSICS3D_BAROMETER_H
#define REACTPHYSICS3D_BAROMETER_H

#include "reactphysics3d.h"
#include "openglframework.h"
#include "pid.h"
#include "Drone.h"


namespace drone {

    class Drone::Barometer {

        double _currentAltitude;
    public:
        explicit Barometer(double altitude) : _currentAltitude(altitude) {}

        ~Barometer() = default;

        double getAltitude(const Drone& drone) {
            _currentAltitude = drone.getTransform().getPosition().y;
            return _currentAltitude;
        }
    };
}

#endif //REACTPHYSICS3D_BAROMETER_H
