//
// Created by kirill on 30.03.2020.
//

#ifndef REACTPHYSICS3D_STABILIZER_H
#define REACTPHYSICS3D_STABILIZER_H


#include "pid.h"
#include "PhysicsObject.h"
#include "Motor.h"
#include "QuadPids.h"
#include "QuadAttitudeParameters.h"

namespace drone {

    typedef enum {
        STAB = 0,
        STAB_HEIGHT = 1
    } flightModes;

    class Stabilizer {
    private:
        QuadPids _quadPids;
        QuadAttitudeParameters _targetParams;
        QuadAttitudeParameters _currentParams;
        flightModes _flightMode;

        double computeHoverMode(double dt);

    public:

        explicit Stabilizer(QuadPids& quadPids, const QuadAttitudeParameters& currentParameters,
                            const QuadAttitudeParameters& targetParameters, flightModes flightMode = STAB_HEIGHT);

        double computePwm(const std::vector<Motor*>& motors, double dt);

        void setTargetParameters(double targetAltitude, const rp3d::Vector3& targetAxis);

        void setCurrentParameters(double currentAltitude, const rp3d::Vector3& currentAxis);

        void setFlightMode(flightModes flightMode);

        void reset();

        ~Stabilizer();
    };
}

#endif //REACTPHYSICS3D_STABILIZER_H
