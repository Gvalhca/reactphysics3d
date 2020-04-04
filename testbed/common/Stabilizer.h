//
// Created by kirill on 30.03.2020.
//

#ifndef REACTPHYSICS3D_STABILIZER_H
#define REACTPHYSICS3D_STABILIZER_H


#include "pid.h"
#include "PhysicsObject.h"
#include "Motor.h"
#include "QuadPIDs.h"
#include "QuadAttitudeParameters.h"
#include "Barometer.h"
#include "Gyroscope.h"

namespace drone {

    typedef enum {
        STAB = 0,
        STAB_HEIGHT = 1
    } flightModes;

    class Stabilizer {
    private:
        QuadPIDs _quadPIDs;
        QuadAttitudeParameters _targetParams;
        QuadAttitudeParameters _currentParams;
        double _throttle;

        flightModes _flightMode;

        std::vector<Sensor*> _sensors;

        void computeHoverMode(double dt);

    public:

        Stabilizer(const QuadPIDs& quadPIDs, const PhysicsObject* objectToRead,
                   const QuadAttitudeParameters& currentParameters = QuadAttitudeParameters(),
                   const QuadAttitudeParameters& targetParameters = QuadAttitudeParameters(),
                   flightModes flightMode = STAB);

        double computePwm(const std::vector<Motor*>& motors, double dt);

        void readSensorsData();

        const QuadAttitudeParameters& getCurrentParameters() const;

        const QuadAttitudeParameters& getTargetParameters() const;

        void setTargetAxisPRY(const rp3d::Vector3& targetAxis);

        void setTargetParameters(double targetAltitude, const rp3d::Vector3& targetAxis);

        void setThrottle(double throttle);

        void setFlightMode(flightModes flightMode);

        void reset();

        ///TODO: delete sensors using sharedPtr
        ~Stabilizer();
    };
}

#endif //REACTPHYSICS3D_STABILIZER_H
