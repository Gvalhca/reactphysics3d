
#include "Stabilizer.h"

namespace drone {

    double Stabilizer::computePwm(const std::vector<Motor*>& motors, double dt) {
        double addThrust = computeHoverMode(dt);
        for (const auto& motor : motors) {
            motor->setPwm(addThrust);
        }
    }

    double Stabilizer::computeHoverMode(double dt) {
        std::cout << "Current Altitude: " << _currentParams.getAltitude()
                  << " Target Altitude: " << _targetParams.getAltitude()
                  << " Current Pitch: " << _currentParams.getAxis().x
                  << " Roll: " << _currentParams.getAxis().y
                  << " Yaw: " << _currentParams.getAxis().z
                  << std::endl;
        double currentAltitude = _currentParams.getAltitude();
        double thrust = _quadPids[HOVER_PID].calculate(dt, _targetParams.getAltitude(), currentAltitude);
        return thrust;
    }

    void Stabilizer::reset() {
        _quadPids.reset();
        _targetParams.setAltitude(0);
    }

    void Stabilizer::setFlightMode(flightModes flightMode) {
        _flightMode = flightMode;
        if (flightMode == STAB_HEIGHT) {
            _targetParams.setAltitude(_currentParams.getAltitude());
        }
    }

    Stabilizer::~Stabilizer() {}

    void Stabilizer::setTargetParameters(double targetAltitude, const rp3d::Vector3& targetAxis) {
        _targetParams.setParameters(targetAltitude, targetAxis);
    }

    Stabilizer::Stabilizer(QuadPids& quadPids, const QuadAttitudeParameters& currentParameters,
                           const QuadAttitudeParameters& targetParameters, flightModes flightMode) :
            _quadPids(quadPids), _targetParams(targetParameters), _currentParams(currentParameters) {
        setFlightMode(flightMode);
    }

    void Stabilizer::setCurrentParameters(double currentAltitude, const rp3d::Vector3& currentAxis) {
        _currentParams.setParameters(currentAltitude, currentAxis);
    }


}