
#include "Stabilizer.h"
#include "Drone.h"

namespace drone {

    double Stabilizer::computePwm(const std::vector<Motor*>& motors, double dt) {
        readSensorsData();
        if (_flightMode == STAB_HEIGHT) {
//            computeHoverMode(dt);
        }

        _throttle = 0.2943;

        rp3d::Vector3 currentPRY = _currentParams.getAxisPRY();
        rp3d::Vector3 targetPRY = _targetParams.getAxisPRY();
        double thrustPitch = _quadPIDs[PITCH_PID].calculate(dt, currentPRY[PITCH], targetPRY[PITCH]);
        double thrustRoll = _quadPIDs[ROLL_PID].calculate(dt, currentPRY[ROLL], targetPRY[ROLL]);
//        double thrustYaw = _quadPIDs[YAW_PID].calculate(dt, currentPRY[YAW], targetPRY[YAW]);

//        double thrustPitch = 0;
//        double thrustRoll = 0;
        double thrustYaw = 0;

        std::cout << "Current Altitude: " << _currentParams.getAltitude()
                  << " Target Altitude: " << _targetParams.getAltitude()
                  << std::endl
                  << "Current Pitch: " << _currentParams.getAxisPRY().x
                  << " Roll: " << _currentParams.getAxisPRY().y
                  << " Yaw: " << _currentParams.getAxisPRY().z
                  << " Target Pitch: " << _targetParams.getAxisPRY().x
                  << " Roll: " << _targetParams.getAxisPRY().y
                  << " Yaw: " << _targetParams.getAxisPRY().z
                  << std::endl;

        std::cout << "Thrust Pitch: " << thrustPitch
                  << " Roll: " << thrustRoll
                  << " Yaw: " << thrustYaw
                  << " Throttle: " << _throttle
                  << std::endl;

        std::vector<double> motorsPwm(4, 0);
        motorsPwm[MOTOR_FR] = thrustPitch - thrustRoll + thrustYaw + _throttle;
        motorsPwm[MOTOR_FL] = thrustPitch + thrustRoll - thrustYaw + _throttle;
        motorsPwm[MOTOR_BR] = -thrustPitch - thrustRoll - thrustYaw + _throttle;
        motorsPwm[MOTOR_BL] = -thrustPitch + thrustRoll + thrustYaw + _throttle;

        for (int i = 0; i < 4; i++) {
            motors[i]->setPwm(motorsPwm[i]);
            std::cout << "Motor" << i << " Pwm: " << motorsPwm[i] << " ";
        }
        std::cout << std::endl << "-------------------------------------------------------------" << std::endl;
    }

    void Stabilizer::computeHoverMode(double dt) {

        double currentAltitude = _currentParams.getAltitude();
        _throttle = _quadPIDs[HOVER_PID].calculate(dt, _targetParams.getAltitude(), currentAltitude);
    }

    void Stabilizer::reset() {
        _quadPIDs.reset();
        _targetParams.setAltitude(0);
    }

    void Stabilizer::setFlightMode(flightModes flightMode) {
        readSensorsData();
        _flightMode = flightMode;
        if (flightMode == STAB_HEIGHT) {
            _targetParams.setAltitude(_currentParams.getAltitude());
        }
    }

    Stabilizer::~Stabilizer() {}

    void Stabilizer::setTargetParameters(double targetAltitude, const rp3d::Vector3& targetAxis) {
        _targetParams.setParameters(targetAltitude, targetAxis);
    }

    Stabilizer::Stabilizer(const QuadPIDs& quadPIDs,
                           const PhysicsObject* objectToRead,
                           const QuadAttitudeParameters& currentParameters,
                           const QuadAttitudeParameters& targetParameters,
                           flightModes flightMode) :
            _quadPIDs(quadPIDs),
            _targetParams(targetParameters),
            _currentParams(currentParameters),
            _flightMode(flightMode) {
        _sensors.push_back(new Barometer(objectToRead));
        _sensors.push_back(new Gyroscope(objectToRead));
    }

    const QuadAttitudeParameters& Stabilizer::getCurrentParameters() const {
        return _currentParams;
    }

    const QuadAttitudeParameters& Stabilizer::getTargetParameters() const {
        return _targetParams;
    }

    void Stabilizer::readSensorsData() {
        for (const auto& sensor : _sensors) {
            sensor->getData(_currentParams);
        }
    }

    void Stabilizer::setTargetAxisPRY(const rp3d::Vector3& targetAxis) {
        _targetParams.setAxisPRY(targetAxis);
    }

    void Stabilizer::setThrottle(double throttle) {
        _throttle = throttle;
    }


}