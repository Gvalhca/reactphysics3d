
#include "Stabilizer.h"

#include <memory>
#include <utility>
#include "quadCopter/Drone.h"
#include "Accelerometer.h"

namespace quad {

    double Stabilizer::computePwm(std::vector<std::shared_ptr<Motor>> motors, double dt) {
        readSensorsData();
        if (_flightMode == STAB_HEIGHT) {
            computeHoverMode(dt);
        }

        QuadAngles currentAngles;
        if (_flightMode == STAB || _flightMode == STAB_HEIGHT) {
            currentAngles = _currentParams.getQuadAngles();
            currentAngles[YAW] = _currentParams.getAngularVelocity().y;
        }
        QuadAngles targetAngles = _targetParams.getQuadAngles();
        double thrustPitch = _quadPIDs[PITCH_PID].calculate(dt, currentAngles[PITCH], targetAngles[PITCH]);
        double thrustRoll = _quadPIDs[ROLL_PID].calculate(dt, currentAngles[ROLL], targetAngles[ROLL]);
        double thrustYaw = _quadPIDs[YAW_PID].calculate(dt, currentAngles[YAW], targetAngles[YAW]);

         double throttle = _currentParams.getThrottle();

        std::vector<double> motorsPwm(4, 0);
        motorsPwm[MOTOR_FR] = thrustPitch - thrustRoll + thrustYaw + throttle;
        motorsPwm[MOTOR_FL] = thrustPitch + thrustRoll - thrustYaw + throttle;
        motorsPwm[MOTOR_BR] = -thrustPitch - thrustRoll - thrustYaw + throttle;
        motorsPwm[MOTOR_BL] = -thrustPitch + thrustRoll + thrustYaw + throttle;

        for (int i = 0; i < 4; i++) {
            motors[i]->setPwm(motorsPwm[i]);
//            std::cout << "Thrust" << i + 1 << "_Pwm: " << motorsPwm[i] << " ";
        }

//        double thrustPitch = 0;
//        double thrustRoll = 0;
//        double thrustYaw = 0;

//        std::cout << "FLight Mode: " << _flightMode << std::endl;
//
        std::cout << "Current Altitude: " << _currentParams.getAltitude()
                  << " Target Altitude: " << _targetParams.getAltitude()
                  << std::endl
                  << "Current Pitch: " << _currentParams.getQuadAngles().getPitch()
                  << " Roll: " << _currentParams.getQuadAngles().getRoll()
                  << " Yaw: " << currentAngles[YAW]
                  << " Throttle: " << _currentParams.getThrottle()
                  << std::endl
                  << " Target Pitch: " << _targetParams.getQuadAngles().getPitch()
                  << " Roll: " << _targetParams.getQuadAngles().getRoll()
                  << " Yaw: " << _targetParams.getQuadAngles().getYaw()
                  << " Throttle: " << _targetParams.getThrottle()
                  << std::endl;

        std::cout << "Thrust Pitch: " << thrustPitch
                  << " Roll: " << thrustRoll
                  << " Yaw: " << thrustYaw
                  << " Throttle: " << throttle
                  << std::endl;

        std::cout << "-------------------------------------------------------------" << std::endl;
    }

    void Stabilizer::computeHoverMode(double dt) {
        double currentAltitude = _currentParams.getAltitude();
        _currentParams.setThrottle(_quadPIDs[HOVER_PID].calculate(dt, _targetParams.getAltitude(), currentAltitude));
        _targetParams.setThrottle(_currentParams.getThrottle());
    }

    void Stabilizer::reset() {
        _quadPIDs.reset();
        _targetParams.reset();
        _currentParams.reset();
    }

    void Stabilizer::setFlightMode(FlightModes flightMode) {
        readSensorsData();
        _flightMode = flightMode;
        if (flightMode == STAB_HEIGHT) {
            _targetParams.setAltitude(_currentParams.getAltitude());
        }
    }

    Stabilizer::~Stabilizer() {
        _sensors.clear();
    }

    void Stabilizer::setInputParameters(QuadAngles inputAngles, double throttle, double maxThrottle) {
        _targetParams.setInputParameters(inputAngles, throttle, maxThrottle);
        _currentParams.setThrottle(_targetParams.getThrottle());
    }

    Stabilizer::Stabilizer(const QuadPIDs& quadPIDs,
                           std::shared_ptr<PhysicsObject> objectToRead,
                           const QuadAttitudeParameters& currentParameters,
                           const QuadAttitudeParameters& targetParameters,
                           FlightModes flightMode) :
            _quadPIDs(quadPIDs),
            _targetParams(targetParameters),
            _currentParams(currentParameters),
            _flightMode(flightMode) {
        _sensors.push_back(std::make_shared<Barometer>(objectToRead));
        _sensors.push_back(std::make_shared<Gyroscope>(objectToRead));
        _sensors.push_back(std::make_shared<Accelerometer>(objectToRead));
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

    FlightModes Stabilizer::getFlightMode() const {
        return _flightMode;
    }

    double Stabilizer::getThrottle() const {
        return _currentParams.getThrottle();
    }

    QuadAngles Stabilizer::getQuadAngles() {
        QuadAttitudeParameters quadParams;
        _sensors[GYROSCOPE]->getData(quadParams);
        return quadParams.getQuadAngles();
    }

}