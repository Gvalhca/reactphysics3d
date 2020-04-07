
#include "Motor.h"

namespace quad {
    Motor::Motor(double propellerRadius, double mass, double maxPwm, RotationDirection rotationDirection,
                 const openglframework::Color& color, const rp3d::Transform& defaultTransform,
                 rp3d::DynamicsWorld* dynamicsWorld, const std::string& meshFolderPath) :
            DroneModule(mass, defaultTransform, dynamicsWorld, meshFolderPath, propellerRadius, color),
            _pwm(0),
            _maxPwm(maxPwm),
            _rotationDirection(rotationDirection) {}

    void Motor::setMaxPwm(double maxPwm) {
        if (maxPwm < 0) {
            throw std::runtime_error("Motor::setMaxPwm: Invalid value of argument. maxPwm should be positive");
        }
        _maxPwm = maxPwm;
    }

    void Motor::setPwm(double pwm) {
        _pwm = std::max(pwm, 0.0);
        _pwm = std::min(_pwm, _maxPwm);
    }

    void Motor::updatePhysics() {
        rp3d::Vector3 liftingForce(0, _pwm, 0);
        rp3d::Vector3 transformedForce = _physicsBody->getTransform().getOrientation() * liftingForce;
        _physicsBody->getRigidBody()->applyForceToCenterOfMass(transformedForce);
        _physicsBody->getRigidBody()->applyTorque(_physicsBody->getTransform().getOrientation() * computeTorque());
    }

    rp3d::Vector3 Motor::computeTorque() {
        // Compute Motor Torque coefficients
        double k = 2 * pow(10, -6);
        double b = 1 * pow(10, -7);
        ///TODO: Update formula according to http://engsi.ru/file/out/723336, page 526, equations (7), (8)
        double momentum = (_pwm / k) * b * _rotationDirection;

        return rp3d::Vector3(0.0, momentum, 0.0);
    }
}