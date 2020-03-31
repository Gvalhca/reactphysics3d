

#ifndef REACTPHYSICS3D_MOTOR_H
#define REACTPHYSICS3D_MOTOR_H

#include "reactphysics3d.h"
#include "openglframework.h"
#include "DroneModule.h"

namespace drone {

    class Motor : public DroneModule {
    private:
        double _pwm;
        double _maxPwm;

    public:
        inline void setMaxPwm(double maxPwm) {
            if (maxPwm < 0) {
                throw "Motor::setMaxPwm: Invalid value of argument. maxPwm should be positive";
            }
            _maxPwm = maxPwm;
        }

        inline double getMaxPwm() const {
            return _maxPwm;
        }

        inline double getPwm() const {
            return _pwm;
        }

        inline void setPwm(double pwm) {
            if (pwm < 0) {
                throw "Motor::setPwm: Invalid value of argument. pwm should be positive";
            }
            _pwm = std::min(pwm, _maxPwm);
        }

        inline void updatePhysics() {
            rp3d::Vector3 liftingForce(0, _pwm, 0);
            rp3d::Vector3 transformedForce = DroneModule::_physicsBody->getTransform().getOrientation() * liftingForce;
            DroneModule::_physicsBody->getRigidBody()->applyForceToCenterOfMass(transformedForce);
        }

        Motor(double propellerRadius, double mass, double maxPwm, const openglframework::Color& color,
              const rp3d::Transform& defaultTransform,
              rp3d::DynamicsWorld* dynamicsWorld,
              const std::string& meshFolderPath);
    };

}


#endif //REACTPHYSICS3D_MOTOR_H
