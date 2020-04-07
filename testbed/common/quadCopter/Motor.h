

#ifndef REACTPHYSICS3D_MOTOR_H
#define REACTPHYSICS3D_MOTOR_H

#include "reactphysics3d.h"
#include "openglframework.h"
#include "quadCopter/DroneModule.h"

namespace drone {

    typedef enum {
        CLOCKWISE = -1,
        COUNTER_CLOCKWISE = 1
    } RotationDirection;

    ///TODO: Add Motor torque
    class Motor : public DroneModule {
    private:
        double _pwm;
        double _maxPwm;
        RotationDirection _rotationDirection;

    public:

        void setMaxPwm(double maxPwm);
        void setPwm(double pwm);

        inline double getMaxPwm() const {
            return _maxPwm;
        }

        inline double getPwm() const {
            return _pwm;
        }

        void updatePhysics();

        Motor(double propellerRadius, double mass, double maxPwm, RotationDirection rotationDirection,
              const openglframework::Color& color, const rp3d::Transform& defaultTransform,
              rp3d::DynamicsWorld* dynamicsWorld, const std::string& meshFolderPath);

        rp3d::Vector3 computeTorque();
    };

}


#endif //REACTPHYSICS3D_MOTOR_H
