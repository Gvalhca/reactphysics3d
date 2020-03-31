
#include "Motor.h"

namespace drone {
    Motor::Motor(double propellerRadius, double mass, double maxPwm, const openglframework::Color& color,
                 const rp3d::Transform& defaultTransform, rp3d::DynamicsWorld* dynamicsWorld,
                 const std::string& meshFolderPath) : DroneModule(mass, color, defaultTransform, dynamicsWorld,
                                                                  meshFolderPath), _pwm(0), _maxPwm(maxPwm) {}
}