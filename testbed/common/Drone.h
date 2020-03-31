//
// Created by kirill on 25.03.2020.
//

#ifndef REACTPHYSICS3D_DRONE_H
#define REACTPHYSICS3D_DRONE_H

#include "reactphysics3d.h"
#include "openglframework.h"
#include "Sphere.h"
#include "pid.h"
#include "Stabilizer.h"

class Drone : public openglframework::Object3D {
private:

    double _mass;
    double _throttle = 0;
    Stabilizer* _stabilizer;
    class Barometer {
    private:
        double _altitude;
    public:
#define dronePosition drone->getCentralModule()->getPhysicsBody()->getTransform().getPosition()
        explicit Barometer(Drone* drone) : _altitude(dronePosition.y) {};
        ~Barometer();
        double getAltitude(Drone* drone) {
            _altitude = dronePosition.y;
        }
    };

    class DroneModule {
    protected:
        Sphere* _physicsBody;
        rp3d::Transform defaultTransform;

    public:
        DroneModule(double mass, const openglframework::Color& color, const rp3d::Transform& defaultTransform,
                    rp3d::DynamicsWorld* dynamicsWorld,
                    const std::string& meshFolderPath);

        ~DroneModule();

        rp3d::Transform getDefaultTransform() const;

        Sphere* getPhysicsBody() const;
    };

    class Motor : public DroneModule {
    private:
        double _pwm;
        double _maxpwm;

    public:
        Motor(double propellerRadius, double mass, double maxpwm, const openglframework::Color& color,
              const rp3d::Transform& defaultTransform,
              rp3d::DynamicsWorld* dynamicsWorld,
              const std::string& meshFolderPath);

        inline void setMaxPwm(double maxpwm) {
            if (maxpwm < 0) {
                throw "Drone::setMaxPwm: Invalid value of argument. maxpwm should be positive";
            }
            _maxpwm = maxpwm;
        }

        inline void setPwm(double pwm) {
            if (pwm < 0) {
                throw "Drone::setPwm: Invalid value of argument. pwm should be positive";
            }
            _pwm = std::min(pwm, _maxpwm);
        };

        inline void updatePhysics() {
            rp3d::Vector3 liftingForce(0, _pwm, 0);
            rp3d::Vector3 transformedForce = _physicsBody->getTransform().getOrientation() * liftingForce;
            _physicsBody->getRigidBody()->applyForceToCenterOfMass(transformedForce);
        }

    };

    std::vector<Motor*> motors;
    std::vector<rp3d::FixedJoint*> fixedJoints;
    std::vector<DroneModule*> droneModules;

    Barometer* _barometer;
    static rp3d::FixedJointInfo
    generateFrameInfo(DroneModule* firstModule, DroneModule* secondModule, const rp3d::Vector3& anchorPoint);

public:

    typedef enum {
        MOTOR_FR = 0,
        MOTOR_FL = 1,
        MOTOR_BR = 2,
        MOTOR_BL = 3,
    } motorsNames;

    Drone(double frameSize, double droneMass, double motorRadius, double motorMass, rp3d::DynamicsWorld* world,
          const std::string& meshFolderPath);

    ~Drone() override;

    void render(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix);

    void updatePhysics();

    void setTransform(const rp3d::Transform& transform);

    void setMotorsMaxPwm(double maxpwm);

    inline std::vector<Motor*> getMotors() const {
        return motors;
    }

    inline std::vector<rp3d::FixedJoint*> getFixedJoints() const {
        return fixedJoints;
    }

    inline std::vector<DroneModule*> getDroneModules() const {
        return droneModules;
    }

    inline DroneModule* getCentralModule() const {
        return droneModules[0];
    }

};

#endif //REACTPHYSICS3D_DRONE_H
