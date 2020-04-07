

#ifndef REACTPHYSICS3D_DRONE_H
#define REACTPHYSICS3D_DRONE_H

#include "reactphysics3d.h"
#include "openglframework.h"
#include "Sphere.h"
#include "pid.h"
#include "DroneModule.h"
#include "Motor.h"
#include "QuadAttitudeParameters.h"
#include "QuadPIDs.h"
#include "CentralModule.h"
#include "Barometer.h"
#include "Gyroscope.h"


namespace drone {

    typedef enum {
        MOTOR_FR = 0,
        MOTOR_FL = 1,
        MOTOR_BR = 2,
        MOTOR_BL = 3
    } MotorsNames;

    class Drone : public openglframework::Object3D {
    private:
        double _mass;

        CentralModule* _centralModule;
        std::vector<Motor*> _motors;
        std::vector<rp3d::FixedJoint*> _fixedJoints;
        std::vector<DroneModule*> _droneModules;


        static rp3d::FixedJointInfo
        generateFrameInfo(DroneModule* firstModule, DroneModule* secondModule, const rp3d::Vector3& anchorPoint);

        void createMotors(double frameSize, double motorRadius, double motorMass, rp3d::DynamicsWorld* world,
                          const std::string& meshFolderPath);

        void createFrames(rp3d::DynamicsWorld* world);

        void setMotorsPwm(double pwm);

    public:


        Drone(double frameSize, double droneMass, double motorRadius, double motorMass, QuadPIDs quadPIDs,
              rp3d::DynamicsWorld* world, const std::string& meshFolderPath);

        ~Drone() override;

        void destroyQuadModules(rp3d::DynamicsWorld* world);

        void render(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix);

        void updatePhysics(double dt);

        void setTransform(const rp3d::Transform& transform);

        inline rp3d::Transform getTransform() const {
            return getCentralModule()->getPhysicsBody()->getTransform();
        }

        inline void setMotorsMaxPwm(double maxPwm) {
            for (const auto& motor : _motors) {
                motor->setMaxPwm(maxPwm);
            }
        };


        inline std::vector<Motor*> getMotors() const {
            return _motors;
        }

        inline std::vector<rp3d::FixedJoint*> getFixedJoints() const {
            return _fixedJoints;
        }

        inline std::vector<DroneModule*> getDroneModules() const {
            return _droneModules;
        }

        inline CentralModule* getCentralModule() const {
            return _centralModule;
        }

        inline FlightModes getFlightMode() const {
            return _centralModule->_stabilizer->getFlightMode();
        };

        double getThrottle() const;

        void setThrottle(double throttle);

        double getAltitude() const;

        void setInputAxisPRY(double pitch, double roll, double yaw);

        void setFlightMode(FlightModes flightMode);

        void reset();
    };
}
#endif //REACTPHYSICS3D_DRONE_H
