

#ifndef REACTPHYSICS3D_DRONE_H
#define REACTPHYSICS3D_DRONE_H

#include "reactphysics3d.h"
#include "openglframework.h"
#include "Sphere.h"
#include "pid.h"
#include "DroneModule.h"
#include "Motor.h"

namespace drone {

    class Drone : public openglframework::Object3D {
    private:
        class Stabilizer;
        class Barometer;
        double _mass;
        double _throttle = 0;
        Stabilizer* _stabilizer = 0;
        Barometer* _barometer = 0;

        std::vector<Motor*> motors;
        std::vector<rp3d::FixedJoint*> fixedJoints;
        std::vector<DroneModule*> droneModules;

        rp3d::FixedJointInfo
        generateFrameInfo(DroneModule* firstModule, DroneModule* secondModule, const rp3d::Vector3& anchorPoint);

        void createMotors(double frameSize, double motorRadius, double motorMass, rp3d::DynamicsWorld* world,
                          const std::string& meshFolderPath);

        void createFrames(rp3d::DynamicsWorld* world);

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

        void updatePhysics(double dt);

        void setTransform(const rp3d::Transform& transform);

        inline rp3d::Transform getTransform() const {
            return getCentralModule()->getPhysicsBody()->getTransform();
        }

        void setMotorsMaxPwm(double maxPwm);

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

        double getAltitude() const;

        void hover();
    };
}
#endif //REACTPHYSICS3D_DRONE_H
