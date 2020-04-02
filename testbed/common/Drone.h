

#ifndef REACTPHYSICS3D_DRONE_H
#define REACTPHYSICS3D_DRONE_H

#include "reactphysics3d.h"
#include "openglframework.h"
#include "Sphere.h"
#include "pid.h"
#include "DroneModule.h"
#include "Motor.h"
#include "QuadAttitudeParameters.h"
#include "QuadPids.h"
#include "Stabilizer.h"

namespace drone {

    typedef enum {
        MOTOR_FR = 0,
        MOTOR_FL = 1,
        MOTOR_BR = 2,
        MOTOR_BL = 3,
    } motorsNames;

    class Drone : public openglframework::Object3D {
    private:
        class Barometer;

        class Gyroscope {
        private:
            rp3d::Vector3 _axisPRY;

            static rp3d::Vector3 getPRYFromQuaternion(const rp3d::Quaternion &Q)
            {
                double yaw   = asin  (2.0 * (Q.w * Q.y - Q.z * Q.x));
                double pitch = atan2 (2.0 * (Q.w * Q.x + Q.y * Q.z),1.0 - 2.0 * (Q.x * Q.x + Q.y * Q.y));
                double roll  = atan2 (2.0 * (Q.w * Q.z + Q.x * Q.y),1.0 - 2.0 * (Q.y * Q.y + Q.z * Q.z));
                return rp3d::Vector3(pitch, roll, yaw);
            }
        public:
            explicit Gyroscope(const rp3d::Vector3& axisPRY): _axisPRY(axisPRY) {};
            rp3d::Vector3 getPRY(const Drone& drone) const {
                return getPRYFromQuaternion(drone.getTransform().getOrientation());
            }
        };


        double _mass;
        double _throttle = 0;

        Stabilizer* _stabilizer = 0;
        Barometer* _barometer = 0;
        Gyroscope* _gyroscope = 0;

        std::vector<Motor*> _motors;
        std::vector<rp3d::FixedJoint*> _fixedJoints;
        std::vector<DroneModule*> _droneModules;

        static rp3d::FixedJointInfo
        generateFrameInfo(DroneModule* firstModule, DroneModule* secondModule, const rp3d::Vector3& anchorPoint);

        void createMotors(double frameSize, double motorRadius, double motorMass, rp3d::DynamicsWorld* world,
                          const std::string& meshFolderPath);

        void createFrames(rp3d::DynamicsWorld* world);

        void readSensorsData();

    public:


        Drone(double frameSize, double droneMass, double motorRadius, double motorMass, QuadPids quadPids,
              rp3d::DynamicsWorld* world, const std::string& meshFolderPath);

        ~Drone() override;

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

        inline void setMotorsPwm(double pwm) {
            for (const auto& motor : _motors) {
                motor->setPwm(pwm);
            }
        }

        inline std::vector<Motor*> getMotors() const {
            return _motors;
        }

        inline std::vector<rp3d::FixedJoint*> getFixedJoints() const {
            return _fixedJoints;
        }

        inline std::vector<DroneModule*> getDroneModules() const {
            return _droneModules;
        }

        inline DroneModule* getCentralModule() const {
            return _droneModules[0];
        }

        double getAltitude() const;

        void hover();

        void reset();
    };
}
#endif //REACTPHYSICS3D_DRONE_H
