#include "Barometer.h"
#include "Drone.h"
#include "Stabilizer.h"

namespace drone {

    Drone::Drone(double frameSize, double droneMass, double motorRadius, double motorMass, rp3d::DynamicsWorld* world,
                 const std::string& meshFolderPath) : _mass(droneMass) {

        openglframework::Color mYellowColor = openglframework::Color(0.9f, 0.88f, 0.145f, 1.0f);

        // --------------- Create the central module --------------- //
        // Position of the module
        rp3d::Vector3 centralModulePosition(0, 0, 0);
        rp3d::Transform centralModuleTransform(centralModulePosition, rp3d::Quaternion::identity());

        // Create a central module and a corresponding rigid in the dynamics world
        double centerMass = droneMass - 4 * motorMass;
        assert(centerMass > 0);
        auto* centralModule = new DroneModule(centerMass, mYellowColor, centralModuleTransform, world, meshFolderPath);
        droneModules.push_back(centralModule);

        createMotors(frameSize, motorRadius, motorMass, world, meshFolderPath);

        createFrames(world);

        PID hoverPID(motors[MOTOR_BL]->getMaxPwm(), 0, 0.7, 0.35, 0.35);
        _stabilizer = new Stabilizer(hoverPID);

        _barometer = new Barometer(getTransform().getPosition().y);
    }

#define droneModuleBody droneModule->getPhysicsBody()

    void Drone::render(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix) {
        for (auto& droneModule : droneModules) {
            droneModuleBody->render(shader, worldToCameraMatrix);
        }
    }

    void Drone::setTransform(const rp3d::Transform& transform) {
        for (auto& droneModule : droneModules) {
            rp3d::Transform defaultTransform = droneModule->getDefaultTransform();
            rp3d::Transform modifiedTransform = transform * defaultTransform;
            droneModuleBody->setTransform(modifiedTransform);
        }
    }

    rp3d::FixedJointInfo Drone::generateFrameInfo(DroneModule* firstModule, DroneModule* secondModule,
                                                  const rp3d::Vector3& anchorPoint) {
        rp3d::RigidBody* firstModuleBody = firstModule->getPhysicsBody()->getRigidBody();
        rp3d::RigidBody* secondModuleBody = secondModule->getPhysicsBody()->getRigidBody();
        rp3d::FixedJointInfo jointInfoTop(firstModuleBody, secondModuleBody, anchorPoint);
        jointInfoTop.isCollisionEnabled = false;
        return jointInfoTop;
    }

    void Drone::updatePhysics(double dt) {
        _stabilizer->computePwm(*this, dt);
        for (auto& motor : motors) {
            motor->updatePhysics();
        }
    }

    Drone::~Drone() {
        delete _stabilizer;
    }

    void Drone::createMotors(double frameSize, double motorRadius, double motorMass, rp3d::DynamicsWorld* world,
                             const std::string& meshFolderPath) {

        double arm = frameSize / 2;
        openglframework::Color mBlueColor = openglframework::Color(0, 0.66f, 0.95f, 1.0f);
        openglframework::Color mGreenColor = openglframework::Color(0.0f, 0.95f, 0.3f, 1.0f);
        openglframework::Color mOrangeColor = openglframework::Color(0.9f, 0.35f, 0, 1.0f);
        // Creating motors
        /// Set maxPwm for motor to lift mass in liftCapability times more than droneMass
        int liftCapability = 3;
        double maxPwm = (_mass * 9.81) * liftCapability / 4;
        // --------------- Create the front right motor --------------- //

        // Position of the motor
        rp3d::Vector3 motorPosition = rp3d::Vector3(1, 0, 1) * arm;
        rp3d::Transform motorTransform(motorPosition, rp3d::Quaternion::identity());

        // Create a sphere and a corresponding rigid in the dynamics world
        auto* motorFR = new Motor(motorRadius, motorMass, maxPwm, mBlueColor, motorTransform, world, meshFolderPath);
        motors.push_back(motorFR);

        // --------------- Create the front left motor --------------- //

        // Position of the motor
        motorPosition = rp3d::Vector3(-1, 0, 1) * arm;
        motorTransform = rp3d::Transform(motorPosition, rp3d::Quaternion::identity());

        // Create a motor and a corresponding rigid in the dynamics world
        auto* motorFL = new Motor(motorRadius, motorMass, maxPwm, mBlueColor, motorTransform, world, meshFolderPath);
        motors.push_back(motorFL);

        // --------------- Create the back right motor --------------- //

        // Position of the motor
        motorPosition = rp3d::Vector3(1, 0, -1) * arm;
        motorTransform = rp3d::Transform(motorPosition, rp3d::Quaternion::identity());

        // Create a motor and a corresponding rigid in the dynamics world
        auto* motorBR = new Motor(motorRadius, motorMass, maxPwm, mOrangeColor, motorTransform, world,
                                  meshFolderPath);
        motors.push_back(motorBR);

        // --------------- Create the back left motor --------------- //

        // Position of the motor
        motorPosition = rp3d::Vector3(-1, 0, -1) * arm;
        motorTransform = rp3d::Transform(motorPosition, rp3d::Quaternion::identity());

        // Create a motor and a corresponding rigid in the dynamics world
        auto* motorBL = new Motor(motorRadius, motorMass, maxPwm, mOrangeColor, motorTransform, world,
                                  meshFolderPath);
        motors.push_back(motorBL);

        // Add motors to droneModules
        droneModules.insert(droneModules.end(), motors.begin(), motors.end());
    }

    void Drone::createFrames(rp3d::DynamicsWorld* world) {
        DroneModule* centralModule = getCentralModule();
        rp3d::Vector3 centralModulePosition = centralModule->getDefaultTransform().getPosition();
        // --------------- Create the central top fixed joint --------------- //
        // Create the joint in the dynamics world
        fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(
                generateFrameInfo(motors[MOTOR_FR], centralModule, motors[MOTOR_FR]->getDefaultTransform().getPosition()))));

        // --------------- Create the central bottom fixed joint --------------- //
        // Create the joint in the dynamics world
        fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(
                generateFrameInfo(motors[MOTOR_BL], centralModule, motors[MOTOR_BL]->getDefaultTransform().getPosition()))));

        // --------------- Create the central left fixed joint --------------- //
        // Create the joint in the dynamics world
        fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(
                generateFrameInfo(motors[MOTOR_BR], centralModule, motors[MOTOR_BR]->getDefaultTransform().getPosition()))));

        // --------------- Create the central right fixed joint --------------- //
        // Create the joint in the dynamics world
        fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(
                generateFrameInfo(motors[MOTOR_FL], centralModule, motors[MOTOR_FL]->getDefaultTransform().getPosition()))));
    }

    double Drone::getAltitude() const {
        return _barometer->getAltitude(*this);
    }

    void Drone::hover() {
        _stabilizer->setTargetParameters(getAltitude());
//        _stabilizer->setTargetParameters(5.0);
    }

    void Drone::reset() {
        setMotorsPwm(0);
        _stabilizer->reset();
    };


}