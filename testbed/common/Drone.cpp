#include "Barometer.h"
#include "Drone.h"

namespace drone {

    void Drone::createMotors(double frameSize, double motorRadius, double motorMass, rp3d::DynamicsWorld* world,
                             const std::string& meshFolderPath) {

        double arm = frameSize / 2;
        openglframework::Color mBlueColor = openglframework::Color(0, 0.66f, 0.95f, 1.0f);
        openglframework::Color mGreenColor = openglframework::Color(0.0f, 0.95f, 0.3f, 1.0f);
        openglframework::Color mOrangeColor = openglframework::Color(0.9f, 0.35f, 0, 1.0f);
        // Creating _motors
        /// Set maxPwm for motor to lift mass in liftCapability times more than droneMass
        int liftCapability = 3;
        double maxPwm = (_mass * 9.81) * liftCapability / 4;
        // --------------- Create the front right motor --------------- //

        // Position of the motor
        rp3d::Vector3 motorPosition = rp3d::Vector3(1, 0, 1) * arm;
        rp3d::Transform motorTransform(motorPosition, rp3d::Quaternion::identity());

        // Create a sphere and a corresponding rigid in the dynamics world
        auto* motorFR = new Motor(motorRadius, motorMass, maxPwm, mBlueColor, motorTransform, world, meshFolderPath);
        _motors.push_back(motorFR);

        // --------------- Create the front left motor --------------- //

        // Position of the motor
        motorPosition = rp3d::Vector3(-1, 0, 1) * arm;
        motorTransform = rp3d::Transform(motorPosition, rp3d::Quaternion::identity());

        // Create a motor and a corresponding rigid in the dynamics world
        auto* motorFL = new Motor(motorRadius, motorMass, maxPwm, mBlueColor, motorTransform, world, meshFolderPath);
        _motors.push_back(motorFL);

        // --------------- Create the back right motor --------------- //

        // Position of the motor
        motorPosition = rp3d::Vector3(1, 0, -1) * arm;
        motorTransform = rp3d::Transform(motorPosition, rp3d::Quaternion::identity());

        // Create a motor and a corresponding rigid in the dynamics world
        auto* motorBR = new Motor(motorRadius, motorMass, maxPwm, mOrangeColor, motorTransform, world,
                                  meshFolderPath);
        _motors.push_back(motorBR);

        // --------------- Create the back left motor --------------- //

        // Position of the motor
        motorPosition = rp3d::Vector3(-1, 0, -1) * arm;
        motorTransform = rp3d::Transform(motorPosition, rp3d::Quaternion::identity());

        // Create a motor and a corresponding rigid in the dynamics world
        auto* motorBL = new Motor(motorRadius, motorMass, maxPwm, mOrangeColor, motorTransform, world,
                                  meshFolderPath);
        _motors.push_back(motorBL);

        // Add _motors to _droneModules
        _droneModules.insert(_droneModules.end(), _motors.begin(), _motors.end());
    }

    void Drone::createFrames(rp3d::DynamicsWorld* world) {
        DroneModule* centralModule = getCentralModule();
        rp3d::Vector3 centralModulePosition = centralModule->getDefaultTransform().getPosition();
        // --------------- Create the central top fixed joint --------------- //
        // Create the joint in the dynamics world
        _fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(
                generateFrameInfo(_motors[MOTOR_FR], centralModule,
                                  _motors[MOTOR_FR]->getDefaultTransform().getPosition()))));

        // --------------- Create the central bottom fixed joint --------------- //
        // Create the joint in the dynamics world
        _fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(
                generateFrameInfo(_motors[MOTOR_BL], centralModule,
                                  _motors[MOTOR_BL]->getDefaultTransform().getPosition()))));

        // --------------- Create the central left fixed joint --------------- //
        // Create the joint in the dynamics world
        _fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(
                generateFrameInfo(_motors[MOTOR_BR], centralModule,
                                  _motors[MOTOR_BR]->getDefaultTransform().getPosition()))));

        // --------------- Create the central right fixed joint --------------- //
        // Create the joint in the dynamics world
        _fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(
                generateFrameInfo(_motors[MOTOR_FL], centralModule,
                                  _motors[MOTOR_FL]->getDefaultTransform().getPosition()))));
    }

    Drone::Drone(double frameSize, double droneMass, double motorRadius, double motorMass, QuadPids quadPids,
                 rp3d::DynamicsWorld* world, const std::string& meshFolderPath) : _mass(droneMass) {

        openglframework::Color mYellowColor = openglframework::Color(0.9f, 0.88f, 0.145f, 1.0f);

        // --------------- Create the central module --------------- //
        // Position of the module
        rp3d::Vector3 centralModulePosition(0, 0, 0);
        rp3d::Transform centralModuleTransform(centralModulePosition, rp3d::Quaternion::identity());

        // Create a central module and a corresponding rigid in the dynamics world
        double centerMass = droneMass - 4 * motorMass;
        assert(centerMass > 0);
        auto* centralModule = new DroneModule(centerMass, mYellowColor, centralModuleTransform, world, meshFolderPath);
        _droneModules.push_back(centralModule);

        createMotors(frameSize, motorRadius, motorMass, world, meshFolderPath);

        createFrames(world);

        ///TODO: Constructors should be without params
        _barometer = new Barometer(getTransform().getPosition().y);
        _gyroscope = new Gyroscope(rp3d::Vector3::zero());
        for (int i = 0; i < HOVER_PID; ++i) {
            pidTypes type = (pidTypes) i;
            quadPids[type].setMinMax(0, _motors[MOTOR_BL]->getMaxPwm());
        }

        ///TODO: gyroscope
        QuadAttitudeParameters currentParams(0, rp3d::Vector3::zero());
        QuadAttitudeParameters targetParams(0, rp3d::Vector3::zero());
        _stabilizer = new Stabilizer(quadPids, currentParams, targetParams);

    }

#define droneModuleBody droneModule->getPhysicsBody()

    void Drone::render(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix) {
        for (auto& droneModule : _droneModules) {
            droneModuleBody->render(shader, worldToCameraMatrix);
        }
    }

    void Drone::setTransform(const rp3d::Transform& transform) {
        for (auto& droneModule : _droneModules) {
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
        readSensorsData();
        _stabilizer->computePwm(_motors, dt);
        for (auto& motor : _motors) {
            motor->updatePhysics();
        }
    }

    void Drone::readSensorsData() {
        double currAlt = getAltitude();
        rp3d::Vector3 currPRY = _gyroscope->getPRY(*this);
        _stabilizer->setCurrentParameters(currAlt, currPRY);
    };

    Drone::~Drone() {
        delete _stabilizer;
    }

    double Drone::getAltitude() const {
        return _barometer->getAltitude(*this);
    }

    void Drone::hover() {
        readSensorsData();
        _stabilizer->setFlightMode(STAB_HEIGHT);
//        _stabilizer->setTargetPRY(5.0);
    }

    void Drone::reset() {
        setMotorsPwm(0);
        _stabilizer->reset();
    }


}