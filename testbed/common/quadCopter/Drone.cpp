#include <vector>
#include "Drone.h"

namespace quad {

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
        auto motorFR = std::make_shared<Motor>(motorRadius, motorMass, maxPwm,
                                               CLOCKWISE, mOrangeColor, motorTransform, world, meshFolderPath);
        _motors.push_back(motorFR);

        // --------------- Create the front left motor --------------- //

        // Position of the motor
        motorPosition = rp3d::Vector3(-1, 0, 1) * arm;
        motorTransform = rp3d::Transform(motorPosition, rp3d::Quaternion::identity());

        // Create a motor and a corresponding rigid in the dynamics world
        auto motorFL = std::make_shared<Motor>(motorRadius, motorMass, maxPwm,
                                               COUNTER_CLOCKWISE, mOrangeColor, motorTransform, world, meshFolderPath);
        _motors.push_back(motorFL);

        // --------------- Create the back right motor --------------- //

        // Position of the motor
        motorPosition = rp3d::Vector3(1, 0, -1) * arm;
        motorTransform = rp3d::Transform(motorPosition, rp3d::Quaternion::identity());

        // Create a motor and a corresponding rigid in the dynamics world
        auto motorBR = std::make_shared<Motor>(motorRadius, motorMass, maxPwm, COUNTER_CLOCKWISE, mBlueColor,
                                               motorTransform,
                                               world, meshFolderPath);
        _motors.push_back(motorBR);

        // --------------- Create the back left motor --------------- //

        // Position of the motor
        motorPosition = rp3d::Vector3(-1, 0, -1) * arm;
        motorTransform = rp3d::Transform(motorPosition, rp3d::Quaternion::identity());

        // Create a motor and a corresponding rigid in the dynamics world
        auto motorBL = std::make_shared<Motor>(motorRadius, motorMass, maxPwm, CLOCKWISE, mBlueColor, motorTransform,
                                               world, meshFolderPath);
        _motors.push_back(motorBL);

        // Add _motors to _droneModules
        _droneModules.insert(_droneModules.end(), _motors.begin(), _motors.end());
    }

    void Drone::createFrames(rp3d::DynamicsWorld* world) {

        // --------------- Create the central to FR motor fixed joint --------------- //
        _fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(
                generateFrameInfo(_centralModule, _motors[MOTOR_FR],
                                  _centralModule->getDefaultTransform().getPosition()))));

        // --------------- Create the central to FL motor fixed joint --------------- //
        _fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(
                generateFrameInfo(_centralModule, _motors[MOTOR_FL],
                                  _centralModule->getDefaultTransform().getPosition()))));

        // --------------- Create the central to BR motor fixed joint --------------- //
        _fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(
                generateFrameInfo(_centralModule, _motors[MOTOR_BR],
                                  _centralModule->getDefaultTransform().getPosition()))));

        // --------------- Create the central to BL motor fixed joint --------------- //
        _fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(
                generateFrameInfo(_centralModule, _motors[MOTOR_BL],
                                  _centralModule->getDefaultTransform().getPosition()))));

        // ----- Create motors to central fixed joints for better physics dynamics ---- //
        _fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(
                generateFrameInfo(_motors[MOTOR_FL], _centralModule,
                                  _motors[MOTOR_FL]->getDefaultTransform().getPosition()))));

        _fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(
                generateFrameInfo(_motors[MOTOR_FR], _centralModule,
                                  _motors[MOTOR_FR]->getDefaultTransform().getPosition()))));

        _fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(
                generateFrameInfo(_motors[MOTOR_BL], _centralModule,
                                  _motors[MOTOR_BL]->getDefaultTransform().getPosition()))));

        _fixedJoints.push_back(dynamic_cast<rp3d::FixedJoint*>(world->createJoint(
                generateFrameInfo(_motors[MOTOR_BR], _centralModule,
                                  _motors[MOTOR_BR]->getDefaultTransform().getPosition()))));
    }

    Drone::Drone(double frameSize, double droneMass, double motorRadius, double motorMass, QuadPIDs quadPIDs,
                 rp3d::DynamicsWorld* world, const std::string& meshFolderPath) : _mass(droneMass) {

        // ---------------     Create motors     --------------- //
        createMotors(frameSize, motorRadius, motorMass, world, meshFolderPath);

        // --------------- Create central module --------------- //
        // Position of central module
        rp3d::Vector3 centralModulePosition(0, 0, 0);
        rp3d::Transform centralModuleTransform(centralModulePosition, rp3d::Quaternion::identity());

        // Mass of central module
        double centralModuleMass = droneMass - 4 * motorMass;
        assert(centralModuleMass > 0);

        // Update min and max of Quad PIDs
        for (int i = 0; i < HOVER_PID; ++i) {
            auto type = (pidTypes) i;
            quadPIDs[type].setMinMax(-0.05, 0.05);
        }
        quadPIDs[HOVER_PID].setMinMax(0, _motors[MOTOR_BL]->getMaxPwm());


        // Create a central module and a corresponding rigid in the dynamics world
        openglframework::Vector3 boxDimensions(frameSize, 0.01, frameSize);
        _centralModule = std::make_shared<CentralModule>(centralModuleMass, boxDimensions, centralModuleTransform,
                                                         quadPIDs, world,
                                                         meshFolderPath);
        _droneModules.push_back(_centralModule);

        // Set scale functions for managing input from controller
        ScaleManager<double, double> scaleCubeFunction([](double x) { return x * x * x; });
        /// TODO: create interface for setting scaleFunctions in quad creation
//        _centralModule->_stabilizer->getTargetParameters().setScaleFunctions(scaleCubeFunction,
//                           scaleCubeFunction,
//                           scaleCubeFunction,
//                           scaleCubeFunction);


        // ------------ Create fixed joints for quad frames --------- //
        createFrames(world);
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

    rp3d::FixedJointInfo
    Drone::generateFrameInfo(std::shared_ptr<DroneModule> firstModule, std::shared_ptr<DroneModule> secondModule,
                             const rp3d::Vector3& anchorPoint) {
        rp3d::RigidBody* firstModuleBody = firstModule->getPhysicsBody()->getRigidBody();
        rp3d::RigidBody* secondModuleBody = secondModule->getPhysicsBody()->getRigidBody();
        rp3d::FixedJointInfo jointInfo(firstModuleBody, secondModuleBody, anchorPoint);
        jointInfo.isCollisionEnabled = false;
        return jointInfo;
    }

    void Drone::updatePhysics(double dt) {
        _centralModule->_stabilizer->computePwm(_motors, dt);
        for (auto& motor : _motors) {
            motor->updatePhysics();
//            _centralModule->getPhysicsBody()->getRigidBody()->applyTorque(motor->computeTorque());
        }
    }

    Drone::~Drone() = default;

    double Drone::getAltitude() const {
        return _centralModule->_stabilizer->getCurrentParameters().getAltitude();
    }

    void Drone::setFlightMode(FlightModes flightMode) {
        _centralModule->_stabilizer->setFlightMode(flightMode);
//        _stabilizer->setTargetPRY(5.0);
    }

    void Drone::reset() {
        setMotorsPwm(0);
        _centralModule->_stabilizer->reset();
    }

    /**
     * Set target Pitch, Roll, Yaw, Throttle from input device
     * @param pitch in range [QuadAttitudeParameters::minInput, QuadAttitudeParameters::maxInput]
     * @param roll  in range [QuadAttitudeParameters::minInput, QuadAttitudeParameters::maxInput]
     * @param quadAngles   in range [QuadAttitudeParameters::minInput, QuadAttitudeParameters::maxInput]
     * @param throttle in range range [QuadAttitudeParameters::minInput, QuadAttitudeParameters::maxInput]
     */
    void Drone::setInputParams(QuadAngles quadAngles, double throttle) {
        _centralModule->_stabilizer->setInputParameters(quadAngles, throttle,
                                                        _motors[MOTOR_BL]->getMaxPwm());
    }

    void Drone::setMotorsPwm(double pwm) {
        for (const auto& motor : _motors) {
            motor->setPwm(pwm);
        }
    }

//    void Drone::setThrottle(double throttle) {
//        _centralModule->_stabilizer->setThrottle(std::max(0.0, throttle));
//    }

    double Drone::getThrottle() const {
        return _centralModule->_stabilizer->getThrottle();
    }

    void Drone::destroyQuadModules(rp3d::DynamicsWorld* world) {
        for (auto& fixedJoint : _fixedJoints) {
            world->destroyJoint(fixedJoint);
//            delete fixedJoint;
        }
        _fixedJoints.clear();

        for (auto& droneModule : _droneModules) {
            world->destroyRigidBody(droneModule->getPhysicsBody()->getRigidBody());
//            delete droneModule;
        }
        _droneModules.clear();
        _motors.clear();

//        world->destroyRigidBody(_centralModule->getPhysicsBody()->getRigidBody());
//        delete _centralModule;

    }


}