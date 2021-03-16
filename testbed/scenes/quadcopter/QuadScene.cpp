
// Libraries
#include "QuadScene.h"
#include <cmath>


// Namespaces
using namespace openglframework;
using namespace quadscene;

// Constructor
QuadScene::QuadScene(const std::string& name, EngineSettings& settings)
        : SceneDemo(name, settings, SCENE_RADIUS) {

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 5, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);
    mCamera.rotateAroundWorldPoint(Vector3(0, 1, 0), PI, mCenterScene);

    // Gravity vector in the dynamics world
    rp3d::Vector3 gravity(0, rp3d::decimal(-9.81), 0);
//    rp3d::Vector3 gravity(0, 0, 0);

    rp3d::WorldSettings worldSettings;
    worldSettings.worldName = name;

    // Create the dynamics world for the physics simulation
    mPhysicsWorld = new rp3d::DynamicsWorld(gravity, worldSettings);

    //Create Drone
    createDrone();

    // Create the floor
    createFloor();

    createWall();

    createTestBox();

    // Get the physics engine parameters
    mEngineSettings.isGravityEnabled = getDynamicsWorld()->isGravityEnabled();
    rp3d::Vector3 gravityVector = getDynamicsWorld()->getGravity();
    mEngineSettings.gravity = openglframework::Vector3(gravityVector.x, gravityVector.y, gravityVector.z);
    mEngineSettings.isSleepingEnabled = getDynamicsWorld()->isSleepingEnabled();
    mEngineSettings.sleepLinearVelocity = getDynamicsWorld()->getSleepLinearVelocity();
    mEngineSettings.sleepAngularVelocity = getDynamicsWorld()->getSleepAngularVelocity();
    mEngineSettings.nbPositionSolverIterations = getDynamicsWorld()->getNbIterationsPositionSolver();
    mEngineSettings.nbVelocitySolverIterations = getDynamicsWorld()->getNbIterationsVelocitySolver();
    mEngineSettings.timeBeforeSleep = getDynamicsWorld()->getTimeBeforeSleep();
}

// Destructor
QuadScene::~QuadScene() {
#if 0
    // Destroy the joints
    getDynamicsWorld()->destroyJoint(mSliderJoint);
    getDynamicsWorld()->destroyJoint(mPropellerHingeJoint);
    getDynamicsWorld()->destroyJoint(mFixedJoint1);
    getDynamicsWorld()->destroyJoint(mFixedJoint2);
    for (int i = 0; i < NB_BALLSOCKETJOINT_BOXES - 1; i++) {
        getDynamicsWorld()->destroyJoint(mBallAndSocketJoints[i]);
    }

    // Destroy all the rigid bodies of the scene
    getDynamicsWorld()->destroyRigidBody(mSliderJointBottomBox->getRigidBody());
    getDynamicsWorld()->destroyRigidBody(mSliderJointTopBox->getRigidBody());
    getDynamicsWorld()->destroyRigidBody(mPropellerBox->getRigidBody());
    getDynamicsWorld()->destroyRigidBody(mFixedJointBox1->getRigidBody());
    getDynamicsWorld()->destroyRigidBody(mFixedJointBox2->getRigidBody());
    for (int i = 0; i < NB_BALLSOCKETJOINT_BOXES; i++) {
        getDynamicsWorld()->destroyRigidBody(mBallAndSocketJointChainBoxes[i]->getRigidBody());
    }

    delete mSliderJointBottomBox;
    delete mSliderJointTopBox;
    delete mPropellerBox;
    delete mFixedJointBox1;
    delete mFixedJointBox2;
    for (int i = 0; i < NB_BALLSOCKETJOINT_BOXES; i++) {
        delete mBallAndSocketJointChainBoxes[i];
    }
#endif
    //Destroy the quad
    mDrone->destroyQuadModules(getDynamicsWorld());
    delete mDrone;

    // Destroy the floor
    getDynamicsWorld()->destroyRigidBody(mFloor->getRigidBody());
    delete mFloor;

    getDynamicsWorld()->destroyRigidBody(mWall->getRigidBody());
    delete mWall;

    // Destroy the dynamics world
    delete getDynamicsWorld();
}

// Update the physics world (take a simulation step)
void QuadScene::updatePhysics() {
    double elapsedTime = static_cast<double>(mEngineSettings.elapsedTime) - simStartTime;

    mDrone->updatePhysics(mEngineSettings.timeStep);
    SceneDemo::updatePhysics();
    rp3d::Vector3 dronePos = mDrone->getCentralModule()->getPhysicsBody()->getTransform().getPosition();
//               mCamera.setTransformMatrix(mDrone->getCentralModule()->getPhysicsBody()->getTransformMatrix());
//    resetCameraToViewAll();
//    mCamera.translateWorld(openglframework::Vector3(dronePos.x, dronePos.y, dronePos.z));

//    openglframework::Vector3 cameraShift(mDrone->getTransformMatrix().getTranspose() * openglframework::Vector3(0, 1, 0));
//    openglframework::Vector3 cameraPosition(dronePos.x, dronePos.y, dronePos.z);
//    cameraPosition = cameraPosition + cameraShift;
//    setScenePosition(cameraPosition, 3);

//    QuadAngles quadAngles = mDrone->getQuadAngles();
//    quadAngles.setPitch(PI / 6);
//    mCamera.rotateAroundWorldPoint(openglframework::Vector3(0, 1, 0), quadAngles.getPitch() - cameraAngles.x, mCenterScene);
//    cameraAngles.x = quadAngles.getPitch();


    refreshCamera();
}

// Reset the scene
void QuadScene::reset() {
    simStartTime = static_cast<double>(mEngineSettings.elapsedTime);

    // --------------- Drone --------------- //
    mDrone->reset();
    rp3d::Vector3 positionDrone(0, initialHeight, 0);
    mDrone->setTransform(rp3d::Transform(positionDrone, rp3d::Quaternion::identity()));
    mDrone->setFlightMode(STAB_HEIGHT);


}

// Create the floor
void QuadScene::createFloor() {

    // Create the floor
    rp3d::Vector3 floorPosition(0, 0, 0);
    mFloor = new Box(FLOOR_SIZE, FLOOR_MASS, getDynamicsWorld(), mMeshFolderPath);

    // Set the box color
    mFloor->setColor(mBlackColorDemo);
    mFloor->setSleepingColor(mBlackColorDemo);

    // The floor must be a static rigid body
    mFloor->getRigidBody()->setType(rp3d::BodyType::STATIC);

    // Change the material properties of the rigid body
    rp3d::Material& material = mFloor->getRigidBody()->getMaterial();
    material.setBounciness(rp3d::decimal(0.3));
    mPhysicsObjects.push_back(mFloor);
}

void QuadScene::createWall() {

    // Create the floor
    rp3d::Vector3 floorPosition(0, 0, 0);
    mWall = new Box(WALL_SIZE, FLOOR_MASS, getDynamicsWorld(), mMeshFolderPath);
    mWall->setTransform(rp3d::Transform(rp3d::Vector3(0, 0, 25), rp3d::Quaternion::identity()));
    // Set the box color
    mWall->setColor(mRedColorDemo);
    mWall->setSleepingColor(mRedColorDemo);

    // The floor must be a static rigid body
    mWall->getRigidBody()->setType(rp3d::BodyType::STATIC);

    // Change the material properties of the rigid body
    rp3d::Material& material = mWall->getRigidBody()->getMaterial();
    material.setBounciness(rp3d::decimal(0.3));
    mPhysicsObjects.push_back(mWall);
}


void QuadScene::createTestBox() {

    // Create the floor
    rp3d::Vector3 floorPosition(0, 0, 0);
    testBox = new Box(TESTBOX_SIZE, FLOOR_MASS, getDynamicsWorld(), mMeshFolderPath);
    testBox->setTransform(rp3d::Transform(rp3d::Vector3(5, 10, -5), rp3d::Quaternion::identity()));
    // Set the box color
    testBox->setColor(mGreenColorDemo);
    testBox->setSleepingColor(mGreenColorDemo);

    // The floor must be a static rigid body
    testBox->getRigidBody()->setType(rp3d::BodyType::STATIC);

    // Change the material properties of the rigid body
    rp3d::Material& material = testBox->getRigidBody()->getMaterial();
    material.setBounciness(rp3d::decimal(0.3));
    mPhysicsObjects.push_back(testBox);
}


void QuadScene::createDrone() {
    rp3d::Vector3 positionDrone(0, initialHeight, 0);
    QuadPIDs quadPIDs(PID(0.07, 0.035, 0.015),
                      PID(0.07, 0.035, 0.015),
                      PID(0.07, 0.035, 0.0005),
                      PID(7, 3.5, 3.5));
    double droneFrame = 0.088;
    double droneMass = 0.12;
    double motorMass = 0.01;
    double propellerRadius = 0.02;

    mDrone = new Drone(droneFrame, droneMass, propellerRadius, motorMass,
                       quadPIDs, getDynamicsWorld(), mMeshFolderPath);
    mDrone->setTransform(rp3d::Transform(positionDrone, rp3d::Quaternion::identity()));
    for (auto& mDroneModule : mDrone->getDroneModules()) {
        mPhysicsObjects.push_back(mDroneModule->getPhysicsBody().get());
    }
    mDrone->setFlightMode(STAB_HEIGHT);
}

///////////////////////////////////////////////////////////////////////////////
// rotate matrix to face along the target direction
// NOTE: This function will clear the previous rotation and scale info and
// rebuild the matrix with the target vector. But it will keep the previous
// translation values.
// NOTE: It is for rotating object to look at the target, NOT for camera
///////////////////////////////////////////////////////////////////////////////
//Matrix4& lookAt(const Vector3& target)
//{
//    // compute forward vector and normalize
//    Vector3 position = Vector3(m[12], m[13], m[14]);
//    Vector3 forward = target - position;
//    forward.normalize();
//    Vector3 up;             // up vector of object
//    Vector3 left;           // left vector of object
//
//    // compute temporal up vector
//    // if forward vector is near Y-axis, use up vector (0,0,-1) or (0,0,1)
//    if(fabs(forward.x) < EPSILON && fabs(forward.z) < EPSILON)
//    {
//        // forward vector is pointing +Y axis
//        if(forward.y > 0)
//            up.set(0, 0, -1);
//            // forward vector is pointing -Y axis
//        else
//            up.set(0, 0, 1);
//    }
//    else
//    {
//        // assume up vector is +Y axis
//        up.set(0, 1, 0);
//    }
//
//    // compute left vector
//    left = up.cross(forward);
//    left.normalize();
//
//    // re-compute up vector
//    up = forward.cross(left);
//    //up.normalize();
//
//    // NOTE: overwrite rotation and scale info of the current matrix
//    this->setColumn(0, left);
//    this->setColumn(1, up);
//    this->setColumn(2, forward);
//
//    return *this;
//}

//Matrix4& QuadScene::fLookAt(const Vector3& target, const Vector3& upVec)
//{
//    // compute forward vector and normalize
//    Vector3 position(mCamera.getTransformMatrix().m[12], mCamera.getTransformMatrix().m[13], mCamera.getTransformMatrix().m[14]);
//    Vector3 forward = target - position;
//    forward.normalize();
//
//    // compute left vector
//    Vector3 left = upVec.cross(forward);
//    left.normalize();
//
//    // compute orthonormal up vector
//    Vector3 up = forward.cross(left);
//    up.normalize();
//
//    // NOTE: overwrite rotation and scale info of the current matrix
//    this->setColumn(0, left);
//    this->setColumn(1, up);
//    this->setColumn(2, forward);
//
//    return *this;
//}

//
//Matrix4& Matrix4::lookAt(float tx, float ty, float tz)
//{
//    return lookAt(Vector3(tx, ty, tz));
//}
//
//Matrix4& Matrix4::lookAt(float tx, float ty, float tz, float ux, float uy, float uz)
//{
//    return lookAt(Vector3(tx, ty, tz), Vector3(ux, uy, uz));
//}


openglframework::Matrix4 lookAt(openglframework::Vector3 const& eye, openglframework::Vector3 const& center,
                                openglframework::Vector3 const& up) {
    openglframework::Vector3 f = (center - eye).normalize();
    openglframework::Vector3 u = up;
    u.normalize();
    openglframework::Vector3 s = f.cross(u).normalize();
    u = s.cross(f);

    openglframework::Matrix4 result;
    result.m[0][0] = s.x;
    result.m[1][0] = s.y;
    result.m[2][0] = s.z;
    result.m[0][1] = u.x;
    result.m[1][1] = u.y;
    result.m[2][1] = u.z;
    result.m[0][2] = -f.x;
    result.m[1][2] = -f.y;
    result.m[2][2] = -f.z;
    result.m[3][0] = -s.dot(eye);
    result.m[3][1] = -u.dot(eye);
    result.m[3][2] = f.dot(eye);
    return result;
}

//void glhLookAtf2(float* matrix, float* eyePosition3D,
//                 float* center3D, float* upVector3D) {
//    float forward[3], side[3], up[3];
//    float matrix2[16], resultMatrix[16];
//    // --------------------
//    forward[0] = center3D[0] - eyePosition3D[0];
//    forward[1] = center3D[1] - eyePosition3D[1];
//    forward[2] = center3D[2] - eyePosition3D[2];
//    NormalizeVector(forward);
//    // --------------------
//    // Side = forward x up
//    ComputeNormalOfPlane(side, forward, upVector3D);
//    NormalizeVector(side);
//    -- -- -- -- -- -- -- -- -- --
//            // Recompute up as: up = side x forward
//            ComputeNormalOfPlane(up, side, forward);
//    // --------------------
//    matrix2[0] = side[0];
//    matrix2[4] = side[1];
//    matrix2[8] = side[2];
//    matrix2[12] = 0.0;
//    // --------------------
//    matrix2[1] = up[0];
//    matrix2[5] = up[1];
//    matrix2[9] = up[2];
//    matrix2[13] = 0.0;
//    // --------------------
//    matrix2[2] = -forward[0];
//    matrix2[6] = -forward[1];
//    matrix2[10] = -forward[2];
//    matrix2[14] = 0.0;
//    // --------------------
//    matrix2[3] = matrix2[7] = matrix2[11] = 0.0;
//    matrix2[15] = 1.0;
//    // --------------------
//    MultiplyMatrices4by4OpenGL_FLOAT(resultMatrix, matrix, matrix2);
//    glhTranslatef2(resultMatrix,
//                   -eyePosition3D[0], -eyePosition3D[1], -eyePosition3D[2]);
//    // --------------------
//    memcpy(matrix, resultMatrix, 16 * sizeof(float));
//}

openglframework::Matrix4
setLookAt(openglframework::Vector3 eye, openglframework::Vector3 at, openglframework::Vector3 up) {
    openglframework::Vector3 zaxis = (eye - at).normalize();
    openglframework::Vector3 xaxis = zaxis.cross(up).normalize();
    openglframework::Vector3 yaxis = xaxis.cross(zaxis);

    zaxis *= -1;

    openglframework::Matrix4 viewMatrix = {
            openglframework::Vector4(xaxis.x, xaxis.y, xaxis.z, -xaxis.dot(eye)),
            openglframework::Vector4(yaxis.x, yaxis.y, yaxis.z, -yaxis.dot(eye)),
            openglframework::Vector4(zaxis.x, zaxis.y, zaxis.z, -zaxis.dot(eye))
    };

    return viewMatrix;
}

void QuadScene::refreshCamera() {
    rp3d::Vector3 dronePos = mDrone->getCentralModule()->getPhysicsBody()->getTransform().getPosition();
    openglframework::Vector3 dronePosVector(dronePos.x, dronePos.y, dronePos.z);

    rp3d::Vector3 testBoxPosition = testBox->getTransform().getPosition();
    Matrix4 testMatrix(setLookAt(openglframework::Vector3(testBoxPosition.x, testBoxPosition.y,
                                                          testBoxPosition.z), dronePosVector,
                                 Vector3(0, 1, 0)));
    openglframework::Matrix3 m33 = testMatrix.getUpperLeft3x3Matrix();
    rp3d::Matrix3x3 orientationMatrix(m33.getColumn(0)[0], m33.getColumn(0)[1], m33.getColumn(0)[2],
                                      m33.getColumn(1)[0], m33.getColumn(1)[1], m33.getColumn(1)[2],
                                      m33.getColumn(2)[0], m33.getColumn(2)[1], m33.getColumn(2)[2]);
//    testBox->getTransformMatrix().print();
    testBox->setTransform(rp3d::Transform(testBoxPosition, orientationMatrix.getTranspose()));

    /// camera calculation
    rp3d::Vector3 camPositionShift(0, 0.05, mDrone->getFrameSize());
    rp3d::Vector3 camTransformedPositionShift(mDrone->getTransform() * camPositionShift);
    openglframework::Vector3 camPosition(camTransformedPositionShift.x, camTransformedPositionShift.y, camTransformedPositionShift.z);

//    Vector3 lookAtShift(0, 0, 5);
//    Vector3 lookAtTransformedShift(lookAtShift);
//    rp3d::Vector3 lookAtOrientedShift = mDrone->getTransform().getOrientation() *
//                                        rp3d::Vector3(lookAtTransformedShift.x, lookAtTransformedShift.y, lookAtTransformedShift.z);
//    openglframework::Vector3 cameraLookAtVec = dronePosVector;
//    cameraLookAtVec = cameraLookAtVec + Vector3(lookAtOrientedShift.x, lookAtOrientedShift.y, lookAtOrientedShift.z);

    rp3d::Vector3 lookAtShift(0, 0, 5);
    rp3d::Vector3 lookAtOrientedShift = mDrone->getTransform() * lookAtShift;
    openglframework::Vector3 cameraLookAtVec(lookAtOrientedShift.x, lookAtOrientedShift.y, lookAtOrientedShift.z);

    rp3d::Vector3 upVector(0, 1, 0);
    rp3d::Quaternion climbQuaternion(rp3d::Quaternion::fromEulerAngles(0.0f, 1.0f, 0.0f));
    rp3d::Vector3 orientedUpVector = mDrone->getTransform().getOrientation() * climbQuaternion * upVector;
    Vector3 cameraUpVector(orientedUpVector.x, orientedUpVector.y, orientedUpVector.z);

    Matrix4 cameraLookAtMatrix(lookAt(camPosition, cameraLookAtVec, cameraUpVector));
    openglframework::Matrix3 cam33 = cameraLookAtMatrix.getUpperLeft3x3Matrix();
    Matrix4 camOrientationMatrix(cam33.getColumn(0)[0], cam33.getColumn(1)[0], cam33.getColumn(2)[0], camPosition.x,
                                 cam33.getColumn(0)[1], cam33.getColumn(1)[1], cam33.getColumn(2)[1], camPosition.y,
                                 cam33.getColumn(0)[2], cam33.getColumn(1)[2], cam33.getColumn(2)[2], camPosition.z,
                                 0, 0, 0, 1);

    mCamera.translateWorld(-mCamera.getOrigin());
    mCamera.translateWorld(camPosition);
    mCamera.setTransformMatrix(camOrientationMatrix);
//    cameraLookAtMatrix.print();
}

//void QuadScene::SaveImage(){
//    int* buffer = new int[ 1920 * 1080 * 3];
//    glReadPixels(0, 0, WIDTH, 1080, GL_BGR, )
//}

static QuadAngles testPRY(1500.0, 1500.0, 1500.0);
static double quadThrottle = 900.0;

bool QuadScene::keyboardEvent(int key, int scancode, int action, int mods) {

    if (action == GLFW_PRESS) {
        switch (key) {
            case GLFW_KEY_W:
                testPRY.setPitch(2100);
                break;
            case GLFW_KEY_S:
                testPRY.setPitch(900);
                break;
            case GLFW_KEY_A:
                testPRY.setRoll(900);
                break;
            case GLFW_KEY_D:
                testPRY.setRoll(2100);
                break;
            case GLFW_KEY_Q:
                testPRY.setYaw(2100);
                break;
            case GLFW_KEY_E:
                testPRY.setYaw(900);
                break;
            case GLFW_KEY_SPACE:
                mDrone->setFlightMode(mDrone->getFlightMode() ? STAB : STAB_HEIGHT);
                break;
            case GLFW_KEY_R:
                mDrone->setFlightMode(STAB);
                quadThrottle = quadThrottle + 50;
                break;
            case GLFW_KEY_F:
                mDrone->setFlightMode(STAB);
                quadThrottle = quadThrottle - 50;
                break;
            case GLFW_KEY_1:
                rp3d::Vector3 dronePos = mDrone->getCentralModule()->getPhysicsBody()->getTransform().getPosition();
                openglframework::Vector3 dronePosVector(dronePos.x, dronePos.y, dronePos.z);
                Vector3 cameraPosition(0, 10, 0);
                Matrix4 cameraLookAtMatrix(lookAt(cameraPosition, dronePosVector,
                                                  Vector3(0, 1, 0)));
//    openglframework::Matrix3 cam33 = cameraLookAtMatrix.getUpperLeft3x3Matrix();
//    rp3d::Matrix3x3 orientationMatrix(m33.getColumn(0)[0], m33.getColumn(0)[1], m33.getColumn(0)[2],
//                                      m33.getColumn(1)[0], m33.getColumn(1)[1], m33.getColumn(1)[2],
//                                      m33.getColumn(2)[0], m33.getColumn(2)[1], m33.getColumn(2)[2]);
                mCamera.translateWorld(-mCamera.getOrigin());
                mCamera.translateWorld(cameraPosition);

                openglframework::Matrix3 cam33 = cameraLookAtMatrix.getUpperLeft3x3Matrix();
                Matrix4 camOrientationMatrix(cam33.getColumn(0)[0], cam33.getColumn(1)[0], cam33.getColumn(2)[0],
                                             cameraPosition.x,
                                             cam33.getColumn(0)[1], cam33.getColumn(1)[1], cam33.getColumn(2)[1],
                                             cameraPosition.y,
                                             cam33.getColumn(0)[2], cam33.getColumn(1)[2], cam33.getColumn(2)[2],
                                             cameraPosition.z,
                                             0, 0, 0, 1);
//                cameraLookAtMatrix.m[3][3] = 1;
//                cameraLookAtMatrix = mCamera.getTransformMatrix() * camOrientationMatrix;
                mCamera.setTransformMatrix(camOrientationMatrix);

                cameraLookAtMatrix.print();
                break;
        }
    }

    if (action == GLFW_RELEASE) {
        switch (key) {
            case GLFW_KEY_W:
            case GLFW_KEY_S:
                testPRY.setPitch(1500.0);
                break;
            case GLFW_KEY_A:
            case GLFW_KEY_D:
                testPRY.setRoll(1500.0);
                break;
            case GLFW_KEY_Q:
            case GLFW_KEY_E:
                testPRY.setYaw(1500.0);
                break;
        }
    }

    mDrone->setInputParams(testPRY, quadThrottle);

    return false;
}


#include "QuadScene.h"
