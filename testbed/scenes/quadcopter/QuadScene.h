
#ifndef REACTPHYSICS3D_QUADSCENE_H
#define REACTPHYSICS3D_QUADSCENE_H
#define TEST_FLAG 0
// Libraries
#include "openglframework.h"
#include "reactphysics3d.h"
#include "Box.h"
#include "SceneDemo.h"
#include "Sphere.h"
#include "quadCopter/Drone.h"


namespace quadscene {
using namespace quad;
// Constants
    const float SCENE_RADIUS = 30.0f;
    const openglframework::Vector3 BOX_SIZE(2, 2, 2);           // Box dimensions in meters
    const openglframework::Vector3 FLOOR_SIZE(50, 0.5f, 50); // Floor dimensions in meters
    const openglframework::Vector3 WALL_SIZE(50, 50, 0.5f); // Floor dimensions in meters
    const openglframework::Vector3 TESTBOX_SIZE(0.5, 2.5, 0.1); // Floor dimensions in meters
    const float BOX_MASS = 1.0f;                                // Box mass in kilograms
    const float FLOOR_MASS = 100.0f;                            // Floor mass in kilograms
    const int NB_BALLSOCKETJOINT_BOXES = 7;                     // Number of Ball-And-Socket chain boxes
    const int NB_HINGE_BOXES = 7;                               // Number of Hinge chain boxes

// Class QuadScene
    class QuadScene : public SceneDemo {
    private:
        openglframework::Vector3 cameraAngles;

    protected :

        // -------------------- Attributes -------------------- //

        /// Box for the floor
        Box* mFloor;

        Box* mWall;

        Box* testBox;

        float initialHeight = 5;

        Drone* mDrone;
        double simStartTime, simElapsedTime;

        // -------------------- Methods -------------------- //

        /// Create the floor
        void createFloor();

        void createDrone();

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        QuadScene(const std::string& name, EngineSettings& settings);

        /// Destructor
        virtual ~QuadScene() override;

        /// Update the physics world (take a simulation step)
        /// Can be called several times per frame
        virtual void updatePhysics() override;

        /// Called when a keyboard event occurs
        virtual bool keyboardEvent(int key, int scancode, int action, int mods) override;

        /// Reset the scene
        virtual void reset() override;

        /// Return all the contact points of the scene
//        virtual std::vector<ContactPoint> getContactPoints() override;
        void createWall();

        void createTestBox();

        openglframework::Matrix4&
        fLookAt(const openglframework::Vector3& target, const openglframework::Vector3& upVec);

        void refreshCamera();

        void SaveImage();
    };

// Return all the contact points of the scene
//    inline std::vector<ContactPoint> QuadScene::getContactPoints() {
//        return computeContactPointsOfWorld(getDynamicsWorld());
//    }

}


#endif //REACTPHYSICS3D_QUADSCENE_H
