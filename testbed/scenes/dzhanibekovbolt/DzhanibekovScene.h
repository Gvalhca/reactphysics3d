
#ifndef REACTPHYSICS3D_DZHANIBEKOVSCENE_H
#define REACTPHYSICS3D_DZHANIBEKOVSCENE_H
#define TEST_FLAG 0
// Libraries
#include "openglframework.h"
#include "reactphysics3d.h"
#include "Box.h"
#include "SceneDemo.h"
#include "Sphere.h"
#include "Drone.h"

namespace dzhanibekovscene {
using namespace drone;
// Constants
    const float SCENE_RADIUS = 30.0f;
    const openglframework::Vector3 BOX_SIZE(2, 2, 2);           // Box dimensions in meters
    const openglframework::Vector3 FLOOR_SIZE(50, 0.5f, 50);    // Floor dimensions in meters
    const float BOX_MASS = 1.0f;                                // Box mass in kilograms
    const float FLOOR_MASS = 100.0f;                            // Floor mass in kilograms
    const int NB_BALLSOCKETJOINT_BOXES = 7;                     // Number of Ball-And-Socket chain boxes
    const int NB_HINGE_BOXES = 7;                               // Number of Hinge chain boxes

// Class DzhanibekovsScene
    class DzhanibekovScene : public SceneDemo {

    protected :

        // -------------------- Attributes -------------------- //

        /// Box for the floor
        Box* mFloor;

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
        DzhanibekovScene(const std::string& name, EngineSettings& settings);

        /// Destructor
        virtual ~DzhanibekovScene() override;

        /// Update the physics world (take a simulation step)
        /// Can be called several times per frame
        virtual void updatePhysics() override;

        /// Reset the scene
        virtual void reset() override;

        /// Return all the contact points of the scene
//        virtual std::vector<ContactPoint> getContactPoints() override;
    };

// Return all the contact points of the scene
//    inline std::vector<ContactPoint> DzhanibekovScene::getContactPoints() {
//        return computeContactPointsOfWorld(getDynamicsWorld());
//    }

}


#endif //REACTPHYSICS3D_DZHANIBEKOVSCENE_H
