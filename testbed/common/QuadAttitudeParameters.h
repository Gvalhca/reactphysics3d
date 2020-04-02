//
// Created by kirill on 02.04.2020.
//

#ifndef REACTPHYSICS3D_QUADATTITUDEPARAMETERS_H
#define REACTPHYSICS3D_QUADATTITUDEPARAMETERS_H


#include <reactphysics3d.h>

namespace drone {
    typedef enum {
        PITCH = 0,
        ROLL = 1,
        YAW = 2
    } flightAxis;

    class QuadAttitudeParameters {
    private:
        double _altitude;
        rp3d::Vector3 _axis;

    public:
        QuadAttitudeParameters(double altitude, const rp3d::Vector3& axisPRY);

        QuadAttitudeParameters(const QuadAttitudeParameters&);

        void setParameters(double altitude, const rp3d::Vector3& axisPRY);

        void setAltitude(double altitude);

        void setAxis(const rp3d::Vector3& axisPRY);

        double getAltitude() const;

        rp3d::Vector3 getAxis() const;
    };
}

#endif //REACTPHYSICS3D_QUADATTITUDEPARAMETERS_H
