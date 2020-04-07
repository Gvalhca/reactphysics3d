//
// Created by kirill on 02.04.2020.
//

#ifndef REACTPHYSICS3D_QUADATTITUDEPARAMETERS_H
#define REACTPHYSICS3D_QUADATTITUDEPARAMETERS_H


#include <reactphysics3d.h>

namespace quad {
    typedef enum {
        PITCH = 0,
        ROLL = 1,
        YAW = 2
    } flightAxis;

    class QuadAttitudeParameters {
    private:
        double _altitude;
        rp3d::Vector3 _axisPRY;
        rp3d::Vector3 _angularVelocity;

    public:
        explicit QuadAttitudeParameters(double altitude = 0, const rp3d::Vector3& axisPRY = rp3d::Vector3::zero());

        ~QuadAttitudeParameters() = default;

        QuadAttitudeParameters(const QuadAttitudeParameters&);

        void setParameters(double altitude, const rp3d::Vector3& axisPRY);

        void setAltitude(double altitude);

        void setAxisPRY(const rp3d::Vector3& axisPRY);

        void setAngularVelocity(const rp3d::Vector3& angularVelocity);

        double getAltitude() const;

        rp3d::Vector3 getAxisPRY() const;

        rp3d::Vector3 getAngularVelocity() const;
    };
}

#endif //REACTPHYSICS3D_QUADATTITUDEPARAMETERS_H
