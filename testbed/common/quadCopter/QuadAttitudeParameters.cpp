//
// Created by kirill on 02.04.2020.
//

#include "QuadAttitudeParameters.h"

namespace drone {
    QuadAttitudeParameters::QuadAttitudeParameters(double altitude, const rp3d::Vector3& axisPRY) : _altitude(altitude),
                                                                                                    _axisPRY(axisPRY) {}

    void QuadAttitudeParameters::setParameters(double altitude, const rp3d::Vector3& axisPRY) {
        setAltitude(altitude);
        setAxisPRY(axisPRY);
    }

    QuadAttitudeParameters::QuadAttitudeParameters(const QuadAttitudeParameters& attitudeParameters) {
        _altitude = attitudeParameters._altitude;
        _axisPRY = attitudeParameters._axisPRY;
    }

    double QuadAttitudeParameters::getAltitude() const {
        return _altitude;
    }

    rp3d::Vector3 QuadAttitudeParameters::getAxisPRY() const {
        return _axisPRY;
    }

    void QuadAttitudeParameters::setAltitude(double altitude) {
        _altitude = altitude;
    }

    void QuadAttitudeParameters::setAxisPRY(const rp3d::Vector3& axisPRY) {
        _axisPRY = axisPRY;
    }

    rp3d::Vector3 QuadAttitudeParameters::getAngularVelocity() const {
        return _angularVelocity;
    }

    void QuadAttitudeParameters::setAngularVelocity(const rp3d::Vector3& angularVelocity) {
        _angularVelocity = angularVelocity;
    }
}