//
// Created by kirill on 02.04.2020.
//

#include "QuadAttitudeParameters.h"

namespace drone {
    QuadAttitudeParameters::QuadAttitudeParameters(double altitude, const rp3d::Vector3& axisPRY) : _altitude(altitude),
                                                                                                    _axis(axisPRY) {}

    void QuadAttitudeParameters::setParameters(double altitude, const rp3d::Vector3& axisPRY) {
        setAltitude(altitude);
        setAxis(axisPRY);
    }

    QuadAttitudeParameters::QuadAttitudeParameters(const QuadAttitudeParameters& attitudeParameters) {
        _altitude = attitudeParameters._altitude;
        _axis = attitudeParameters._axis;
    }

    double QuadAttitudeParameters::getAltitude() const {
        return _altitude;
    }

    rp3d::Vector3 QuadAttitudeParameters::getAxis() const {
        return _axis;
    }

    void QuadAttitudeParameters::setAltitude(double altitude) {
        _altitude = altitude;
    }

    void QuadAttitudeParameters::setAxis(const rp3d::Vector3& axisPRY) {
        _axis = axisPRY;
    }
}