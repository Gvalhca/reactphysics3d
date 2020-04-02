
#include "QuadPids.h"

namespace drone {

    QuadPids::QuadPids(const PID& pitchPID, const PID& rollPID, const PID& yawPID, const PID& hoverPID) {
        _pids[PITCH_PID] = new PID(pitchPID);
        _pids[ROLL_PID] = new PID(rollPID);
        _pids[YAW_PID] = new PID(yawPID);
        _pids[HOVER_PID] = new PID(hoverPID);
    }

    PID& QuadPids::getPid(pidTypes pidType) const {
        return *(_pids.at(pidType));
    }

    QuadPids::QuadPids(const QuadPids& quadPids) {
        _pids[PITCH_PID] = new PID(quadPids[PITCH_PID]);
        _pids[ROLL_PID] = new PID(quadPids[ROLL_PID]);
        _pids[YAW_PID] = new PID(quadPids[YAW_PID]);
        _pids[HOVER_PID] = new PID(quadPids[HOVER_PID]);
    }

    const PID& QuadPids::operator[](pidTypes pidType) const {
        return getPid(pidType);
    }

    PID& QuadPids::operator[](pidTypes pidType) {
        return getPid(pidType);
    }

    void QuadPids::reset() {
        for (const auto& item : _pids) {
            item.second->reset();
        }
    }

}