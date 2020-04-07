
#include "QuadPIDs.h"

namespace drone {

    QuadPIDs::QuadPIDs(const PID& pitchPID, const PID& rollPID, const PID& yawPID, const PID& hoverPID) {
        _pids[PITCH_PID] = new PID(pitchPID);
        _pids[ROLL_PID] = new PID(rollPID);
        _pids[YAW_PID] = new PID(yawPID);
        _pids[HOVER_PID] = new PID(hoverPID);
    }

    PID& QuadPIDs::getPid(pidTypes pidType) const {
        return *(_pids.at(pidType));
    }

    QuadPIDs::QuadPIDs(const QuadPIDs& quadPids) {
        _pids[PITCH_PID] = new PID(quadPids[PITCH_PID]);
        _pids[ROLL_PID] = new PID(quadPids[ROLL_PID]);
        _pids[YAW_PID] = new PID(quadPids[YAW_PID]);
        _pids[HOVER_PID] = new PID(quadPids[HOVER_PID]);
    }

    const PID& QuadPIDs::operator[](pidTypes pidType) const {
        return getPid(pidType);
    }

    PID& QuadPIDs::operator[](pidTypes pidType) {
        return getPid(pidType);
    }

    void QuadPIDs::reset() {
        for (const auto& item : _pids) {
            item.second->reset();
        }
    }

    QuadPIDs::~QuadPIDs() {
        for (auto& pid : _pids) {
            delete pid.second;
        }
        _pids.clear();
    }

}