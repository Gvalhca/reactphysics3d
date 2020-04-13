
#include "QuadPIDs.h"

namespace quad {

    QuadPIDs::QuadPIDs(const PID& pitchPID, const PID& rollPID, const PID& yawPID, const PID& hoverPID) {
        _pids[PITCH_PID] = std::make_shared<PID>(pitchPID);
        _pids[ROLL_PID] = std::make_shared<PID>(rollPID);
        _pids[YAW_PID] = std::make_shared<PID>(yawPID);
        _pids[HOVER_PID] = std::make_shared<PID>(hoverPID);


    }

    PID& QuadPIDs::getPid(pidTypes pidType) const {
        return *(_pids.at(pidType));
    }

    QuadPIDs::QuadPIDs(const QuadPIDs& quadPIDs) {
        for (int i = 0; i <= HOVER_PID; ++i) {
            auto pidType = (pidTypes) i;
            _pids[pidType] = std::make_shared<PID>(quadPIDs[pidType]);
        }
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
//        for (auto& pid : _pids) {
//            delete pid.second;
//        }
        _pids.clear();
    }





}