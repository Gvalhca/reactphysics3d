//
// Created by kirill on 02.04.2020.
//

#ifndef REACTPHYSICS3D_QUADPIDS_H
#define REACTPHYSICS3D_QUADPIDS_H

#include <map>
#include "pid.h"

namespace quad {

    typedef enum {
        PITCH_PID = 0,
        ROLL_PID = 1,
        YAW_PID = 2,
        HOVER_PID = 3
    } pidTypes;

    class QuadPIDs {
    private:
        std::map<pidTypes, PID*> _pids;

        PID& getPid(pidTypes pidType) const;

    public:
        QuadPIDs(const PID& pitchPID, const PID& rollPID, const PID& yawPID,
                 const PID& hoverPID = PID(0.7, 0.35, 0.35));

        ~QuadPIDs();

        QuadPIDs(const QuadPIDs&);

        PID& operator[](pidTypes pidType);

        const PID& operator[](pidTypes pidType) const;

        void reset();
    };
}

#endif //REACTPHYSICS3D_QUADPIDS_H
