#pragma once

#include "pros/motors.hpp"
#include "../Constants.hpp"

using namespace pros;
using namespace Constants;

class Indexer {//indexer is at 0 pos, when we want to move we will move certain motor values and move back to normal
    private:
        Motor index = Motor(index_p);
    public:
        Indexer() {
            index.set_brake_mode(E_MOTOR_BRAKE_HOLD);
            index.tare_position();
        }
        inline void launch() {
            index.move_absolute(50);
        }
        inline void reset() {
            index.move_absolute(0, 50);
        }
};
