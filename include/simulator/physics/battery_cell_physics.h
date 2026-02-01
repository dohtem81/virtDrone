#ifndef BATTERY_CELL_PHYSICS_H
#define BATTERY_CELL_PHYSICS_H

#include "drone/model/components/battery_cell.h"

namespace drone::simulator::physics {
using namespace drone::model::components;

class BatteryCellPhysics {
public:
    friend class Battery_Cell;

    static void calculateVoltageDrop(const Battery_Cell& cell);
    static void setCurrentA(Battery_Cell& cell, double current_a);
    static void update(Battery_Cell& cell, int delta_time_ms);

};

} // namespace drone::simulator::physics


#endif // BATTERY_CELL_PHYSICS_H