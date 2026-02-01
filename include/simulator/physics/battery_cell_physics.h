#ifndef BATTERY_CELL_PHYSICS_H
#define BATTERY_CELL_PHYSICS_H

#include "drone/model/components/battery_cell.h"

using namespace drone::model::components;

namespace drone::simulator::physics {

class BatteryCellPhysics {
public:
    friend class Battery_Cell;

    static void calculateVoltageDrop(const Battery_Cell& cell);
    static void setCurrentA(Battery_Cell& cell, double current_a);

};

} // namespace drone::simulator::physics


#endif // BATTERY_CELL_PHYSICS_H