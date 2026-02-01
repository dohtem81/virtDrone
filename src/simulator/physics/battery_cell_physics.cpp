#include "simulator/physics/battery_cell_physics.h"
    
/**
 * @brief Calculates the voltage drop based on the state of charge.
 * 
 * Top knee: 100 → ~85% (voltage falls fast)
 * Flat plateau: ~85 → 30% (slow change) - voltage 3.9->3.5V
 * Bottom knee: <30% (voltage collapses)
 * 
 *   4.2V ┐
 *       │\        ← quick drop after takeoff
 *       │ \
 *   3.9V │  ────── plateau (most of flight)
 *       │         \
 *   3.5V │          \ ← cliff near empty
 *       └────────────
 *       0%        100%
*/
void BatteryCellPhysics::calculateVoltageDrop(const Battery_Cell& cell) {
    double voltage_v = 3.9;
    double state_of_charge_percent = cell.getStateOfChargePercent();
    if (state_of_charge_percent > 85.0) {
        double deltaV = (4.2 - 3.9);
        voltage_v = 3.9 + ((state_of_charge_percent - 85.0) / 15.0) * deltaV;   
    }
    else if (state_of_charge_percent <= 85.0 && state_of_charge_percent >= 30.0) {
        double deltaV = (3.9 - 3.5);
        voltage_v = 3.9 - ((85.0 - state_of_charge_percent) / 55.0) * deltaV;
    }        
    else if (state_of_charge_percent < 30.0) {
        double deltaV = (3.5 - 3.2);
        voltage_v = 3.5 - ((30.0 - state_of_charge_percent) / 30.0) * deltaV;
    }
    return voltage_v;
}

/**
 * @brief Sets the current in amperes for the battery cell.
 * @param cell The battery cell object.
 * @param current_a The current in A.
 */
void BatteryCellPhysics::setCurrentA(Battery_Cell& cell, double current_a) {
    cell.setCurrentA(current_a);
}