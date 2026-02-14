#include "drone/model/components/battery_cell.h"
#include "simulator/physics/battery_cell_physics.h"

namespace drone::simulator::physics {
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
void BatteryCellPhysics::calculateVoltageDrop(Battery_Cell& cell) {
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

    // update cell voltage
    cell.voltage_v_ = voltage_v;
}

/**
 * @brief Sets the current in amperes for the battery cell.
 * @param cell The battery cell object.
 * @param current_a The current in A.
 */
void BatteryCellPhysics::setCurrentA(Battery_Cell& cell, double current_a) {
    cell.setCurrentA(current_a);
}

/**
 * @brief Sets state of charge and recalculates voltage.
 * @param cell The battery cell object.
 * @param soc_percent The state of charge in percent.
 */
void BatteryCellPhysics::setStateOfChargePercent(Battery_Cell& cell, double soc_percent) {
    if (soc_percent < 0.0) {
        soc_percent = 0.0;
    }
    if (soc_percent > 100.0) {
        soc_percent = 100.0;
    }

    cell.setStateOfChargePercent(soc_percent);
    calculateVoltageDrop(cell);
}

/**
 * @brief Updates the battery cell state.
 * @param delta_time_ms Time elapsed since last update in milliseconds.
 */
void BatteryCellPhysics::update(Battery_Cell& cell, int delta_time_ms) {
    // depending on current, lower the state of charge, reclaculate voltage and state of charge

    cell.capacity_mah_ -= (cell.getCurrentA() * (delta_time_ms / 3600000.0)) * 1000; // Convert ms to hours, mA
    if (cell.capacity_mah_ < 0) {
        cell.capacity_mah_ = 0;
    }

    double soc_percent = 0.0;
    if (cell.nominal_capacity_mah_ > 0.0) {
        soc_percent = (cell.capacity_mah_ / cell.nominal_capacity_mah_) * 100.0;
    }

    cell.state_of_charge_percent_ = soc_percent;
    calculateVoltageDrop(cell);
}

} // namespace drone::simulator::physics