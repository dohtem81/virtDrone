#ifndef BATTERY_CELL_H
#define BATTERY_CELL_H

#include <string>
#include "simulator/physics/battery_cell_physics.h"

namespace drone::model::components {
using namespace drone::simulator::physics;

struct CellSpecs {
    double capacity_mah;       // Capacity in milliamp-hours
    double nominal_voltage_v;  // Nominal voltage in volts

    CellSpecs(double cap = 1500.0, double nom_vol = 4.2)
        : capacity_mah(cap), nominal_voltage_v(nom_vol) {}
};

class Battery_Cell {
public:
    /**
     * @brief Constructor for Battery_Cell.
     * @param cell_id The identifier for the battery cell.
     * @param capacity_mah The capacity of the cell in milliamp-hours.
     * @param nominal_voltage_v The nominal voltage of the cell in volts.
     */
    Battery_Cell(const std::string& cell_id, double capacity_mah, double nominal_voltage_v)
        : cell_id_(cell_id), nominal_capacity_mah_(capacity_mah), nominal_voltage_v_(nominal_voltage_v), state_of_charge_percent_(100.0), current_a_(0.0), capacity_mah_(capacity_mah), voltage_v_(nominal_voltage_v) {}

    /**
     * @brief Constructor for Battery_Cell using CellSpecs.
     * @param cell_id The identifier for the battery cell.
     * @param specs The specifications for the cell.
     */     
    Battery_Cell(const std::string& cell_id, const CellSpecs& specs)
        : cell_id_(cell_id), nominal_capacity_mah_(specs.capacity_mah), nominal_voltage_v_(specs.nominal_voltage_v), state_of_charge_percent_(100.0), current_a_(0.0), capacity_mah_(specs.capacity_mah), voltage_v_(specs.nominal_voltage_v) {}
    
    /**
     * @brief Gets the cell identifier.
     * @return The cell ID as a string.
     */
    std::string getCellID() const { return cell_id_; }

    /**
     * @brief Gets the capacity of the cell in milliamp-hours.
     * @return The capacity in mAh.
     */
    double getCapacityMah() const { return nominal_capacity_mah_; }

    /**
     * @brief Gets the nominal voltage of the cell in volts.
     * @return The nominal voltage in V.
     */
    double getNominalVoltageV() const { return nominal_voltage_v_; }

    /**
     * @brief Gets the current voltage of the cell in volts.
     * @return The voltage in V.
     */
    double getVoltageV() const { return voltage_v_; }

    /**
     * @brief Gets the current in amperes.
     * @return The current in A.
     */
    double getCurrentA() const { return current_a_; } // Placeholder implementation;

    /**
     * @brief Gets the remaining capacity in milliamp-hours.
     * @return The remaining capacity in mAh.
     */
    double getRemainingCapacityMah() const {
        return capacity_mah_;
    }

    /**
     * @brief Gets the state of charge as a percentage.
     * @return The state of charge in %.
     */
    double getStateOfChargePercent() const { return state_of_charge_percent_; } // Placeholder implementation;

    /**
     * @brief Sets the current in amperes. It's virtual to avoid exposing this one in actual model implementation.
     * this should be used only by simulation/testing purposes.
     * @param current_a The current in A.
     */
    virtual void setCurrentA(double current_a) {
        current_a_ = current_a;
    }

    /**
     * @brief Sets the state of charge as a percentage.
     * @param soc_percent The state of charge in %.
     */
    virtual void setStateOfChargePercent(double soc_percent) {
        state_of_charge_percent_ = soc_percent;
        capacity_mah_ = (state_of_charge_percent_ / 100.0) * nominal_capacity_mah_;
        BatteryCellPhysics::calculateVoltageDrop(*this);
    }

private:
    std::string cell_id_;
    double nominal_capacity_mah_;
    double nominal_voltage_v_;
    double voltage_v_;
    double current_a_;
    double state_of_charge_percent_;
    double capacity_mah_;

};
} 
#endif // BATTERY_CELL_H