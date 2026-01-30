#ifndef BATTERY_CELL_H
#define BATTERY_CELL_H

#include <string>

namespace drone::model::components {

class Battery_Cell {
public:
    /**
     * @brief Constructor for Battery_Cell.
     * @param cell_id The identifier for the battery cell.
     * @param capacity_mah The capacity of the cell in milliamp-hours.
     * @param nominal_voltage_v The nominal voltage of the cell in volts.
     */
    Battery_Cell(const std::string& cell_id, double capacity_mah, double nominal_voltage_v)
        : cell_id_(cell_id), capacity_mah_(capacity_mah), nominal_voltage_v_(nominal_voltage_v) {}

    /**
     * @brief Gets the cell identifier.
     * @return The cell ID as a string.
     */
    std::string getCellID() const { return cell_id_; }

    /**
     * @brief Gets the capacity of the cell in milliamp-hours.
     * @return The capacity in mAh.
     */
    double getCapacityMah() const { return capacity_mah_; }

    /**
     * @brief Gets the nominal voltage of the cell in volts.
     * @return The nominal voltage in V.
     */
    double getNominalVoltageV() const { return nominal_voltage_v_; }

    /**
     * @brief Gets the current in amperes.
     * @return The current in A.
     */
    double getCurrentA() const { return 0; } // Placeholder implementation;

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
    double getStateOfChargePercent() const { return 100.0; } // Placeholder implementation;

private:
    std::string cell_id_;
    double capacity_mah_;
    double nominal_voltage_v_;

};
} 
#endif // BATTERY_CELL_H