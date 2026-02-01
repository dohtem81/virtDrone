#ifndef BATTERY_BASE_H
#define BATTERY_BASE_H

#include <string>
#include <vector>
#include <memory>
#include "battery_cell.h"
#include "battery_cell.h"

namespace drone::model::components {

/**
 * @brief Struct for battery specifications.
 */
struct BatterySpecs {
    double capacity_mah;      ///< Battery capacity in milliamp-hours.
    double nominal_voltage_v; ///< Nominal voltage in volts.
    double max_discharge_rate_c; ///< Maximum discharge rate in C (capacity multiplier).
    int cells;               ///< Number of cells in the battery.
    CellSpecs cell_specs;  ///< Specifications for individual cells.

    BatterySpecs(int cells = 4, CellSpecs cell_specs = CellSpecs())
        : cells(cells), cell_specs(cell_specs), max_discharge_rate_c(1.0) {
        
            // assumption is that cells are in series
            capacity_mah = cell_specs.capacity_mah;
            nominal_voltage_v = cells * cell_specs.nominal_voltage_v;
        }
};

/**
 * @brief Abstract base class for battery components.
 * 
 * This class provides a base for modeling battery behavior, including voltage, current,
 * capacity, and state of charge.
 */
class Battery_base {
public:
    Battery_base(const std::string& name, const BatterySpecs& specs)
    {
        name_ = name;
        specs_ = specs;
        for (int i = 0; i < specs.cells; ++i) {
            std::shared_ptr<Battery_Cell> newCell = std::make_shared<Battery_Cell>("Cell_" + std::to_string(i + 1), specs.cell_specs.capacity_mah, specs.cell_specs.nominal_voltage_v);
            cells_.push_back(*newCell);
        }
    };
    virtual ~Battery_base() = default;

    // Getters
    /**
     * @brief Gets the battery name.
     * @return The name as a string.
     */
    std::string getName() const { return name_; }

    /**
     * @brief Gets the current voltage in volts.
     * @return The voltage in V.
     */
    double getVoltageV(){ 
        double total_voltage = 0.0;
        for (const auto& cell : cells_) {
            total_voltage += cell.getVoltageV();
        }
        return total_voltage;
    }

    /**
     * @brief Gets the remaining capacity in milliamp-hours.
     * @return The capacity in mAh.
     */
    double getRemainingCapacityMah(){
        if (!cells_.empty()) {
            return cells_.front().getRemainingCapacityMah();
        }        
        return 0.0;
    }    

    /**
     * @brief Sets the current discharge current in amperes for all cells.
     * @param current_a The current in A.
     */
    virtual void setCurrentA(double current_a) {
        for (auto& cell : cells_) {
            cell.setCurrentA(current_a);
        }
    }

    /**
     * @brief Gets the current discharge current in amperes.
     * @return The current in A.
     */
    double getCurrentA(){
        // Assuming all cells have the same current in series configuration
        if (!cells_.empty()) {
            return cells_.front().getCurrentA();
        }
        return 0.0;
    }

    /**
     * @brief Gets the state of charge as a percentage.
     * @return The state of charge in %.
     */
    double getStateOfChargePercent(){
        if (!cells_.empty()) {
            return cells_.front().getStateOfChargePercent();
        }        
        return 0.0;
    }

    /**
     * @brief Sets the state of charge as a percentage for all cells.
     * @param soc_percent The state of charge in %.
     */
    virtual void setStateOfChargePercent(double soc_percent) {
        for (auto& cell : cells_) {
            cell.setStateOfChargePercent(soc_percent);
        }
    }

    // /**
    //  * @brief Gets the remaining capacity in milliamp-hours.
    //  * @return The capacity in mAh.
    //  */
    // double getRemainingCapacityMah(){
    //     double total_capacity = 0.0;
    //     for (const auto& cell : cells_) {
    //         total_capacity += cell.getRemainingCapacityMah();
    //     }
    //     return total_capacity;
    // }

    // /**
    //  * @brief Gets the state of charge as a fraction (0.0 to 1.0).
    //  * @return The SOC.
    //  */
    // virtual double getStateOfCharge() const = 0;

    // /**
    //  * @brief Gets the battery specifications.
    //  * @return The specs.
    //  */
    // BatterySpecs getSpecs() const { return specs_; }

    virtual void update(int delta_time_ms = 1000) {
        // for (auto& cell : cells_) {
        //     cell.update(delta_time_ms);
        // }
    }

protected:
    std::string name_;
    BatterySpecs specs_;
    std::vector<Battery_Cell> cells_;
};

}  // namespace drone::model::components

#endif // BATTERY_BASE_H