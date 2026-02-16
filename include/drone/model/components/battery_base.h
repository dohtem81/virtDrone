#ifndef BATTERY_BASE_H
#define BATTERY_BASE_H

#include <string>
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
    double weight_kg;        ///< Battery weight in kilograms.

    BatterySpecs(int cells = 4, CellSpecs cell_specs = CellSpecs(), double weight_kg = 0.0)
        : cells(cells), cell_specs(cell_specs), max_discharge_rate_c(1.0), weight_kg(weight_kg) {
        
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
    virtual ~Battery_base() = default;

    virtual std::string getName() const = 0;
    virtual double getVoltageV() const = 0;
    virtual double getCurrentA() const = 0;
    virtual double getStateOfChargePercent() const = 0;
    virtual double getRemainingCapacityMah() const = 0;
    virtual double getRemainingEnergyWh() const = 0;
    virtual double getWeightKg() const = 0;
};

}  // namespace drone::model::components

#endif // BATTERY_BASE_H