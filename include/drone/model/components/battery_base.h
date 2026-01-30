#ifndef BATTERY_BASE_H
#define BATTERY_BASE_H

#include <string>
#include <vector>
#include "battery_cell.h"

namespace drone::model::components {

/**
 * @brief Struct for battery specifications.
 */
struct BatterySpecs {
    double capacity_mah;      ///< Battery capacity in milliamp-hours.
    double nominal_voltage_v; ///< Nominal voltage in volts.
    double max_discharge_rate_c; ///< Maximum discharge rate in C (capacity multiplier).

    BatterySpecs(double cap = 5000.0, double nom_vol = 14.8, double max_disch = 1.0)
        : capacity_mah(cap), nominal_voltage_v(nom_vol), max_discharge_rate_c(max_disch) {}
};

/**
 * @brief Abstract base class for battery components.
 * 
 * This class provides a base for modeling battery behavior, including voltage, current,
 * capacity, and state of charge.
 */
class Battery_base {
public:
    /**
     * @brief Constructor for Battery_base.
     * @param name The name of the battery.
     * @param specs The battery specifications.
     */
    Battery_base(const std::string& name, const BatterySpecs& specs);

    /**
     * @brief Virtual destructor.
     */
    virtual ~Battery_base() = default;

    /**
     * @brief Pure virtual update method for battery simulation.
     * 
     * Derived classes must implement this to update battery state.
     */
    virtual void update() = 0;

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
            total_voltage += cell.getNominalVoltageV();
        }
        return total_voltage;
     }

    /**
     * @brief Gets the current discharge current in amperes.
     * @return The current in A.
     */
    virtual double getCurrentA() const = 0;

    /**
     * @brief Gets the remaining capacity in milliamp-hours.
     * @return The capacity in mAh.
     */
    double getRemainingCapacityMah(){
        double total_capacity = 0.0;
        for (const auto& cell : cells_) {
            total_capacity += cell.getRemainingCapacityMah();
        }
        return total_capacity;
    }

    /**
     * @brief Gets the state of charge as a fraction (0.0 to 1.0).
     * @return The SOC.
     */
    virtual double getStateOfCharge() const = 0;

    /**
     * @brief Gets the battery specifications.
     * @return The specs.
     */
    BatterySpecs getSpecs() const { return specs_; }

protected:
    std::string name_;
    BatterySpecs specs_;
    std::vector<Battery_Cell> cells_;
};

}  // namespace drone::model::components

#endif // BATTERY_BASE_H