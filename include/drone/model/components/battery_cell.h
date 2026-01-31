#ifndef BATTERY_CELL_H
#define BATTERY_CELL_H

#include <string>

namespace drone::model::components {

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
        calculateVoltageDrop();
    }

    /**
     * @brief Calculates the voltage drop based on the state of charge.
     */
    virtual void calculateVoltageDrop() {
        /**
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
        // Simple linear approximation for demonstration purposes
        voltage_v_ = 3.9;
        if (state_of_charge_percent_ > 85.0) {
            double deltaV = (4.2 - 3.9);
            voltage_v_ = 3.9 + ((state_of_charge_percent_ - 85.0) / 15.0) * deltaV;   
        }
        else if (state_of_charge_percent_ <= 85.0 && state_of_charge_percent_ >= 30.0) {
            double deltaV = (3.9 - 3.5);
            voltage_v_ = 3.9 - ((85.0 - state_of_charge_percent_) / 55.0) * deltaV;
        }        
        else if (state_of_charge_percent_ < 30.0) {
            double deltaV = (3.5 - 3.2);
            voltage_v_ = 3.5 - ((30.0 - state_of_charge_percent_) / 30.0) * deltaV;
        }
    }

    /**
     * @brief Updates the battery cell state.
     * @param delta_time_ms Time elapsed since last update in milliseconds.
     */
    virtual void update(int delta_time_ms = 1000) {
        // depending on current, lower the state of charge
        capacity_mah_ -= (current_a_ * (delta_time_ms / 3600000.0)) * 1000; // Convert ms to hours, mA
        if (capacity_mah_ < 0) {
            capacity_mah_ = 0;
        }
        state_of_charge_percent_ = (capacity_mah_ / nominal_capacity_mah_) * 100.0;

        return;
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