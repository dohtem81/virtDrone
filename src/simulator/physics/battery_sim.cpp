#include "simulator/physics/battery_sim.h"

#include "simulator/physics/battery_cell_physics.h"

namespace drone::simulator::physics {

BatterySim::BatterySim(const std::string& name, const drone::model::components::BatterySpecs& specs)
    : name_(name), specs_(specs) {
    cells_.reserve(specs_.cells);
    for (int i = 0; i < specs_.cells; ++i) {
        cells_.emplace_back(
            "Cell_" + std::to_string(i + 1),
            specs_.cell_specs.capacity_mah,
            specs_.cell_specs.nominal_voltage_v);
    }
}

std::string BatterySim::getName() const {
    return name_;
}

double BatterySim::getVoltageV() const {
    double total_voltage = 0.0;
    for (const auto& cell : cells_) {
        total_voltage += (cell.getCapacityMah() < 0.1 ? 0.0 : cell.getVoltageV()); // If capacity is very low, consider voltage to be 0 to simulate cutoff
    }
    return total_voltage;
}

double BatterySim::getCurrentA() const {
    if (cells_.empty()) {
        return 0.0;
    }
    return cells_.front().getCurrentA();
}

double BatterySim::getStateOfChargePercent() const {
    if (cells_.empty()) {
        return 0.0;
    }

    double total_soc = 0.0;
    for (const auto& cell : cells_) {
        total_soc += cell.getStateOfChargePercent();
    }
    return total_soc / cells_.size();
}

double BatterySim::getRemainingCapacityMah() const {
    if (cells_.empty()) {
        return 0.0;
    }

    double total_capacity = 0.0;
    for (const auto& cell : cells_) {
        total_capacity += cell.getRemainingCapacityMah();
    }
    return total_capacity / cells_.size();
}

double BatterySim::getRemainingEnergyWh() const {
    double total_energy_wh = 0.0;
    for (const auto& cell : cells_) {
        total_energy_wh += (cell.getRemainingCapacityMah() / 1000.0) * cell.getVoltageV();
    }
    return total_energy_wh;
}

double BatterySim::getWeightKg() const {
    return specs_.weight_kg;
}

void BatterySim::setCurrentA(double current_a) {
    for (auto& cell : cells_) {
        BatteryCellPhysics::setCurrentA(cell, current_a);
    }
}

void BatterySim::setStateOfChargePercent(double soc_percent) {
    for (auto& cell : cells_) {
        BatteryCellPhysics::setStateOfChargePercent(cell, soc_percent);
    }
}

void BatterySim::update(int delta_time_ms) {
    for (auto& cell : cells_) {
        BatteryCellPhysics::update(cell, delta_time_ms);
    }
}

} // namespace drone::simulator::physics
