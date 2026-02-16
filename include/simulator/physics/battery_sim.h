#ifndef BATTERY_SIM_H
#define BATTERY_SIM_H

#include <string>
#include <vector>

#include "drone/model/components/battery_base.h"
#include "drone/model/components/battery_cell.h"

namespace drone::simulator::physics {

class BatterySim final : public drone::model::components::Battery_base {
public:
    BatterySim(const std::string& name, const drone::model::components::BatterySpecs& specs);

    std::string getName() const override;
    double getVoltageV() const override;
    double getCurrentA() const override;
    double getStateOfChargePercent() const override;
    double getRemainingCapacityMah() const override;
    double getRemainingEnergyWh() const override;
    double getWeightKg() const override;

    void setCurrentA(double current_a);
    void setStateOfChargePercent(double soc_percent);
    void update(int delta_time_ms = 1000);

private:
    std::string name_;
    drone::model::components::BatterySpecs specs_;
    std::vector<drone::model::components::Battery_Cell> cells_;
};

} // namespace drone::simulator::physics

#endif // BATTERY_SIM_H
