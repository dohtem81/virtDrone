#ifndef GPS_SIM_H
#define GPS_SIM_H

#include "drone/model/components/gps_module_base.h"

namespace drone::simulator::physics {

class GPSSim final : public drone::model::components::GPSModule_base {
public:
    GPSSim(const std::string& name, const drone::model::components::GPSSensorSpecs& specs)
        : GPSModule_base(name, specs) {}

    void update() override {}
    void setAltitudeM(double altitude_m);

private:
    double altitude_m_{0.0};
};

}  // namespace drone::simulator::physics

#endif  // GPS_SIM_H
