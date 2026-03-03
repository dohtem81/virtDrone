#ifndef GPS_SIM_H
#define GPS_SIM_H

#include "drone/model/components/gps_module_base.h"
#include "drone/drone_data_types.h"

namespace drone::simulator::physics {

class GPSSim final : public drone::model::components::GPSModule_base {
public:
    GPSSim(const std::string& name, const drone::model::components::GPSSensorSpecs& specs)
        : GPSModule_base(name, specs) {}

    void update() override {}
    void setAltitudeM(double altitude_m);
    void setReferenceGeodetic(const drone::Position3D& reference_position);
    void setPerfectEnuState(const drone::Vector3& position_enu_m, const drone::Vector3& velocity_enu_mps);

private:
    drone::Position3D reference_position_{};
};

}  // namespace drone::simulator::physics

#endif  // GPS_SIM_H
