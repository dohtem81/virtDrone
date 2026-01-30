#ifndef GPS_MODULE_H_
#define GPS_MODULE_H_

#include "drone/drone_data_types.h"  // For Position3D, Velocity3D
#include "drone/model/sensors/base_sensor.h"

using namespace drone;
using namespace drone::model::sensors;

namespace drone::model::components {

struct GPSSensorSpecs {
    double horizontal_accuracy_m;  ///< Horizontal position accuracy in meters.
    double vertical_accuracy_m;    ///< Vertical position accuracy in meters.
    double velocity_accuracy_mps;  ///< Velocity accuracy in meters per second.
    int update_rate_hz;            ///< Update rate in Hertz.
    int max_satellites;            ///< Maximum number of satellites the GPS can track.
    GPSSensorSpecs(double horiz_acc = 5.0, double vert_acc = 10.0, double vel_acc = 0.5, int upd_rate = 5, int max_sats = 8)
        : horizontal_accuracy_m(horiz_acc), vertical_accuracy_m(vert_acc), velocity_accuracy_mps(vel_acc),
          update_rate_hz(upd_rate), max_satellites(max_sats) {}
};

// we use that class to make testing easier - update method will be implemented in derived classes
// and will handle differences between various GPS module implementations and simulation
class GPSModule_base {
public:
    GPSModule_base(const std::string& name, const GPSSensorSpecs& specs)
        : name_(name), specs_(specs), sensor_status_(SensorStatus::INACTIVE) {}
    virtual ~GPSModule_base() = default;

    // Add GPS module specific methods and members here
    virtual void update() = 0;

    // setters must be implemented in derived classes to allow GPSPhysics to set position and velocity
    // this will allow to avoid letting setting position and velocity publicly in non-simulated GPS modules
    virtual void setPosition(const Position3D& pos) { position_ = pos; }
    virtual void setVelocity(const Velocity3D& vel) { velocity_ = vel; }
    virtual void setStatus(SensorStatus status) { sensor_status_ = status; }
    virtual void setSatelliteCount(int count) {  satellite_count_ = count; }
    // getters
    SensorStatus getStatus() const { return sensor_status_; }
    int getSatelliteCount() const { return satellite_count_; }
    Position3D getPosition() const { return position_; }
    Velocity3D getVelocity() const { return velocity_; }
    std::string getName() const { return name_; }
    GPSSensorSpecs getSpecs() const { return specs_; }  
private:
    Position3D position_;
    Velocity3D velocity_;
    std::string name_;
    GPSSensorSpecs specs_;
    SensorStatus sensor_status_;
    int satellite_count_;
};
}  // namespace drone::model::components


#endif  // GPS_MODULE_H_