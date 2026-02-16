#include <catch2/catch_test_macros.hpp>
#include "drone/model/components/gps_module_base.h"

using namespace drone::model::components;

// Assuming these are defined elsewhere or add them here if needed
GPSSensorSpecs gps_specs(5.0, 10.0, 0.5, 5, 8, 0.05);

class GPSensor_test : public GPSModule_base {
public:
    GPSensor_test(const std::string& name, const GPSSensorSpecs& specs)
        : GPSModule_base(name, specs) {}

    // Expose private setters for testing
    using GPSModule_base::setPosition;
    using GPSModule_base::setVelocity;
    using GPSModule_base::setSatelliteCount;
    using GPSModule_base::setStatus;

    void update() override {
        // Dummy implementation for testing
    }  
};

// Test cases for GPSSensor class
TEST_CASE("GPSSensor initializes correctly", "[GPSSensor]") {
    GPSensor_test gps("TestGPS", gps_specs);

    REQUIRE(gps.getName() == "TestGPS");
    REQUIRE(gps.getStatus() == SensorStatus::INACTIVE);  // Assuming default status
    REQUIRE(gps.getSpecs().horizontal_accuracy_m == 5.0);
    REQUIRE(gps.getSpecs().update_rate_hz == 5);
    REQUIRE(gps.getSpecs().max_satellites == 8);
}

// New test case for set/get functionality using the test class
TEST_CASE("GPSSensor set/get operations work correctly", "[GPSSensor]") {
    GPSensor_test gps_test("TestGPS", gps_specs);

    // Test set/get position
    Position3D new_pos{45.0, -122.0, 100.0};  // Example: lat, lon, alt
    gps_test.setPosition(new_pos);
    REQUIRE(gps_test.getPosition().latitude_deg == 45.0);
    REQUIRE(gps_test.getPosition().longitude_deg == -122.0);
    REQUIRE(gps_test.getPosition().altitude_m == 100.0);

    // Test set/get velocity
    Velocity3D new_vel{10.0, 5.0, -2.0};  // Example: north, east, down
    gps_test.setVelocity(new_vel);
    REQUIRE(gps_test.getVelocity().north_mps == 10.0);
    REQUIRE(gps_test.getVelocity().east_mps == 5.0);
    REQUIRE(gps_test.getVelocity().down_mps == -2.0);

    // Test set/get satellite count
    gps_test.setSatelliteCount(6);
    REQUIRE(gps_test.getSatelliteCount() == 6);

    // Test set/get status
    gps_test.setStatus(SensorStatus::ACTIVE);
    REQUIRE(gps_test.getStatus() == SensorStatus::ACTIVE);
}
