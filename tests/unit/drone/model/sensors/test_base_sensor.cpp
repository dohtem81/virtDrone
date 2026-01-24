#include <gtest/gtest.h>
#include "drone/model/sensors/base_sensor.h"

// Test subclass to implement the pure virtual method
class TestSensor : public BaseSensor {
public:
    TestSensor(const std::string& name, SensorType type)
        : BaseSensor(name, type) {}

    void update() override {
        // Dummy implementation for testing
    }
};

class BaseSensorTest : public ::testing::Test {
protected:
    TestSensor sensor{"TestSensor", SensorType::SENSING};
};

// Test constructor
TEST_F(BaseSensorTest, ConstructorSetsNameAndType) {
    EXPECT_EQ(sensor.getName(), "TestSensor");
    EXPECT_EQ(sensor.getType(), SensorType::SENSING);
    EXPECT_EQ(sensor.getStatus(), SensorStatus::INACTIVE);  // Default status
}

// Test getters
TEST_F(BaseSensorTest, GettersReturnCorrectValues) {
    EXPECT_EQ(sensor.getName(), "TestSensor");
    EXPECT_EQ(sensor.getType(), SensorType::SENSING);
    EXPECT_EQ(sensor.getStatus(), SensorStatus::INACTIVE);
}

// Test setter
TEST_F(BaseSensorTest, SetStatusUpdatesStatus) {
    sensor.setStatus(SensorStatus::ACTIVE);
    EXPECT_EQ(sensor.getStatus(), SensorStatus::ACTIVE);

    sensor.setStatus(SensorStatus::ERROR);
    EXPECT_EQ(sensor.getStatus(), SensorStatus::ERROR);
}

// Test actuator type
TEST_F(BaseSensorTest, ActuatorType) {
    TestSensor actuatorSensor("ActuatorSensor", SensorType::ACTUATOR);
    EXPECT_EQ(actuatorSensor.getType(), SensorType::ACTUATOR);
}