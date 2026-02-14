#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp> // Add this line
#include "drone/model/components/battery_base.h"
#include "drone/model/components/battery_cell.h"
#include "simulator/physics/battery_cell_physics.h"
#include "simulator/physics/battery_sim.h"
#include <memory>

using namespace drone::model::components;
using namespace drone::simulator::physics;
using Catch::Approx;

std::unique_ptr<Battery_Cell> testCellSpecs = std::make_unique<Battery_Cell>("TestCell", 1500.0, 4.2);
// Create CellSpecs from Cell_test
auto cellSpecs = std::make_shared<CellSpecs>(
    testCellSpecs->getCapacityMah(),
    testCellSpecs->getNominalVoltageV()
);

// test cell contructor
TEST_CASE("Battery_Cell initializes correctly", "[Battery_Cell]") {
    Battery_Cell cell("Cell_1", 1500.0, 3.7);

    REQUIRE(cell.getCellID() == "Cell_1");
    REQUIRE(cell.getCapacityMah() == 1500.0);
    REQUIRE(cell.getNominalVoltageV() == 3.7);
    REQUIRE(cell.getCurrentA() == 0.0); // Placeholder value
    REQUIRE(cell.getRemainingCapacityMah() == 1500.0);
    REQUIRE(cell.getStateOfChargePercent() == 100.0); // Placeholder value
}


// Test battery capacity decrease over time
TEST_CASE("Battery_Cell updates state of charge correctly", "[Battery_Cell]") {
    Battery_Cell cell("Cell_1", 1500.0, 3.7);
    REQUIRE(cell.getRemainingCapacityMah() == 1500.0);
    REQUIRE(cell.getStateOfChargePercent() == 100.0); // Placeholder value
    
    // Simulate a discharge current
    BatteryCellPhysics::setCurrentA(cell, 1.5);
    REQUIRE(cell.getCurrentA() == 1.5);
    BatteryCellPhysics::update(cell, 3600000); // 1 hour in milliseconds
    REQUIRE(cell.getStateOfChargePercent() == 0.0); // Should have decreased
    REQUIRE(cell.getRemainingCapacityMah() == 0.0); // Should have decreased
}

// Test volatge drop calculation at levels 100, 85, 50, 30, 0
TEST_CASE("Battery_Cell calculates voltage drop correctly", "[Battery_Cell]") {
    Battery_Cell cell("Cell_1", 1500.0, 4.2);

    BatteryCellPhysics::setStateOfChargePercent(cell, 100.0);
    REQUIRE(cell.getStateOfChargePercent() == 100.0);
    REQUIRE(cell.getVoltageV() == Approx(4.2));

    BatteryCellPhysics::setStateOfChargePercent(cell, 85.0);
    REQUIRE(cell.getStateOfChargePercent() == 85.0);
    REQUIRE(cell.getVoltageV() == Approx(3.9).margin(0.1));

    BatteryCellPhysics::setStateOfChargePercent(cell, 50.0);
    REQUIRE(cell.getStateOfChargePercent() == 50.0);
    REQUIRE(cell.getVoltageV() == Approx(3.7).margin(0.1));

    BatteryCellPhysics::setStateOfChargePercent(cell, 30.0);
    REQUIRE(cell.getStateOfChargePercent() == 30.0);
    REQUIRE(cell.getVoltageV() == Approx(3.5).margin(0.1));

    BatteryCellPhysics::setStateOfChargePercent(cell, 0.0);
    REQUIRE(cell.getStateOfChargePercent() == 0.0);
    REQUIRE(cell.getVoltageV() == Approx(3.2).margin(0.1));
}

TEST_CASE("Battery_Cell initializes correctly with CellSpecs, 1 cell", "[Battery_Cell]") {
    BatterySim battery("TestBattery", BatterySpecs(1, *cellSpecs));

    REQUIRE(battery.getName() == "TestBattery");
    REQUIRE(battery.getVoltageV() == Approx(4.2));
    REQUIRE(battery.getRemainingCapacityMah() == Approx(1500.0));
}

TEST_CASE("Battery_Cell initializes correctly with CellSpecs, 4 cell", "[Battery_Cell]") {
    BatterySim battery("TestBattery", BatterySpecs(4, *cellSpecs));

    REQUIRE(battery.getName() == "TestBattery");
    REQUIRE(battery.getVoltageV() == Approx(16.8));
    REQUIRE(battery.getRemainingCapacityMah() == Approx(1500.0));
}

//battery test - capacity decrease over time
TEST_CASE("Battery_base updates cells correctly, 1 cell", "[Battery_base]") {
    BatterySim battery("TestBattery", BatterySpecs(1, *cellSpecs));
    REQUIRE(battery.getName() == "TestBattery");
    REQUIRE(battery.getVoltageV() == Approx(4.2));
    REQUIRE(battery.getRemainingCapacityMah() == Approx(1500.0));
    battery.setCurrentA(1.5);
    REQUIRE(battery.getCurrentA() == 1.5);

    battery.update(3600000); // 1 hour in milliseconds
    REQUIRE(battery.getRemainingCapacityMah() == Approx(0.0)); // Should have decreased
}

// Test volatge drop calculation at levels 100, 85, 50, 30, 0
TEST_CASE("Battery_Cell calculates voltage drop correctly, 1 cell", "[Battery_Cell]") {
    BatterySim battery("TestBattery", BatterySpecs(1, *cellSpecs));

    battery.setStateOfChargePercent(100.0);
    REQUIRE(battery.getStateOfChargePercent() == 100.0);
    REQUIRE(battery.getVoltageV() == Approx(4.2));

    battery.setStateOfChargePercent(85.0);
    REQUIRE(battery.getStateOfChargePercent() == 85.0);
    REQUIRE(battery.getVoltageV() == Approx(3.9).margin(0.1));

    battery.setStateOfChargePercent(50.0);
    REQUIRE(battery.getStateOfChargePercent() == 50.0);
    REQUIRE(battery.getVoltageV() == Approx(3.7).margin(0.1));

    battery.setStateOfChargePercent(30.0);
    REQUIRE(battery.getStateOfChargePercent() == 30.0);
    REQUIRE(battery.getVoltageV() == Approx(3.5).margin(0.1));

    battery.setStateOfChargePercent(0.0);
    REQUIRE(battery.getStateOfChargePercent() == 0.0);
    REQUIRE(battery.getVoltageV() == Approx(3.2).margin(0.1));
}

// Test volatge drop calculation at levels 100, 85, 50, 30, 0
TEST_CASE("Battery_Cell calculates voltage drop correctly, 4 cells", "[Battery_Cell]") {
    BatterySim battery("TestBattery", BatterySpecs(4, *cellSpecs));

    battery.setStateOfChargePercent(100.0);
    REQUIRE(battery.getStateOfChargePercent() == 100.0);
    REQUIRE(battery.getVoltageV() == Approx(16.8));

    battery.setStateOfChargePercent(85.0);
    REQUIRE(battery.getStateOfChargePercent() == 85.0);
    REQUIRE(battery.getVoltageV() == Approx(15.6).margin(0.1));

    battery.setStateOfChargePercent(50.0);
    REQUIRE(battery.getStateOfChargePercent() == 50.0);
    REQUIRE(battery.getVoltageV() == Approx(14.6).margin(0.1));

    battery.setStateOfChargePercent(30.0);
    REQUIRE(battery.getStateOfChargePercent() == 30.0);
    REQUIRE(battery.getVoltageV() == Approx(14.0).margin(0.1));

    battery.setStateOfChargePercent(0.0);
    REQUIRE(battery.getStateOfChargePercent() == 0.0);
    REQUIRE(battery.getVoltageV() == Approx(12.8).margin(0.1));
}