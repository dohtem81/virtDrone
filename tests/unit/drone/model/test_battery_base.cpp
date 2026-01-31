#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp> // Add this line
#include "drone/model/components/battery_base.h"
#include "drone/model/components/battery_cell.h"
#include <memory>

using namespace drone::model::components;
using Catch::Approx;

// Assuming Battery_Cell is defined elsewhere; for testing, use a simple mock or assume defaults
class Battery_test : public Battery_base {
public:
    Battery_test(const std::string& name, const BatterySpecs& specs)
        : Battery_base(name, specs) {}

    using Battery_base::update;
    using Battery_base::setCurrentA;
};


// Derived class to expose protected members for testing
class Cell_test : public Battery_Cell {
public:
    Cell_test(const std::string& cell_id, double capacity_mah, double nominal_voltage_v)
        : Battery_Cell(cell_id, capacity_mah, nominal_voltage_v) {}
    
    Cell_test(const std::string& cell_id, const CellSpecs& specs)
        : Battery_Cell(cell_id, specs) {}

    using Battery_Cell::update;
    using Battery_Cell::setCurrentA;
    using Battery_Cell::setStateOfChargePercent;
};

std::unique_ptr<Cell_test> testCellSpecs = std::make_unique<Cell_test>("TestCell", 1500.0, 4.2);
// Create CellSpecs from Cell_test
auto cellSpecs = std::make_shared<CellSpecs>(
    testCellSpecs->getCapacityMah(),
    testCellSpecs->getNominalVoltageV()
);

// test cell contructor
TEST_CASE("Battery_Cell initializes correctly", "[Battery_Cell]") {
    Cell_test cell("Cell_1", 1500.0, 3.7);

    REQUIRE(cell.getCellID() == "Cell_1");
    REQUIRE(cell.getCapacityMah() == 1500.0);
    REQUIRE(cell.getNominalVoltageV() == 3.7);
    REQUIRE(cell.getCurrentA() == 0.0); // Placeholder value
    REQUIRE(cell.getRemainingCapacityMah() == 1500.0);
    REQUIRE(cell.getStateOfChargePercent() == 100.0); // Placeholder value
}


// Test battery capacity decrease over time
TEST_CASE("Battery_Cell updates state of charge correctly", "[Battery_Cell]") {
    Cell_test cell("Cell_1", 1500.0, 3.7);
    REQUIRE(cell.getRemainingCapacityMah() == 1500.0);
    REQUIRE(cell.getStateOfChargePercent() == 100.0); // Placeholder value
    
    // Simulate a discharge current
    cell.setCurrentA(1.5); // Assuming a setter exists; if not, modify the class accordingly
    REQUIRE(cell.getCurrentA() == 1.5);
    cell.update(3600000); // 1 hour in milliseconds
    REQUIRE(cell.getStateOfChargePercent() == 0.0); // Should have decreased
    REQUIRE(cell.getRemainingCapacityMah() == 0.0); // Should have decreased
}

// Test volatge drop calculation at levels 100, 85, 50, 30, 0
TEST_CASE("Battery_Cell calculates voltage drop correctly", "[Battery_Cell]") {
    Cell_test cell("Cell_1", 1500.0, 4.2);

    cell.setStateOfChargePercent(100.0);
    REQUIRE(cell.getStateOfChargePercent() == 100.0);
    REQUIRE(cell.getVoltageV() == Approx(4.2));

    cell.setStateOfChargePercent(85.0);
    REQUIRE(cell.getStateOfChargePercent() == 85.0);
    REQUIRE(cell.getVoltageV() == Approx(3.9).margin(0.1));

    cell.setStateOfChargePercent(50.0);
    REQUIRE(cell.getStateOfChargePercent() == 50.0);
    REQUIRE(cell.getVoltageV() == Approx(3.7).margin(0.1));

    cell.setStateOfChargePercent(30.0);
    REQUIRE(cell.getStateOfChargePercent() == 30.0);
    REQUIRE(cell.getVoltageV() == Approx(3.5).margin(0.1));

    cell.setStateOfChargePercent(0.0);
    REQUIRE(cell.getStateOfChargePercent() == 0.0);
    REQUIRE(cell.getVoltageV() == Approx(3.2).margin(0.1));
}

TEST_CASE("Battery_Cell initializes correctly with CellSpecs, 1 cell", "[Battery_Cell]") {
    Battery_test battery("TestBattery", BatterySpecs(1, *cellSpecs));

    REQUIRE(battery.getName() == "TestBattery");
    REQUIRE(battery.getVoltageV() == Approx(4.2));
    REQUIRE(battery.getRemainingCapacityMah() == Approx(1500.0));
}

TEST_CASE("Battery_Cell initializes correctly with CellSpecs, 4 cell", "[Battery_Cell]") {
    Battery_test battery("TestBattery", BatterySpecs(4, *cellSpecs));

    REQUIRE(battery.getName() == "TestBattery");
    REQUIRE(battery.getVoltageV() == Approx(16.8));
    REQUIRE(battery.getRemainingCapacityMah() == Approx(1500.0));
}

//battery test - capacity decrease over time
TEST_CASE("Battery_base updates cells correctly, 1 cell", "[Battery_base]") {
    Battery_test battery("TestBattery", BatterySpecs(1, *cellSpecs));
    REQUIRE(battery.getName() == "TestBattery");
    REQUIRE(battery.getVoltageV() == Approx(4.2));
    REQUIRE(battery.getRemainingCapacityMah() == Approx(1500.0));
    battery.setCurrentA(1.5); // Assuming a setter exists; if not, modify the class accordingly
    REQUIRE(battery.getCurrentA() == 1.5);

    battery.update(3600000); // 1 hour in milliseconds
    REQUIRE(battery.getRemainingCapacityMah() == Approx(0.0)); // Should have decreased
}

// Test volatge drop calculation at levels 100, 85, 50, 30, 0
TEST_CASE("Battery_Cell calculates voltage drop correctly, 1 cell", "[Battery_Cell]") {
    Battery_test battery("TestBattery", BatterySpecs(1, *cellSpecs));

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
    Battery_test battery("TestBattery", BatterySpecs(4, *cellSpecs));

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