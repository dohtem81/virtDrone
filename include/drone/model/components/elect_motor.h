#ifndef ELEC_MOTOR_H
#define ELEC_MOTOR_H

#include "drone/model/sensors/temperature_sensor.h"
#include <memory>
#include <chrono>    // For time calculations

namespace drone::model::components {

/**
 * @brief Struct for electrical motor specifications.
 */
struct ElecMotorSpecs {
    double max_speed_rpm;     ///< Maximum speed in RPM.
    double nominal_voltage_v; ///< Nominal voltage in volts.
    double max_current_a;     ///< Maximum current in amperes.
    double efficiency;        ///< Efficiency as a fraction (0.0 to 1.0).
    double thermal_resistance; ///< Thermal resistance in °C/W for temperature calculation.

    ElecMotorSpecs(double max_speed = 10000.0, double nom_vol = 12.0, double max_curr = 10.0, double eff = 0.85, double therm_res = 0.5)
        : max_speed_rpm(max_speed), nominal_voltage_v(nom_vol), max_current_a(max_curr), efficiency(eff), thermal_resistance(therm_res) {}
};

/**
 * @brief Class simulating an electrical motor, inheriting from BaseSensor.
 * 
 * This class models motor behavior including speed, current consumption, efficiency,
 * losses, temperature, and battery drain calculations.
 */
class ElecMotor : public drone::model::sensors::BaseSensor {
public:
    /**
     * @brief Constructor for ElecMotor.
     * @param name The name of the motor.
     * @param spec The analog IO specification.
     * @param motor_specs The motor specifications.
     */
    ElecMotor(const std::string& name, const drone::model::sensors::AnalogIOSpec& spec, const ElecMotorSpecs& motor_specs = ElecMotorSpecs());
    
    /**
     * @brief Virtual destructor.
     */
    virtual ~ElecMotor() = default;

    /**
     * @brief Updates the motor state (simulates motor operation).
     * 
     * This method simulates speed, current, losses, temperature, and battery drain.
     */
    void update() override;

    // Motor-specific getters
    /**
     * @brief Gets the current speed in RPM.
     * @return The speed in RPM.
     */
    double getSpeedRPM() const { return speed_rpm_; }
    
    /**
     * @brief Gets the current consumption in amperes.
     * @return The current in A.
     */
    double getCurrentA() const { return current_a_; }
    
    /**
     * @brief Gets the motor temperature in Celsius.
     * @return The temperature in °C.
     */
    double getTemperatureC() const { return temperature_c_; }

    drone::model::sensors::TemperatureSensorReading getTemperatureReading() const { 
        drone::model::sensors::TemperatureSensorReading temp_reading = {
            temp_sensor_.getTemperature(),
            temp_sensor_.getUnits(),
            temp_sensor_.getStatus()
        };
        return temp_reading; 
    }
    
    /**
     * @brief Gets the motor efficiency.
     * @return The efficiency as a fraction.
     */
    double getEfficiency() const { return specs_.efficiency; }
    
    /**
     * @brief Calculates battery drain (energy consumed in Joules) over a given time.
     * @param time_s Time in seconds.
     * @return Energy consumed in Joules.
     */
    double calculateBatteryDrain(double time_s) const;
    
    /**
     * @brief Gets the motor specifications.
     * @return A const reference to the ElecMotorSpecs.
     */
    const ElecMotorSpecs& getSpecs() const { return specs_; }

    // Additional getters for simulation state
    double getDesiredSpeedRPM() const { return desired_speed_rpm_; }
    double getVoltageV() const { return voltage_v_; }
    double getLossesW() const { return losses_w_; }
    double getAmbientTempC() const { return ambient_temp_c_; }
    std::chrono::steady_clock::time_point getLastUpdateTime() const { return last_update_time_; }
    drone::model::sensors::TemperatureSensor& getTempSensor() { return temp_sensor_; }

    // Setters for simulation state
    void setSpeedRPM(double speed) { speed_rpm_ = speed; }
    void setCurrentA(double current) { current_a_ = current; }
    void setTemperatureC(double temp) { temperature_c_ = temp; }
    void setLossesW(double losses) { losses_w_ = losses; }
    void setVoltageV(double voltage) { voltage_v_ = voltage; }
    void setAmbientTempC(double temp) { ambient_temp_c_ = temp; }
    void setLastUpdateTime(std::chrono::steady_clock::time_point time) { last_update_time_ = time; }

    // Setters for simulation inputs
    /**
     * @brief Sets the desired speed (input for simulation).
     * @param speed_rpm Desired speed in RPM.
     */
    void setDesiredSpeedRPM(double speed_rpm);

private:
    ElecMotorSpecs specs_;     ///< Motor specifications.
    double speed_rpm_;         ///< Current speed in RPM.
    double desired_speed_rpm_; ///< Desired speed in RPM (input).
    double current_a_;         ///< Current consumption in A.
    double voltage_v_;         ///< Operating voltage in V.
    double temperature_c_;     ///< Motor temperature in °C.
    double losses_w_;          ///< Power losses in W.
    double ambient_temp_c_;    ///< Ambient temperature in °C (assumed constant).
    std::chrono::steady_clock::time_point last_update_time_; ///< Time point of the last update.
    drone::model::sensors::TemperatureSensor temp_sensor_; ///< Internal temperature sensor for motor.
};

}  // namespace drone::model::components

#endif // ELEC_MOTOR_H