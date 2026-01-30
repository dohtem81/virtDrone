#ifndef BASE_SENSOR_H
#define BASE_SENSOR_H

#include <string>
#include <memory>  // Added for std::unique_ptr

namespace drone::model::sensors {

/**
 * @brief Enumeration for sensor types.
 */
enum class SensorType {
    ACTUATOR,  ///< Represents an actuator sensor.
    SENSING    ///< Represents a sensing sensor.
};

/**
 * @brief Enumeration for sensor status.
 */
enum class SensorStatus {
    ACTIVE,   ///< Sensor is active.
    INACTIVE, ///< Sensor is inactive.
    ERROR     ///< Sensor has an error.
};

/**
 * @brief Class representing analog IO specifications for sensors.
 * 
 * This class encapsulates the direction, current/voltage range, and counts range
 * for analog inputs/outputs in drone sensors.
 */
class AnalogIOSpec {
public:
    /**
     * @brief Enumeration for IO direction.
     */
    enum class IODirection {
        INPUT,  ///< Input direction.
        OUTPUT  ///< Output direction.
    };

    /**
     * @brief Enumeration for current/voltage ranges.
     */
    enum class CurrentRange {
        ZERO_TO_20mA,   ///< 0-20mA range.
        FOUR_TO_20mA,   ///< 4-20mA range.
        PLUS_MINUS_10V, ///< ±10V range.
        ZERO_TO_10V     ///< 0-10V range.
    };

    /**
     * @brief Struct for counts range.
     */
    struct CountsRange {
        uint64_t min; ///< Minimum count value.
        uint64_t max; ///< Maximum count value.
    };

    /**
     * @brief Constructor for AnalogIOSpec.
     * @param dir The IO direction.
     * @param range The current/voltage range.
     * @param min_counts Minimum counts.
     * @param max_counts Maximum counts.
     */
    AnalogIOSpec(IODirection dir, CurrentRange range, uint64_t min_counts, uint64_t max_counts)
        : direction(dir), current_range(range), counts_range{min_counts, max_counts} {}
    
    /**
     * @brief Copy constructor for AnalogIOSpec.
     * @param spec The AnalogIOSpec to copy.
     */
    AnalogIOSpec(const AnalogIOSpec& spec)
        : direction(spec.direction), current_range(spec.current_range), counts_range(spec.counts_range) {}

    IODirection direction;       ///< The IO direction.
    CurrentRange current_range;  ///< The current/voltage range.
    CountsRange counts_range;    ///< The counts range.
};

/**
 * @brief Base class for drone sensors.
 * 
 * This abstract class provides common functionality for all sensors,
 * including name, status, and IO specifications. Derived classes must
 * implement the update() method.
 */
class BaseSensor {
public:
    /**
     * @brief Constructor for BaseSensor.
     * @param name The name of the sensor.
     * @param type The analog IO specification.
     */
    BaseSensor(const std::string& name, AnalogIOSpec type);
    
    /**
     * @brief Virtual destructor.
     */
    virtual ~BaseSensor() = default;

    // Getters
    /**
     * @brief Gets the sensor name.
     * @return The sensor name as a string.
     */
    std::string getName() const;
    
    /**
     * @brief Gets the sensor status.
     * @return A const reference to the sensor status.
     */
    const SensorStatus& getStatus() const;
    
    /**
     * @brief Gets the analog IO specification.
     * @return A const reference to the AnalogIOSpec.
     */
    const AnalogIOSpec& getType() const;

    /**
     * @brief Gets the last counts reading from the sensor.
     * @return A const reference to the last counts reading.
     */
    const uint64_t& getLastCountsReading() const { return last_reading_; }

    // Setters
    // Pure virtual method to be implemented by derived classes
    /**
     * @brief Updates the sensor state.
     * 
     * This method must be implemented by derived classes to perform
     * sensor-specific updates.
     */
    virtual void update() = 0;
    
    /**
     * @brief Sets the last counts reading from the sensor.
     * @param reading The counts reading to set.
     * @return True if the reading is within valid range, false otherwise.
     */
    bool setLastCountsReading(uint64_t reading);

protected:
    std::string name_;      ///< The sensor name.
    SensorStatus status_;   ///< The sensor status.
    std::unique_ptr<AnalogIOSpec> type_;  ///< Smart pointer to the analog IO specification.
    void setStatus(SensorStatus status);  ///< Sets the sensor status.
    uint64_t last_reading_; ///< The last reading from the sensor.
};

}  // namespace drone::model::sensors

#endif // BASE_SENSOR_H