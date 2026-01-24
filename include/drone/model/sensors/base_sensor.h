#ifndef BASE_SENSOR_H
#define BASE_SENSOR_H

#include <string>

enum class SensorType {
    ACTUATOR,
    SENSING
};

enum class SensorStatus {
    ACTIVE,
    INACTIVE,
    ERROR
};

class BaseSensor {
public:
    BaseSensor(const std::string& name, SensorType type);
    virtual ~BaseSensor() = default;

    // Getters
    std::string getName() const;
    SensorStatus getStatus() const;
    SensorType getType() const;

    // Setters
    void setStatus(SensorStatus status);

    // Pure virtual method to be implemented by derived classes
    virtual void update() = 0;

protected:
    std::string name_;
    SensorStatus status_;
    SensorType type_;
};

#endif // BASE_SENSOR_H