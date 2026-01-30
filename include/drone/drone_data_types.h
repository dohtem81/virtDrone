#ifndef DRONE_DATA_TYPES_H
#define DRONE_DATA_TYPES_H

#include <cmath>  // For std::abs, etc., if needed

namespace drone {

// Structure for 3D position (geospatial: lat/lon/alt in degrees/meters)
struct Position3D {
    double latitude_deg;   // Latitude in degrees (-90 to 90)
    double longitude_deg;  // Longitude in degrees (-180 to 180)
    double altitude_m;     // Altitude in meters (above sea level or ground)

    // Default constructor
    Position3D() : latitude_deg(0.0), longitude_deg(0.0), altitude_m(0.0) {}

    // Parameterized constructor
    Position3D(double lat, double lon, double alt)
        : latitude_deg(lat), longitude_deg(lon), altitude_m(alt) {}

    // Equality operator
    bool operator==(const Position3D& other) const {
        return latitude_deg == other.latitude_deg &&
               longitude_deg == other.longitude_deg &&
               altitude_m == other.altitude_m;
    }

    // Addition operator (for offsets, e.g., simulation updates)
    Position3D operator+(const Position3D& other) const {
        return Position3D(latitude_deg + other.latitude_deg,
                          longitude_deg + other.longitude_deg,
                          altitude_m + other.altitude_m);
    }

    // Subtraction operator
    Position3D operator-(const Position3D& other) const {
        return Position3D(latitude_deg - other.latitude_deg,
                          longitude_deg - other.longitude_deg,
                          altitude_m - other.altitude_m);
    }

    // Distance calculation (simple Euclidean, for Cartesian; use Haversine for geo)
    double distanceTo(const Position3D& other) const {
        double dlat = latitude_deg - other.latitude_deg;
        double dlon = longitude_deg - other.longitude_deg;
        double dalt = altitude_m - other.altitude_m;
        return std::sqrt(dlat * dlat + dlon * dlon + dalt * dalt);
    }
};

// Placeholder for Velocity3D (if not defined elsewhere)
struct Velocity3D {
    double north_mps;  // Velocity north in m/s
    double east_mps;   // Velocity east in m/s
    double down_mps;   // Velocity down in m/s (negative for up)

    Velocity3D() : north_mps(0.0), east_mps(0.0), down_mps(0.0) {}
    Velocity3D(double n, double e, double d) : north_mps(n), east_mps(e), down_mps(d) {}
};

}  // namespace drone

#endif  // DRONE_DATA_TYPES_H