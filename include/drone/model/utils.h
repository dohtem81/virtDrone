#ifndef UTILS_H
#define UTILS_H

#include <type_traits>  // For std::common_type

class Utils {
public:
    // Utility functions can be added here
    template <typename T>
    static T clamp(const T& value, const T& min, const T& max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    template <typename T, typename U, typename V, typename W, typename X>
    static auto mapRange(T value, U in_min, V in_max, W out_min, X out_max) {
        using Common = std::common_type_t<T, U, V, W, X>;
        Common val = static_cast<Common>(value);
        Common i_min = static_cast<Common>(in_min);
        Common i_max = static_cast<Common>(in_max);
        Common o_min = static_cast<Common>(out_min);
        Common o_max = static_cast<Common>(out_max);
        if (i_max == i_min) {
            return o_min;  // Avoid division by zero
        }
        return (val - i_min) * (o_max - o_min) / (i_max - i_min) + o_min;
    }
};

#endif // UTILS_H