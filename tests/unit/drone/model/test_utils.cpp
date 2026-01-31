#include <catch2/catch_test_macros.hpp>
#include "drone/model/utils.h"

TEST_CASE("Clamp function works correctly [utils]", "[utils]") {
    REQUIRE(Utils::clamp(5, 0, 10) == 5);    // Within range
    REQUIRE(Utils::clamp(-5, 0, 10) == 0);   // Below min
    REQUIRE(Utils::clamp(15, 0, 10) == 10);  // Above max
    REQUIRE(Utils::clamp(0, 0, 10) == 0);    // At min
    REQUIRE(Utils::clamp(10, 0, 10) == 10);  // At max
}

TEST_CASE("MapRange function works correctly [utils]", "[utils]") {
    REQUIRE(Utils::mapRange(5, 0, 10, 0, 100) == 50);    // Midpoint
    REQUIRE(Utils::mapRange(0, 0, 10, 0, 100) == 0);     // Min input
    REQUIRE(Utils::mapRange(10, 0, 10, 0, 100) == 100);  // Max input
    REQUIRE(Utils::mapRange(2.5, 0, 10, 0, 100) == 25);  // Quarter point
    REQUIRE(Utils::mapRange(7.5, 0, 10, 0, 100) == 75);  // Three-quarter point

    // Test with outside rabge inputs
    REQUIRE(Utils::mapRange(-5, 0, 10, 0, 100) == -50);   // Below min input
    REQUIRE(Utils::mapRange(15, 0, 10, 0, 100) == 150);  // Above max input
   
}