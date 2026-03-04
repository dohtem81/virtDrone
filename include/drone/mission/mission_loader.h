#ifndef DRONE_MISSION_MISSION_LOADER_H
#define DRONE_MISSION_MISSION_LOADER_H

#include <string>

#include "drone/mission/mission_types.h"

namespace drone::mission {

class MissionLoader {
public:
    bool loadFromFile(const std::string& file_path, Mission& mission_out, std::string* error_out = nullptr) const;
};

}  // namespace drone::mission

#endif  // DRONE_MISSION_MISSION_LOADER_H
