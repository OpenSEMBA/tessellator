#pragma once

#include "types/Mesh.h"
#include "core/SnapperOptions.h"

namespace meshlib::meshers {

class MesherBaseOptions {
public:
    bool isVolume = false;
    std::set<GroupId> volumeGroups{};
};

}
