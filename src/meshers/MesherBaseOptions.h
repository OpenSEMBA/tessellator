#pragma once

#include "types/Mesh.h"
#include "core/SnapperOptions.h"

namespace meshlib::meshers {

class MesherBaseOptions {
public:
    bool allow_log=false;
    std::set<GroupId> volumeGroups{};
};

}
