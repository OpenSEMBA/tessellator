#pragma once

#include "types/Mesh.h"
#include "core/SnapperOptions.h"

namespace meshlib::meshers {

class ConformalMesherOptions {
public:
    core::SnapperOptions snapperOptions;
    std::set<GroupId> volumeGroups{};
};

}
