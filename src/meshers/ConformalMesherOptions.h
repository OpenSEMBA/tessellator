#pragma once

#include "types/Mesh.h"
#include "core/SnapperOptions.h"

namespace meshlib::meshers {

class ConformalMesherOptions {
public:
    core::SnapperOptions snapperOptions;
    int decimalPlacesInCollapser = 2;
    std::set<GroupId> volumeGroups{};
};

}
