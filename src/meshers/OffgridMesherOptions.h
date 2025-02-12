#pragma once

#include "core/SnapperOptions.h"

namespace meshlib::meshers {

class OffgridMesherOptions {
public:

    bool forceSlicing = true;
    bool collapseInternalPoints = true;
    bool snap = true;
    core::SnapperOptions snapperOptions;
    int decimalPlacesInCollapser = 4;
    std::set<GroupId> volumeGroups{};

};

}
