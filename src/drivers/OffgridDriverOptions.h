#pragma once

#include "core/SnapperOptions.h"

namespace meshlib::drivers {

class OffgridDriverOptions {
public:

    bool forceSlicing = true;
    bool collapseInternalPoints = true;
    bool snap = true;
    core::SnapperOptions snapperOptions;
    int decimalPlacesInCollapser = 4;
    std::set<GroupId> volumeGroups{};

};

}
