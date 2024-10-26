#pragma once

#include "SnapperOptions.h"

namespace meshlib::drivers {

class OffgridDriverOptions {
public:

    bool forceSlicing = true;
    bool collapseInternalPoints = true;
    bool snap = true;
    SnapperOptions snapperOptions;
    int decimalPlacesInCollapser = 4;
    std::set<GroupId> volumeGroups{};

};

}
