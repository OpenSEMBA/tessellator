#pragma once

#include "SnapperOptions.h"

namespace meshlib {
namespace tessellator {

class DriverOptions {
public:

    bool forceSlicing = true;
    bool collapseInternalPoints = true;
    bool snap = true;
    SnapperOptions snapperOptions;
    int decimalPlacesInCollapser = 4;
    std::set<GroupId> volumeGroups{};

};

}
}
