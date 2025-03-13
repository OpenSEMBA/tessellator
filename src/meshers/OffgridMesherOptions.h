#pragma once

#include "core/SnapperOptions.h"
#include "types/Mesh.h"

namespace meshlib::meshers {

class OffgridMesherOptions {
    public:
        bool forceSlicing = true;
        bool smooth = true;
        bool snap = true;
        core::SnapperOptions snapperOptions;
        int decimalPlacesInCollapser = 4;
        std::set<GroupId> volumeGroups{};
    
    };
}