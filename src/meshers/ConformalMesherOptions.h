#pragma once

#include "types/Mesh.h"
#include "MesherBaseOptions.h"
#include "core/SnapperOptions.h"

namespace meshlib::meshers {

class ConformalMesherOptions : public MesherBaseOptions {
public:
    core::SnapperOptions snapperOptions;
    bool staircaseSharedCells = true;
    // std::set<GroupId> volumeGroups{};
};

}
