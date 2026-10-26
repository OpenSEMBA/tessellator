#pragma once

#include <string>

#include "launcher.h"
#include "meshers/StaircaseMesherOptions.h"
#include "types/Mesh.h"

namespace meshlib {
namespace app {

struct MeshedObject {
    ObjectDefinition definition;
    std::string mesherType;
    std::string extension;
    bool compress = false;
    meshlib::meshers::ThinSlotMode thinSlotMode =
        meshlib::meshers::ThinSlotMode::None;
    bool thinSlotAllowOnSurfacePerimeter = false;
    bool thinSlotIsExplicit = false;
    Mesh mesh;
};

}
}
