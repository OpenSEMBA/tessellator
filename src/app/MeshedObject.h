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
    meshlib::meshers::EmbeddedMode embeddedMode =
        meshlib::meshers::EmbeddedMode::None;
    bool embeddedAllowOnSurfacePerimeter = false;
    bool embeddedIsExplicit = false;
    Mesh mesh;
};

}
}
