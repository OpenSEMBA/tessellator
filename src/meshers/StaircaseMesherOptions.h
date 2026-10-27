#pragma once

#include "MesherBaseOptions.h"

namespace meshlib::meshers {

enum class EmbeddedMode {
    None,
    Line,
    Surface
};

class StaircaseMesherOptions : public MesherBaseOptions {
public:
    bool compress = true;
    bool splitHexahedra = false;
    int decimalPlacesInCollapser = 4;
    // JSON: "embedded": "line" | "surface".
    EmbeddedMode embeddedMode = EmbeddedMode::None;
    bool embeddedAllowOnSurfacePerimeter = false;
    bool embeddedIsExplicit = false;
};

}
