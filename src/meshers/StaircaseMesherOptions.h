#pragma once

#include "MesherBaseOptions.h"

namespace meshlib::meshers {

enum class ThinSlotMode {
    None,
    Line,
    Surface
};

class StaircaseMesherOptions : public MesherBaseOptions {
public:
    bool compress = true;
    bool splitHexahedra = false;
    int decimalPlacesInCollapser = 4;
    // JSON: "thinSlot": "line" | "surface".
    ThinSlotMode thinSlotMode = ThinSlotMode::None;
    bool thinSlotAllowOnSurfacePerimeter = false;
    bool thinSlotIsExplicit = false;
};

}
