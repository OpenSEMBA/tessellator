#pragma once

#include "types/Mesh.h"
#include "utils/GridTools.h"

namespace meshlib::core {

class VolumeFiller : private utils::GridTools {
public:
    explicit VolumeFiller(const Mesh& staircasedSurface);

    Mesh getMesh() const;

private:
    Mesh mesh_;
};

}
