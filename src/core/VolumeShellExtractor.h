#pragma once

#include "types/Mesh.h"

namespace meshlib::core {

class VolumeShellExtractor {
public:
    explicit VolumeShellExtractor(const Mesh& volumeMesh);

    Mesh getMesh() const;

private:
    Mesh mesh_;
};

}
