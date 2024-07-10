#pragma once

#include "SmootherTools.h"

#include "types/Mesh.h"

namespace meshlib {
namespace tessellator {

struct SmootherOptions {
    double featureDetectionAngle = 30.0;
    double contourAlignmentAngle = 1.0;
};

class Smoother {
public:
    Smoother(const Mesh&, const SmootherOptions& opts = SmootherOptions());
    Mesh getMesh() const { return mesh_; }

private:
    SmootherOptions opts_;
    SmootherTools sT_;
    Mesh mesh_;
    
    Mesh orient(const Mesh& mesh) const;
    Group orientGroup(const Coordinates& coordinates, const Group& group) const;


};

}
}