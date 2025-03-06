#include "Collapser.h"

#include "utils/Geometry.h"
#include "utils/RedundancyCleaner.h"
#include "utils/MeshTools.h"

#include "Collapser.h"

namespace meshlib {
namespace core {

using namespace utils;

Collapser::Collapser(const Mesh& in, int decimalPlaces)
{    
    mesh_ = in;
    double factor = std::pow(10.0, decimalPlaces);
    for (auto& v : mesh_.coordinates) {
        v = v.round(factor);
    }
    
    RedundancyCleaner::fuseCoords(mesh_);
    RedundancyCleaner::cleanCoords(mesh_);
    
    RedundancyCleaner::collapseCoordsInLineDegenerateTriangles(mesh_, 0.4 / (factor * factor));
    RedundancyCleaner::removeRepeatedElements(mesh_);
    utils::meshTools::checkNoNullAreasExist(mesh_);
}


}
}