#include "Collapser.h"

#include "utils/Geometry.h"
#include "utils/Cleaner.h"
#include "utils/MeshTools.h"

#include "Collapser.h"


namespace meshlib {
namespace tessellator {

using namespace utils;

Collapser::Collapser(const Mesh& in, int decimalPlaces)
{
    mesh_ = in;
    double factor = std::pow(10.0, decimalPlaces);
    for (auto& v : mesh_.coordinates) {
        v = v.round(factor);
    }
    
    Cleaner::fuseCoords(mesh_);
    Cleaner::cleanCoords(mesh_);
    
    Cleaner::collapseCoordsInLineDegenerateTriangles(mesh_, 0.4 / (factor * factor));
    Cleaner::removeRepeatedElements(mesh_);
    utils::meshTools::checkNoNullAreasExist(mesh_);
}


}
}