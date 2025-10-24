#include "Collapser.h"

#include "utils/Geometry.h"
#include "utils/RedundancyCleaner.h"
#include "utils/MeshTools.h"

#include "Collapser.h"

namespace meshlib {
namespace core {

using namespace utils;

Collapser::Collapser(const Mesh& in, int decimalPlaces, const std::vector<Element::Type>& dimensionPolicy)
{
    if (dimensionPolicy.size() == 0) {
        dimensionPolicy_ = std::vector<Element::Type>(in.groups.size(), Element::Type::Surface);
    }
    else {
        dimensionPolicy_ = dimensionPolicy;
    }

    mesh_ = in;
    double factor = std::pow(10.0, decimalPlaces);
    for (auto& v : mesh_.coordinates) {
        v = v.round(factor);
    }
    
    RedundancyCleaner::fuseCoords(mesh_);
    RedundancyCleaner::cleanCoords(mesh_);
    
    collapseDegenerateElements(mesh_, 0.4 / (factor * factor));
    RedundancyCleaner::removeOverlappedElementsByDimension(mesh_, dimensionPolicy_);
    utils::meshTools::checkNoNullAreasExist(mesh_);
}



void Collapser::collapseDegenerateElements(Mesh& mesh, const double& areaThreshold) 
{
    const std::size_t MAX_NUMBER_OF_ITERATION = 1000;
    bool degeneratedTrianglesFound = true;
    for (std::size_t iter = 0; 
        iter < MAX_NUMBER_OF_ITERATION && degeneratedTrianglesFound; 
        ++iter) 
    {
        degeneratedTrianglesFound = false;
        for (auto& group : mesh.groups) {
            for (auto& element : group.elements) {
                if (!element.isTriangle() ||
                    !Geometry::isDegenerate(Geometry::asTriV(element, mesh.coordinates), areaThreshold)) {
                    continue;
                }
                degeneratedTrianglesFound = true;
                Coordinates& coords = mesh.coordinates;
                const std::vector<CoordinateId>& v = element.vertices;
                std::pair<std::size_t, CoordinateId> replace;

                std::array<double, 3> sumOfDistances{ 0,0,0 };
                for (std::size_t d : {0, 1, 2}) {
                    for (std::size_t dd : {1, 2}) {
                        sumOfDistances[d] += (coords[v[d]] - coords[v[(d + dd) % 3]]).norm();
                    }
                }
                auto minPos = std::min_element(sumOfDistances.begin(), sumOfDistances.end());
                auto midId = std::distance(sumOfDistances.begin(), minPos);

                const auto& cMid = coords[v[midId]];
                const auto& cExt1 = coords[v[(midId + 1) % 3]];
                const auto& cExt2 = coords[v[(midId + 2) % 3]];

                if ((cMid - cExt1).norm() < (cMid - cExt2).norm()) {
                    coords[element.vertices[midId]] = coords[element.vertices[(midId + 1) % 3]];
                }
                else {
                    coords[element.vertices[midId]] = coords[element.vertices[(midId + 2) % 3]];
                }
            }
        }

        RedundancyCleaner::fuseCoords(mesh);
        RedundancyCleaner::cleanCoords(mesh);

        for (auto & group : mesh.groups) {
            for (auto& element : group.elements) {
                if (element.isLine()){
                    if (element.vertices.front() == element.vertices.back()) {
                        element.vertices.pop_back();
                        element.type = Element::Type::Node;
                    }
                }

                else if (element.isTriangle()) {
                    if (element.vertices.front() == element.vertices.back()) {
                        element.vertices.pop_back();
                    }
                    auto newEnd = std::unique(element.vertices.begin(), element.vertices.end());
                    element.vertices.resize(newEnd - element.vertices.begin());
                    
                    if (element.vertices.size() == 1){
                        element.type = Element::Type::Node;
                    }
                    else if (element.vertices.size() == 2) {
                        element.type = Element::Type::Line;
                    }
                }
            }
        }
    }
     
    std::stringstream msg;
    bool breaksPostCondition = false;
    for (auto const& group : mesh.groups) {
        for (auto const& element : group.elements) {
            if (element.isNode() || element.isLine()) {
                continue;
            }

            double area = Geometry::area(Geometry::asTriV(element, mesh.coordinates));
            if (element.isTriangle() && area < areaThreshold) {
                breaksPostCondition = true;
                msg << std::endl;
                msg << "Group: " << &group - &mesh.groups.front()
                    << ", Element: " << &element - &group.elements.front() << std::endl;
                msg << meshTools::info(element, mesh) << std::endl;
            }
        }
    }
    if (breaksPostCondition) {
        msg << std::endl << "Triangles with area above threshold exist after collapsing.";
        throw std::runtime_error(msg.str());
    }
}


}
}