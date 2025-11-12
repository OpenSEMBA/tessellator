#include "Splicer.h"
#include "utils/Geometry.h"

namespace meshlib {
namespace core {
// using namespace utils;

Splicer::Splicer(const Mesh& inputMesh) : GridTools(inputMesh.grid) {
    this->mesh_.coordinates = inputMesh.coordinates;

    this->mesh_.groups.reserve(inputMesh.groups.size());
    for (auto& group : inputMesh.groups) {
        Group newGroup;
        newGroup.elements.reserve(group.elements.size());
        spliceTrianglesInGroup(newGroup, group);
        
        this->mesh_.groups.push_back(newGroup);
    }



}

void Splicer::spliceTrianglesInGroup(Group & elementGroup, const Group& inputGroup) {
    for (const auto& element : inputGroup.elements) {
        if (!element.isTriangle()) {
            continue;
        }

        auto innerPointIds = getInnerPointIds(element);

        Elements firstPointTriangles = spliceTriangleIntoList(element, 0, innerPointIds[0]);
        Elements secondPointTriangles = spliceTriangleIntoList(firstPointTriangles[firstPointTriangles.size() - 1], 1, innerPointIds[1]);

        Elements thirdPointTriangles;
        
        if (firstPointTriangles.size() != 1 || secondPointTriangles.size() == 1) {
            thirdPointTriangles = spliceTriangleIntoList(firstPointTriangles[0], 2, innerPointIds[2]);
        }
        else {
            thirdPointTriangles = spliceTriangleIntoList(secondPointTriangles[secondPointTriangles.size() - 1], 1, innerPointIds[2]);
        }

        auto firstPointStart = firstPointTriangles.begin();
        if (thirdPointTriangles.size() != 1) {
            ++firstPointStart;
        }

        if (firstPointTriangles.size() != 1) {
            elementGroup.elements.insert(elementGroup.elements.end(), firstPointStart, firstPointTriangles.end() - 1);
        }
        
        auto secondPointEnd = secondPointTriangles.end();
        if (firstPointTriangles.size() == 1 && thirdPointTriangles.size() != 1) {
            --secondPointEnd;
        }

        elementGroup.elements.insert(elementGroup.elements.end(), secondPointTriangles.begin(), secondPointEnd);

        if (thirdPointTriangles.size() != 1) {
            elementGroup.elements.insert(elementGroup.elements.end(), thirdPointTriangles.begin(), thirdPointTriangles.end());
        }
    }
}

std::vector<std::map<double, CoordinateId>> Splicer::getInnerPointIds(const Element& triangle) {
    std::vector<std::map<double, CoordinateId>> innerPointIds(3);

    auto triV = utils::Geometry::asTriV(triangle, this->mesh_.coordinates);
    for (CoordinateId c = 0; c < this->mesh_.coordinates.size(); ++c) {
        if (std::find(triangle.vertices.begin(), triangle.vertices.end(), c) != triangle.vertices.end()) {
            continue;
        }

        auto& coordinate = this->mesh_.coordinates[c];

        for (std::size_t vertexPosition = 0; vertexPosition < 3; ++vertexPosition) {
            if (isBetween(triV[(vertexPosition + 1) % 3], triV[(vertexPosition + 2) % 3], coordinate)) {
                double distance = (coordinate - triV[(vertexPosition + 1) % 3]).norm();
                innerPointIds[vertexPosition][distance] = c;
                break;
            }
        }
    }

    return innerPointIds;
}


bool Splicer::isBetween(const Coordinate& leftExtreme, const Coordinate& rightExtreme, const Coordinate& betweenPoint) {
    auto extremesDistance = rightExtreme - leftExtreme;
    auto betweenDistance = betweenPoint - leftExtreme;
    auto crossProduct = betweenDistance ^ extremesDistance;

    if (!approxDir(crossProduct.norm(), 0.0)) {
        return false;
    }

    auto dotProduct = betweenDistance * extremesDistance;

    if (dotProduct < 0.0) {
        return false;
    }

    double extremeNorm = extremesDistance.norm();

    if (dotProduct > (extremeNorm * extremeNorm)) {
        return false;
    }

    return true;
}

Elements Splicer::spliceTriangleIntoList(const Element& triangle, std::size_t vertexPosition, const std::map<double, CoordinateId>& innerPoints) {
    Elements newTriangles;
    newTriangles.reserve(innerPoints.size() + 1);

    if (innerPoints.size() == 0) {
        newTriangles.push_back(triangle);
        return newTriangles;
    }

    CoordinateId commonId = triangle.vertices[vertexPosition];
    CoordinateId leftExtremeId = triangle.vertices[(vertexPosition + 1) % 3];
    CoordinateId rightExtremeId = triangle.vertices[(vertexPosition + 2) % 3];

    newTriangles.push_back(Element({ commonId, leftExtremeId, innerPoints.begin()->second }, Element::Type::Surface));
    auto secondPointIt = innerPoints.begin();
    auto thirdPointIt = secondPointIt;
    ++thirdPointIt;

    for (; thirdPointIt != innerPoints.end(); ++secondPointIt, ++thirdPointIt) {
        newTriangles.push_back(Element({ commonId, secondPointIt->second, thirdPointIt->second }, Element::Type::Surface));
    }
    newTriangles.push_back(Element({ commonId, innerPoints.rbegin()->second, rightExtremeId }, Element::Type::Surface));

    return newTriangles;
}

}
}