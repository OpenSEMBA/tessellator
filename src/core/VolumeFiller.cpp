#include "VolumeFiller.h"

#include "utils/RedundancyCleaner.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace meshlib::core {

using namespace utils;

namespace {

using Ray = std::array<CellDir, 2>;
using Rays = std::array<std::map<Ray, std::multiset<CellDir>>, 3>;

Cell coordinateCell(const Coordinate& coordinate, const GridTools& tools)
{
    const Relative relative = tools.getRelative(coordinate);
    Cell cell;
    for (Axis axis : {X, Y, Z}) {
        const auto rounded = std::round(relative[axis]);
        if (!GridTools::approxDir(relative[axis], rounded, 1e-7)) {
            throw std::runtime_error(
                "Volume shell contains a coordinate that is not on a grid vertex.");
        }
        cell[axis] = static_cast<CellDir>(rounded);
    }
    return cell;
}

std::pair<Axis, Cell> quadSurfel(
    const Element& quad,
    const Coordinates& coordinates,
    const GridTools& tools)
{
    if (!quad.isQuad()) {
        throw std::runtime_error(
            "Volume filling requires a closed shell made exclusively of quads.");
    }

    std::array<Cell, 4> cells;
    for (std::size_t index = 0; index < cells.size(); ++index) {
        cells[index] = coordinateCell(coordinates.at(quad.vertices[index]), tools);
    }

    std::vector<Axis> fixedAxes;
    for (Axis axis : {X, Y, Z}) {
        if (std::all_of(cells.begin(), cells.end(), [&](const Cell& cell) {
                return cell[axis] == cells.front()[axis];
            })) {
            fixedAxes.push_back(axis);
        }
    }
    if (fixedAxes.size() != 1) {
        throw std::runtime_error(
            "Volume shell contains a quad that is not on one grid face.");
    }

    const Axis axis = fixedAxes.front();
    Cell lower = cells.front();
    Cell upper = cells.front();
    for (const auto& cell : cells) {
        for (Axis direction : {X, Y, Z}) {
            lower[direction] = std::min(lower[direction], cell[direction]);
            upper[direction] = std::max(upper[direction], cell[direction]);
        }
    }
    for (Axis direction : {X, Y, Z}) {
        if (direction != axis && upper[direction] - lower[direction] != 1) {
            throw std::runtime_error(
                "Volume shell contains a quad spanning more than one grid face.");
        }
    }
    return {axis, lower};
}

CoordinateId findOrAddCoordinate(
    Mesh& mesh,
    std::map<Cell, CoordinateId>& coordinateIds,
    const Cell& cell,
    const GridTools& tools)
{
    const auto found = coordinateIds.find(cell);
    if (found != coordinateIds.end()) {
        return found->second;
    }
    const CoordinateId id = mesh.coordinates.size();
    mesh.coordinates.push_back(tools.getPos(GridTools::toRelative(cell)));
    coordinateIds.emplace(cell, id);
    return id;
}

Element buildHexahedron(
    Mesh& mesh,
    std::map<Cell, CoordinateId>& coordinateIds,
    const Cell& lower,
    const Cell& upper,
    const GridTools& tools)
{
    std::array<Cell, 8> vertices;
    vertices[0] = Cell({lower[X], lower[Y], lower[Z]});
    vertices[1] = Cell({upper[X], lower[Y], lower[Z]});
    vertices[2] = Cell({upper[X], upper[Y], lower[Z]});
    vertices[3] = Cell({lower[X], upper[Y], lower[Z]});
    vertices[4] = Cell({lower[X], lower[Y], upper[Z]});
    vertices[5] = Cell({upper[X], lower[Y], upper[Z]});
    vertices[6] = Cell({upper[X], upper[Y], upper[Z]});
    vertices[7] = Cell({lower[X], upper[Y], upper[Z]});

    Element hexahedron;
    hexahedron.type = Element::Type::Volume;
    for (const auto& vertex : vertices) {
        hexahedron.vertices.push_back(
            findOrAddCoordinate(mesh, coordinateIds, vertex, tools));
    }
    return hexahedron;
}

}

VolumeFiller::VolumeFiller(const Mesh& staircasedSurface) :
    GridTools(staircasedSurface.grid)
{
    mesh_.grid = staircasedSurface.grid;
    mesh_.coordinates = staircasedSurface.coordinates;
    mesh_.groups.resize(staircasedSurface.groups.size());
    std::map<Cell, CoordinateId> coordinateIds;
    for (CoordinateId id = 0; id < staircasedSurface.coordinates.size(); ++id) {
        coordinateIds.emplace(
            coordinateCell(staircasedSurface.coordinates[id], *this), id);
    }

    for (GroupId groupId = 0; groupId < staircasedSurface.groups.size(); ++groupId) {
        const auto& inputGroup = staircasedSurface.groups[groupId];
        auto& outputGroup = mesh_.groups[groupId];
        outputGroup.name = inputGroup.name;
        if (inputGroup.elements.empty()) {
            continue;
        }

        Rays rays;
        for (const auto& element : inputGroup.elements) {
            if (element.type != Element::Type::Surface) {
                continue;
            }
            Axis axis;
            Cell surfel;
            std::tie(axis, surfel) = quadSurfel(element, staircasedSurface.coordinates, *this);
            const Axis axis1 = (axis + 1) % 3;
            const Axis axis2 = (axis + 2) % 3;
            rays[axis][{surfel[axis1], surfel[axis2]}].insert(surfel[axis]);
        }

        Axis fillAxis = X;
        for (Axis axis : {Y, Z}) {
            if (rays[axis].size() < rays[fillAxis].size()) {
                fillAxis = axis;
            }
        }
        const Axis axis1 = (fillAxis + 1) % 3;
        const Axis axis2 = (fillAxis + 2) % 3;
        for (const auto& ray : rays[fillAxis]) {
            const auto& crossings = ray.second;
            if (crossings.size() % 2 != 0) {
                std::stringstream message;
                message << "Volume shell has an odd number of crossings on a grid ray in group "
                        << groupId << ".";
                throw std::runtime_error(message.str());
            }
            for (auto crossing = crossings.begin(); crossing != crossings.end();) {
                const CellDir begin = *crossing++;
                const CellDir end = *crossing++;
                if (begin == end) {
                    continue;
                }
                Cell lower;
                Cell upper;
                lower[fillAxis] = begin;
                upper[fillAxis] = end;
                lower[axis1] = ray.first[0];
                upper[axis1] = ray.first[0] + 1;
                lower[axis2] = ray.first[1];
                upper[axis2] = ray.first[1] + 1;
                outputGroup.elements.push_back(
                    buildHexahedron(mesh_, coordinateIds, lower, upper, *this));
            }
        }
    }

    RedundancyCleaner::cleanCoords(mesh_);
}

Mesh VolumeFiller::getMesh() const
{
    return mesh_;
}

}
