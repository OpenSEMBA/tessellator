#include "VolumeShellExtractor.h"

#include "utils/Geometry.h"
#include "utils/RedundancyCleaner.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace meshlib::core {

namespace {

using Face = std::array<CoordinateId, 3>;
using Edge = std::array<CoordinateId, 2>;

Face faceKey(const Face& face)
{
    Face key = face;
    std::sort(key.begin(), key.end());
    return key;
}

Face asFace(const Element& element)
{
    return {element.vertices[0], element.vertices[1], element.vertices[2]};
}

Edge edgeKey(CoordinateId first, CoordinateId second)
{
    return first < second ? Edge{first, second} : Edge{second, first};
}

[[noreturn]] void fail(GroupId groupId, const std::string& reason)
{
    std::stringstream message;
    message << "Invalid volume group " << groupId << ": " << reason;
    throw std::runtime_error(message.str());
}

void validateVertexIds(
    const Element& element,
    const Coordinates& coordinates,
    GroupId groupId)
{
    std::set<CoordinateId> unique;
    for (const CoordinateId id : element.vertices) {
        if (id >= coordinates.size()) {
            fail(groupId, "an element references a coordinate outside the mesh.");
        }
        for (std::size_t axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(coordinates[id][axis])) {
                fail(groupId, "an element references a non-finite coordinate.");
            }
        }
        unique.insert(id);
    }
    if (unique.size() != element.vertices.size()) {
        fail(groupId, "an element contains repeated vertices.");
    }
}

double signedTetrahedronVolume6(
    const Coordinate& first,
    const Coordinate& second,
    const Coordinate& third,
    const Coordinate& fourth)
{
    return ((second - first) ^ (third - first)) * (fourth - first);
}

Face outwardFace(
    Face face,
    CoordinateId opposite,
    const Coordinates& coordinates)
{
    const Coordinate& first = coordinates[face[0]];
    const Coordinate normal =
        (coordinates[face[1]] - first) ^ (coordinates[face[2]] - first);
    if (normal * (coordinates[opposite] - first) > 0.0) {
        std::swap(face[1], face[2]);
    }
    return face;
}

Elements extractTetrahedronBoundary(
    const Group& group,
    const Coordinates& coordinates,
    GroupId groupId)
{
    std::set<std::array<CoordinateId, 4>> tetrahedrons;
    std::map<Face, std::vector<Face>> faces;

    for (const Element& element : group.elements) {
        validateVertexIds(element, coordinates, groupId);
        std::array<CoordinateId, 4> tetrahedron{
            element.vertices[0], element.vertices[1],
            element.vertices[2], element.vertices[3]};
        auto tetrahedronKey = tetrahedron;
        std::sort(tetrahedronKey.begin(), tetrahedronKey.end());
        if (!tetrahedrons.insert(tetrahedronKey).second) {
            fail(groupId, "it contains a duplicate tetrahedron.");
        }

        if (std::abs(signedTetrahedronVolume6(
                coordinates[tetrahedron[0]], coordinates[tetrahedron[1]],
                coordinates[tetrahedron[2]], coordinates[tetrahedron[3]]))
            <= utils::Geometry::NORM_TOLERANCE) {
            fail(groupId, "it contains a degenerate tetrahedron.");
        }

        for (std::size_t opposite = 0; opposite < tetrahedron.size(); ++opposite) {
            Face face;
            std::size_t faceIndex = 0;
            for (std::size_t vertex = 0; vertex < tetrahedron.size(); ++vertex) {
                if (vertex != opposite) {
                    face[faceIndex++] = tetrahedron[vertex];
                }
            }
            face = outwardFace(face, tetrahedron[opposite], coordinates);
            auto& occurrences = faces[faceKey(face)];
            occurrences.push_back(face);
            if (occurrences.size() > 2) {
                fail(groupId, "a tetrahedron face is shared more than twice.");
            }
        }
    }

    Elements boundary;
    for (const auto& entry : faces) {
        if (entry.second.size() == 1) {
            const Face& face = entry.second.front();
            boundary.emplace_back(
                CoordinateIds{face[0], face[1], face[2]}, Element::Type::Surface);
        }
    }
    if (boundary.empty()) {
        fail(groupId, "its tetrahedrons have no external boundary.");
    }
    return boundary;
}

Elements extractSurfaceBoundary(
    const Group& group,
    const Coordinates& coordinates,
    GroupId groupId)
{
    Elements boundary;
    std::set<Face> faces;
    boundary.reserve(group.elements.size());
    for (const Element& element : group.elements) {
        validateVertexIds(element, coordinates, groupId);
        const Face face = asFace(element);
        if (!faces.insert(faceKey(face)).second) {
            fail(groupId, "it contains a duplicate triangle.");
        }
        if (utils::Geometry::isDegenerate(utils::Geometry::asTriV(element, coordinates))) {
            fail(groupId, "it contains a degenerate triangle.");
        }
        boundary.push_back(element);
    }
    return boundary;
}

int edgeDirection(const Element& face, const Edge& edge)
{
    for (std::size_t vertex = 0; vertex < face.vertices.size(); ++vertex) {
        const CoordinateId first = face.vertices[vertex];
        const CoordinateId second = face.vertices[(vertex + 1) % face.vertices.size()];
        if (first == edge[0] && second == edge[1]) {
            return 1;
        }
        if (first == edge[1] && second == edge[0]) {
            return -1;
        }
    }
    throw std::logic_error("A face does not contain its indexed edge.");
}

using EdgeFaces = std::map<Edge, std::vector<ElementId>>;

EdgeFaces buildEdgeFaces(const Elements& faces, GroupId groupId)
{
    EdgeFaces edgeFaces;
    for (ElementId faceId = 0; faceId < faces.size(); ++faceId) {
        const auto& vertices = faces[faceId].vertices;
        for (std::size_t vertex = 0; vertex < vertices.size(); ++vertex) {
            edgeFaces[edgeKey(vertices[vertex], vertices[(vertex + 1) % vertices.size()])]
                .push_back(faceId);
        }
    }
    for (const auto& entry : edgeFaces) {
        if (entry.second.size() == 1) {
            fail(groupId, "its surface is open.");
        }
        if (entry.second.size() != 2) {
            fail(groupId, "its surface contains a non-manifold edge.");
        }
    }
    return edgeFaces;
}

void validateVertexFans(
    const Elements& faces,
    const EdgeFaces& edgeFaces,
    GroupId groupId)
{
    std::map<CoordinateId, std::set<ElementId>> vertexFaces;
    std::map<CoordinateId, std::map<ElementId, std::set<ElementId>>> adjacency;
    for (ElementId faceId = 0; faceId < faces.size(); ++faceId) {
        for (CoordinateId vertex : faces[faceId].vertices) {
            vertexFaces[vertex].insert(faceId);
        }
    }
    for (const auto& entry : edgeFaces) {
        const ElementId first = entry.second[0];
        const ElementId second = entry.second[1];
        for (CoordinateId vertex : entry.first) {
            adjacency[vertex][first].insert(second);
            adjacency[vertex][second].insert(first);
        }
    }

    for (const auto& entry : vertexFaces) {
        const CoordinateId vertex = entry.first;
        const auto& incident = entry.second;
        std::set<ElementId> visited;
        std::queue<ElementId> pending;
        pending.push(*incident.begin());
        visited.insert(*incident.begin());
        while (!pending.empty()) {
            const ElementId current = pending.front();
            pending.pop();
            for (ElementId next : adjacency[vertex][current]) {
                if (visited.insert(next).second) {
                    pending.push(next);
                }
            }
        }
        if (visited.size() != incident.size()) {
            fail(groupId, "its surface contains a non-manifold vertex.");
        }
    }
}

std::vector<std::vector<ElementId>> orientComponents(
    Elements& faces,
    const EdgeFaces& edgeFaces,
    GroupId groupId)
{
    struct Neighbor {
        ElementId face;
        bool requiresFlip;
    };
    std::vector<std::vector<Neighbor>> adjacency(faces.size());
    for (const auto& entry : edgeFaces) {
        const ElementId first = entry.second[0];
        const ElementId second = entry.second[1];
        const bool requiresFlip =
            edgeDirection(faces[first], entry.first)
            == edgeDirection(faces[second], entry.first);
        adjacency[first].push_back({second, requiresFlip});
        adjacency[second].push_back({first, requiresFlip});
    }

    std::vector<int> flipped(faces.size(), -1);
    std::vector<std::vector<ElementId>> components;
    for (ElementId seed = 0; seed < faces.size(); ++seed) {
        if (flipped[seed] != -1) {
            continue;
        }
        components.emplace_back();
        std::queue<ElementId> pending;
        pending.push(seed);
        flipped[seed] = 0;
        while (!pending.empty()) {
            const ElementId current = pending.front();
            pending.pop();
            components.back().push_back(current);
            for (const Neighbor& neighbor : adjacency[current]) {
                const int required = flipped[current] ^ neighbor.requiresFlip;
                if (flipped[neighbor.face] == -1) {
                    flipped[neighbor.face] = required;
                    pending.push(neighbor.face);
                } else if (flipped[neighbor.face] != required) {
                    fail(groupId, "its surface is not orientable.");
                }
            }
        }
    }
    for (ElementId faceId = 0; faceId < faces.size(); ++faceId) {
        if (flipped[faceId] == 1) {
            std::swap(faces[faceId].vertices[1], faces[faceId].vertices[2]);
        }
    }
    return components;
}

void orientOutward(
    Elements& faces,
    const Coordinates& coordinates,
    const std::vector<std::vector<ElementId>>& components,
    GroupId groupId)
{
    for (const auto& component : components) {
        const Coordinate origin = coordinates[faces[component.front()].vertices[0]];
        double volume6 = 0.0;
        for (ElementId faceId : component) {
            const Face face = asFace(faces[faceId]);
            volume6 += (coordinates[face[0]] - origin)
                * ((coordinates[face[1]] - origin) ^ (coordinates[face[2]] - origin));
        }
        if (std::abs(volume6) <= utils::Geometry::NORM_TOLERANCE) {
            fail(groupId, "a closed component encloses zero volume.");
        }
        if (volume6 < 0.0) {
            for (ElementId faceId : component) {
                std::swap(faces[faceId].vertices[1], faces[faceId].vertices[2]);
            }
        }
    }
}

Elements extractGroupShell(
    const Group& group,
    const Coordinates& coordinates,
    GroupId groupId)
{
    if (group.elements.empty()) {
        return {};
    }

    const bool hasTriangles = std::any_of(
        group.elements.begin(), group.elements.end(),
        [](const Element& element) { return element.isTriangle(); });
    const bool hasTetrahedrons = std::any_of(
        group.elements.begin(), group.elements.end(),
        [](const Element& element) { return element.isTetrahedron(); });
    if (hasTriangles && hasTetrahedrons) {
        fail(groupId, "triangles and tetrahedrons cannot be mixed.");
    }
    if (!hasTriangles && !hasTetrahedrons) {
        fail(groupId, "only triangles or tetrahedrons are supported.");
    }
    if (!std::all_of(
            group.elements.begin(), group.elements.end(),
            [hasTriangles](const Element& element) {
                return hasTriangles ? element.isTriangle() : element.isTetrahedron();
            })) {
        fail(groupId, "only triangles or tetrahedrons are supported.");
    }

    Elements faces = hasTriangles
        ? extractSurfaceBoundary(group, coordinates, groupId)
        : extractTetrahedronBoundary(group, coordinates, groupId);
    const EdgeFaces edgeFaces = buildEdgeFaces(faces, groupId);
    validateVertexFans(faces, edgeFaces, groupId);
    const auto components = orientComponents(faces, edgeFaces, groupId);
    orientOutward(faces, coordinates, components, groupId);
    std::sort(faces.begin(), faces.end(), [](const Element& first, const Element& second) {
        return faceKey(asFace(first)) < faceKey(asFace(second));
    });
    return faces;
}

}

VolumeShellExtractor::VolumeShellExtractor(const Mesh& volumeMesh) : mesh_(volumeMesh)
{
    for (GroupId groupId = 0; groupId < mesh_.groups.size(); ++groupId) {
        mesh_.groups[groupId].elements = extractGroupShell(
            volumeMesh.groups[groupId], volumeMesh.coordinates, groupId);
    }
    utils::RedundancyCleaner::cleanCoords(mesh_);
}

Mesh VolumeShellExtractor::getMesh() const
{
    return mesh_;
}

}
