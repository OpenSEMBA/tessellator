#include "EmbeddedElementProcessor.h"

#include "core/EmbeddedElementBuilder.h"
#include "utils/MeshTools.h"

#include <map>
#include <stdexcept>
#include <string>
#include <utility>

namespace meshlib {
namespace app {

namespace {

bool hasSurfaceElements(const Mesh& mesh)
{
    return utils::meshTools::countMeshElementsIf(
        mesh, [](const Element& element) {
            return element.isTriangle() || element.isQuad();
        }) > 0;
}

bool hasLineElements(const Mesh& mesh)
{
    return utils::meshTools::countMeshElementsIf(
        mesh, [](const Element& element) {
            return element.isLine();
        }) > 0;
}

Mesh buildEmbeddedLineMesh(
    const core::EmbeddedElementBuilder::Components& components,
    const Grid& grid)
{
    Mesh result;
    result.grid = grid;
    result.groups.resize(1);
    result.groups[0].name = "line";

    std::map<Cell, CoordinateId> coordinateIds;
    auto addVertex = [&result, &coordinateIds](const Cell& cell) {
        const auto existing = coordinateIds.find(cell);
        if (existing != coordinateIds.end()) {
            return existing->second;
        }
        Coordinate coordinate;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            coordinate[axis] = static_cast<CoordinateDir>(cell[axis]);
        }
        const CoordinateId id = result.coordinates.size();
        result.coordinates.push_back(coordinate);
        coordinateIds.emplace(cell, id);
        return id;
    };

    for (const auto& component : components) {
        for (const auto& segment : component) {
            Element line;
            line.type = Element::Type::Line;
            line.vertices = {
                addVertex(segment[0]),
                addVertex(segment[1])
            };
            result.groups[0].elements.push_back(std::move(line));
        }
    }
    return result;
}

Mesh buildCombinedHostSurface(
    const std::vector<MeshedObject>& objects,
    const std::vector<bool>& isEmbedded)
{
    Mesh combined;
    std::map<Coordinate, CoordinateId> coordinateIds;
    bool initialized = false;

    for (std::size_t index = 0; index < objects.size(); ++index) {
        if (isEmbedded[index]) {
            continue;
        }
        const MeshedObject& object = objects[index];
        if (object.definition.ghost
            || object.definition.isVolume
            || object.mesherType != staircase_mesher
            || !hasSurfaceElements(object.mesh)) {
            continue;
        }

        Mesh relative = object.mesh;
        utils::meshTools::convertToRelativeCoordinates(relative);

        if (!initialized) {
            combined.grid = relative.grid;
            combined.groups.resize(1);
            combined.groups[0].name = "host";
            initialized = true;
        }

        for (const Group& group : relative.groups) {
            for (const Element& element : group.elements) {
                if (!element.isTriangle() && !element.isQuad()) {
                    continue;
                }
                Element remapped;
                remapped.type = Element::Type::Surface;
                for (const CoordinateId vertexId : element.vertices) {
                    const Coordinate& coordinate = relative.coordinates[vertexId];
                    auto [it, inserted] = coordinateIds.emplace(
                        coordinate, combined.coordinates.size());
                    if (inserted) {
                        combined.coordinates.push_back(coordinate);
                    }
                    remapped.vertices.push_back(it->second);
                }
                combined.groups[0].elements.push_back(std::move(remapped));
            }
        }
    }

    return combined;
}

} // namespace

void readEmbeddedOptions(
    const nlohmann::json& options,
    meshlib::meshers::StaircaseMesherOptions& result)
{
    if (options.contains("embedded")) {
        const auto& embedded = options["embedded"];
        if (!embedded.is_string()) {
            throw std::runtime_error(
                "embedded must be \"line\" or \"surface\"");
        }
        const std::string value = embedded.get<std::string>();
        if (value == "line") {
            result.embeddedMode = meshlib::meshers::EmbeddedMode::Line;
        } else if (value == "surface") {
            result.embeddedMode = meshlib::meshers::EmbeddedMode::Surface;
        } else {
            throw std::runtime_error(
                "embedded must be \"line\" or \"surface\"");
        }
    }
    if (options.contains("embeddedAllowOnSurfacePerimeter")) {
        const auto& allow = options["embeddedAllowOnSurfacePerimeter"];
        if (!allow.is_boolean()) {
            throw std::runtime_error(
                "embeddedAllowOnSurfacePerimeter must be a boolean");
        }
        result.embeddedAllowOnSurfacePerimeter = allow.get<bool>();
    }
}

void applyEmbeddedModes(std::vector<MeshedObject>& objects)
{
    std::vector<bool> isEmbedded(objects.size(), false);
    std::vector<std::size_t> embeddedIndices;
    for (std::size_t index = 0; index < objects.size(); ++index) {
        const MeshedObject& object = objects[index];
        if (object.embeddedMode == meshlib::meshers::EmbeddedMode::None) {
            continue;
        }

        // Volumes are never valid embedded targets, even when the option is
        // inherited from the global mesher block.
        if (object.definition.isVolume) {
            isEmbedded[index] = true;
            embeddedIndices.push_back(index);
            continue;
        }

        // Ghost objects are excluded from cross-object decisions. An
        // inherited global option must not turn a ghost line into an embedded line;
        // explicitly configuring one remains an error.
        if (object.definition.ghost && !object.embeddedIsExplicit) {
            continue;
        }

        // A global embedded option also reaches host surface objects. They
        // are hosts, not embedded line objects.
        const bool isHostSurface =
            !hasLineElements(object.mesh)
            && hasSurfaceElements(object.mesh);
        if (isHostSurface) {
            continue;
        }

        isEmbedded[index] = true;
        embeddedIndices.push_back(index);
    }

    if (embeddedIndices.empty()) {
        return;
    }

    const Mesh hostSurface = buildCombinedHostSurface(objects, isEmbedded);
    if (hostSurface.countElems() == 0) {
        throw std::runtime_error(
            "Embedded geometry requires at least one non-ghost staircase surface object.");
    }

    for (const std::size_t index : embeddedIndices) {
        MeshedObject& object = objects[index];
        try {
            if (object.definition.ghost) {
                throw std::runtime_error(
                    "Embedded geometry cannot be used on a ghost object.");
            }
            if (object.definition.isVolume) {
                throw std::runtime_error(
                    "Embedded geometry cannot be used on a volume object.");
            }
            if (object.mesherType != staircase_mesher) {
                throw std::runtime_error(
                    "Embedded geometry can only be used with the staircase mesher.");
            }

            const std::size_t surfaces = utils::meshTools::countMeshElementsIf(
                object.mesh, [](const Element& element) {
                    return element.isTriangle() || element.isQuad()
                        || element.type == Element::Type::Volume;
                });
            if (surfaces != 0) {
                throw std::runtime_error(
                    "Embedded geometry requires an object containing only lines or nodes.");
            }

            Mesh relativeLine = object.mesh;
            utils::meshTools::convertToRelativeCoordinates(relativeLine);
            const core::EmbeddedElementBuilder::Components components =
                core::EmbeddedElementBuilder::buildOrderedUnitSegments(relativeLine);
            if (components.empty()) {
                throw std::runtime_error(
                    "Embedded geometry requires at least one line element.");
            }

            const Mesh cellAlignedSurface =
                core::EmbeddedElementBuilder::buildCellAlignedSurface(
                    hostSurface, components);
            core::EmbeddedElementBuilder::validateEmbedding(
                cellAlignedSurface,
                components,
                object.embeddedAllowOnSurfacePerimeter);

            // Mesh currently carries only grid, coordinates and groups, so
            // replacing it here does not drop any other object metadata.
            if (object.embeddedMode == meshlib::meshers::EmbeddedMode::Surface) {
                Mesh strip = core::EmbeddedElementBuilder::buildSurfaceStrip(
                    cellAlignedSurface, components);
                strip.grid = object.mesh.grid;
                utils::meshTools::convertToAbsoluteCoordinates(strip);
                strip.groups.resize(1);
                strip.groups[0].name = object.definition.group;
                object.mesh = std::move(strip);
            }
            else {
                Mesh cleanLine = buildEmbeddedLineMesh(
                    components, object.mesh.grid);
                utils::meshTools::convertToAbsoluteCoordinates(cleanLine);
                cleanLine.groups[0].name = object.definition.group;
                object.mesh = std::move(cleanLine);
            }
        } catch (const std::exception& e) {
            throw std::runtime_error(
                "Embedded geometry '" + object.definition.group + "': " + e.what());
        }
    }
}

}
}
