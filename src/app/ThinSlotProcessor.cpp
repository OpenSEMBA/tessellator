#include "ThinSlotProcessor.h"

#include "core/ThinSlotBuilder.h"
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

Mesh buildThinSlotLineMesh(
    const core::ThinSlotBuilder::Components& components,
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
    const std::vector<bool>& isSlot)
{
    Mesh combined;
    std::map<Coordinate, CoordinateId> coordinateIds;
    bool initialized = false;

    for (std::size_t index = 0; index < objects.size(); ++index) {
        if (isSlot[index]) {
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

void readThinSlotOptions(
    const nlohmann::json& options,
    meshlib::meshers::StaircaseMesherOptions& result)
{
    if (options.contains("thinSlot")) {
        const auto& thinSlot = options["thinSlot"];
        if (!thinSlot.is_string()) {
            throw std::runtime_error(
                "thinSlot must be \"line\" or \"surface\"");
        }
        const std::string value = thinSlot.get<std::string>();
        if (value == "line") {
            result.thinSlotMode = meshlib::meshers::ThinSlotMode::Line;
        } else if (value == "surface") {
            result.thinSlotMode = meshlib::meshers::ThinSlotMode::Surface;
        } else {
            throw std::runtime_error(
                "thinSlot must be \"line\" or \"surface\"");
        }
    }
    if (options.contains("thinSlotAllowOnSurfacePerimeter")) {
        const auto& allow = options["thinSlotAllowOnSurfacePerimeter"];
        if (!allow.is_boolean()) {
            throw std::runtime_error(
                "thinSlotAllowOnSurfacePerimeter must be a boolean");
        }
        result.thinSlotAllowOnSurfacePerimeter = allow.get<bool>();
    }
}

void applyThinSlotModes(std::vector<MeshedObject>& objects)
{
    std::vector<bool> isSlot(objects.size(), false);
    std::vector<std::size_t> slotIndices;
    for (std::size_t index = 0; index < objects.size(); ++index) {
        const MeshedObject& object = objects[index];
        if (object.thinSlotMode == meshlib::meshers::ThinSlotMode::None) {
            continue;
        }

        // Volumes are never valid thin-slot targets, even when the option is
        // inherited from the global mesher block.
        if (object.definition.isVolume) {
            isSlot[index] = true;
            slotIndices.push_back(index);
            continue;
        }

        // Ghost objects are excluded from cross-object decisions. An
        // inherited global option must not turn a ghost line into a slot;
        // explicitly configuring one remains an error.
        if (object.definition.ghost && !object.thinSlotIsExplicit) {
            continue;
        }

        // A global thinSlot option also reaches host surface objects. They
        // are hosts, not slots.
        const bool isHostSurface =
            !hasLineElements(object.mesh)
            && hasSurfaceElements(object.mesh);
        if (isHostSurface) {
            continue;
        }

        isSlot[index] = true;
        slotIndices.push_back(index);
    }

    if (slotIndices.empty()) {
        return;
    }

    const Mesh hostSurface = buildCombinedHostSurface(objects, isSlot);
    if (hostSurface.countElems() == 0) {
        throw std::runtime_error(
            "thinSlot requires at least one non-ghost staircase surface object.");
    }

    for (const std::size_t index : slotIndices) {
        MeshedObject& object = objects[index];
        try {
            if (object.definition.ghost) {
                throw std::runtime_error(
                    "thinSlot cannot be used on a ghost object.");
            }
            if (object.definition.isVolume) {
                throw std::runtime_error(
                    "thinSlot cannot be used on a volume object.");
            }
            if (object.mesherType != staircase_mesher) {
                throw std::runtime_error(
                    "thinSlot can only be used with the staircase mesher.");
            }

            const std::size_t surfaces = utils::meshTools::countMeshElementsIf(
                object.mesh, [](const Element& element) {
                    return element.isTriangle() || element.isQuad()
                        || element.type == Element::Type::Volume;
                });
            if (surfaces != 0) {
                throw std::runtime_error(
                    "thinSlot requires an object containing only lines or nodes.");
            }

            Mesh relativeLine = object.mesh;
            utils::meshTools::convertToRelativeCoordinates(relativeLine);
            const core::ThinSlotBuilder::Components components =
                core::ThinSlotBuilder::buildOrderedUnitSegments(relativeLine);
            if (components.empty()) {
                throw std::runtime_error(
                    "thinSlot requires at least one line element.");
            }

            const Mesh cellAlignedSurface =
                core::ThinSlotBuilder::buildCellAlignedSurface(
                    hostSurface, components);
            core::ThinSlotBuilder::validateEmbedding(
                cellAlignedSurface,
                components,
                object.thinSlotAllowOnSurfacePerimeter);

            // Mesh currently carries only grid, coordinates and groups, so
            // replacing it here does not drop any other object metadata.
            if (object.thinSlotMode == meshlib::meshers::ThinSlotMode::Surface) {
                Mesh strip = core::ThinSlotBuilder::buildSurfaceStrip(
                    cellAlignedSurface, components);
                strip.grid = object.mesh.grid;
                utils::meshTools::convertToAbsoluteCoordinates(strip);
                strip.groups.resize(1);
                strip.groups[0].name = object.definition.group;
                object.mesh = std::move(strip);
            }
            else {
                Mesh cleanLine = buildThinSlotLineMesh(
                    components, object.mesh.grid);
                utils::meshTools::convertToAbsoluteCoordinates(cleanLine);
                cleanLine.groups[0].name = object.definition.group;
                object.mesh = std::move(cleanLine);
            }
        } catch (const std::exception& e) {
            throw std::runtime_error(
                "Thin slot '" + object.definition.group + "': " + e.what());
        }
    }
}

}
}
