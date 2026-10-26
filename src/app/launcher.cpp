#include "launcher.h"
#include "vtkIO.h"

#include "meshers/MesherBase.h"
#include "meshers/StaircaseMesher.h"
#include "meshers/ConformalMesher.h"
#include "core/Staircaser.h"
#include "core/Compressor.h"
#include "core/ThinSlotBuilder.h"
#include "utils/GridTools.h"
#include "utils/MeshTools.h"
#include "utils/RedundancyCleaner.h"

#include <boost/program_options.hpp>
#include <nlohmann/json.hpp>

#include <iostream>
#include <filesystem>
#include <fstream>
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <map>
#include <optional>
#include <set>

namespace meshlib::app {

using namespace vtkIO;


namespace po = boost::program_options;

Grid parseGridFromJSON(const nlohmann::json &fileData)
{
    if (fileData.find("planes") != fileData.end()) {
        return fileData["planes"];
    }
    else {
        std::array<int, 3> nCells = {
        fileData["numberOfCells"][0],
        fileData["numberOfCells"][1],
        fileData["numberOfCells"][2]
        };
        std::array<double, 3> min, max;
        min = fileData["boundingBox"][0];
        max = fileData["boundingBox"][1];

        return {
            utils::GridTools::linspace(min[0], max[0], nCells[0] + 1),
            utils::GridTools::linspace(min[1], max[1], nCells[1] + 1),
            utils::GridTools::linspace(min[2], max[2], nCells[2] + 1)
        };
    }
}

std::vector<ObjectDefinition> readObjectsFromJSON(const nlohmann::json& fileData)
{
    std::vector<ObjectDefinition> objects;

    if (fileData.contains("objects")) {
        for (const auto& obj : fileData["objects"]) {
            ObjectDefinition objDef;
            objDef.filename = obj["filename"].get<std::string>();
            objDef.group = obj.value("group", std::filesystem::path(objDef.filename).stem().string());
            if (obj.contains("volume")){
                objDef.isVolume = obj["volume"];
            }
            objDef.ghost = obj.value("ghost", false);
            if (obj.contains("mesher")) {
                objDef.mesherOverride = obj["mesher"];
            }
            objects.push_back(objDef);
        }
    } else if (fileData.contains("object")) {
        ObjectDefinition objDef;
        objDef.filename = fileData["object"]["filename"].get<std::string>();
        objDef.group = std::filesystem::path(objDef.filename).stem().string();
        if (fileData["object"].contains("volume")){
            objDef.isVolume = fileData["object"]["volume"];
        }
        objDef.ghost = fileData["object"].value("ghost", false);
        if (fileData.contains("mesher")) {
            objDef.mesherOverride = fileData["mesher"];
        }
        objects.push_back(objDef);
    } else {
        throw std::runtime_error("No objects defined in input file");
    }

    return objects;
}

bool readSingleFileOutputOption(const nlohmann::json& fileData)
{
    if (!fileData.contains("output")) {
        return false;
    }
    return fileData["output"].value("singleFile", false);
}

Mesh readMesh(const nlohmann::json& fileData, const std::filesystem::path& folderPath, const ObjectDefinition& objDef)
{
    std::filesystem::path meshObjectPath = folderPath / objDef.filename;

    std::cout << "-- Reading mesh groups from: " << meshObjectPath;
    Mesh res = vtkIO::readInputMesh(meshObjectPath);
    std::cout << "....... [OK]" << std::endl;

    std::cout << "-- Reading grid from input file";
    res.grid = parseGridFromJSON(fileData["grid"]);
    std::cout << "....... [OK]" << std::endl;

    if (res.groups.empty()) {
        res.groups.push_back(Group{objDef.group, {}});
    } else {
        auto auxResult = utils::meshTools::extractGroupsByName(res, {objDef.group});
        if (auxResult.countElems() != 0) {
            return auxResult;
        } else {
            res.groups[0].name = objDef.group;
        }
    }

    return res;
}


std::string readMesherType(const nlohmann::json& fileData, const std::optional<nlohmann::json>& override)
    {
    nlohmann::json mesherConfig;

    if (override.has_value()) {
        mesherConfig = *override;
    } else if (fileData.contains("mesher")) {
        mesherConfig = fileData["mesher"];
    } else {
        return meshlib::app::staircase_mesher;
    }
    
    if (mesherConfig.contains("type")) {
        return mesherConfig["type"];
    } else {
        return meshlib::app::staircase_mesher;
    }
}

std::string readExtension(const nlohmann::json& fileData, const std::optional<nlohmann::json>& override)
{
    auto mesherType = readMesherType(fileData, override);
    if (mesherType == meshlib::app::staircase_mesher) {
        return "str";
    } else if (mesherType == meshlib::app::conformal_mesher) {
        return "cmsh";
    } else {
        throw std::runtime_error("Unsupported mesher type");
    }
}

meshlib::meshers::StaircaseMesherOptions readStaircaseMesherOptions(const nlohmann::json &fileData, bool isVolume, const std::optional<nlohmann::json>& override)
{    
    nlohmann::json mesherConfig;
    if (override.has_value()) {
        mesherConfig = *override;
    } else if (fileData.contains("mesher")) {
        mesherConfig = fileData["mesher"];
    }

    meshlib::meshers::StaircaseMesherOptions res;
    res.allow_log = true;
    if (isVolume){
        res.volumeGroups.insert(0);
    }
    if (mesherConfig.contains("options")) {
        const auto& options = mesherConfig["options"];
        if (options.contains("compress")) {
            res.compress = options["compress"];
        }
        if (options.contains("splitHexahedra")) {
            res.splitHexahedra = options["splitHexahedra"];
        }
        if (options.contains("decimalPlacesInCollapser")) {
            const auto& decimalPlaces = options["decimalPlacesInCollapser"];
            if (!decimalPlaces.is_number_integer() && !decimalPlaces.is_number_unsigned()) {
                throw std::runtime_error("decimalPlacesInCollapser must be a non-negative integer");
            }
            if (decimalPlaces.is_number_integer() && decimalPlaces.get<std::int64_t>() < 0) {
                throw std::runtime_error("decimalPlacesInCollapser must be a non-negative integer");
            }
            res.decimalPlacesInCollapser = decimalPlaces.get<int>();
        }
        if (options.contains("thinSlot")) {
            const auto& thinSlot = options["thinSlot"];
            if (!thinSlot.is_string()) {
                throw std::runtime_error(
                    "thinSlot must be \"line\" or \"surface\"");
            }
            const std::string value = thinSlot.get<std::string>();
            if (value == "line") {
                res.thinSlotMode = meshlib::meshers::ThinSlotMode::Line;
            } else if (value == "surface") {
                res.thinSlotMode = meshlib::meshers::ThinSlotMode::Surface;
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
            res.thinSlotAllowOnSurfacePerimeter = allow.get<bool>();
        }
    }

    return res;
}

meshlib::meshers::ConformalMesherOptions readConformalMesherOptions(const nlohmann::json& fileData, bool isVolume, const std::optional<nlohmann::json>& override)
{    
    nlohmann::json mesherConfig;
    if (override.has_value()) {
        mesherConfig = *override;
    } else if (fileData.contains("mesher")) {
        mesherConfig = fileData["mesher"];
    }
    
    meshlib::meshers::ConformalMesherOptions res;
    if (isVolume){
        res.volumeGroups.insert(0);
    }
    if (mesherConfig.contains("options")) {
        const auto& options = mesherConfig["options"];
        if (options.contains("edgePoints")) {
            const auto& edgePoints = options["edgePoints"];
            if (!edgePoints.is_number_integer() && !edgePoints.is_number_unsigned()) {
                throw std::runtime_error("edgePoints must be a non-negative integer");
            }
            if (edgePoints.is_number_integer() && edgePoints.get<std::int64_t>() < 0) {
                throw std::runtime_error("edgePoints must be a non-negative integer");
            }
            res.snapperOptions.edgePoints = edgePoints.get<std::size_t>();
        }
        if (options.contains("forbiddenLength")) {
            const auto& forbiddenLength = options["forbiddenLength"];
            if (!forbiddenLength.is_number()) {
                throw std::runtime_error("forbiddenLength must be a number between 0.0 and 0.5");
            }
            const auto value = forbiddenLength.get<double>();
            if (!std::isfinite(value) || value < 0.0 || value > 0.5) {
                throw std::runtime_error("forbiddenLength must be a number between 0.0 and 0.5");
            }
            res.snapperOptions.forbiddenLength = value;
        }
        res.compress = options.value("compress", res.compress);
        res.staircaseSharedCells = options.value(
            "staircaseSharedCells", res.staircaseSharedCells);
        res.mergeAxisAlignedTriangles = options.value(
            "mergeAxisAlignedTriangles", res.mergeAxisAlignedTriangles);
    }
    return res;
}

bool readExportGridOption(const nlohmann::json& fileData, const std::optional<nlohmann::json>& override)
{   
    nlohmann::json mesherConfig;
    if (override.has_value()) {
        mesherConfig = *override;
    } else if (fileData.contains("mesher")) {
        mesherConfig = fileData["mesher"];
    }
    
    if (mesherConfig.contains("options") && 
        mesherConfig["options"].contains("exportGrid")) {
        return mesherConfig["options"]["exportGrid"];
    }
    return true;
}

std::unique_ptr<meshlib::meshers::MesherBase> buildMesher(const Mesh& in, const nlohmann::json & fileData, const ObjectDefinition& objDef)
{
    auto mesherType = readMesherType(fileData, objDef.mesherOverride);
 
    if (mesherType == meshlib::app::staircase_mesher) {
        return std::make_unique<meshlib::meshers::StaircaseMesher>(meshlib::meshers::StaircaseMesher{
            in,
            readStaircaseMesherOptions(fileData, objDef.isVolume, objDef.mesherOverride)
        });
    } else if (mesherType == meshlib::app::conformal_mesher) {
        return std::make_unique<meshlib::meshers::ConformalMesher>(meshlib::meshers::ConformalMesher{in, readConformalMesherOptions(fileData, objDef.isVolume, objDef.mesherOverride)});
    } else {
        throw std::runtime_error("Unsupported mesher type");
    }
}

namespace {

struct MeshedObject {
    ObjectDefinition definition;
    std::string mesherType;
    std::string extension;
    bool staircaseSharedCells = false;
    bool compress = false;
    meshlib::meshers::ThinSlotMode thinSlotMode =
        meshlib::meshers::ThinSlotMode::None;
    bool thinSlotAllowOnSurfacePerimeter = false;
    Mesh mesh;
};

Mesh mergeObjectMeshes(const std::vector<MeshedObject>& objects)
{
    Mesh combined;
    if (objects.empty()) {
        return combined;
    }

    combined.grid = objects.front().mesh.grid;
    for (const auto& object : objects) {
        if (object.mesh.grid != combined.grid) {
            throw std::runtime_error("Cannot combine object meshes with different grids.");
        }
        if (object.mesh.groups.size() != 1) {
            throw std::runtime_error(
                "Each object must produce exactly one mesh group for combined processing.");
        }

        Mesh namedMesh = object.mesh;
        namedMesh.groups.front().name = object.definition.group;
        if (combined.groups.empty()) {
            combined = std::move(namedMesh);
        } else {
            utils::meshTools::mergeMeshAsNewGroup(combined, namedMesh);
        }
    }
    utils::RedundancyCleaner::fuseCoords(combined);
    utils::RedundancyCleaner::cleanCoords(combined);
    return combined;
}

void validateCombinedGroupNames(const std::vector<ObjectDefinition>& objects)
{
    std::set<std::string> groupNames;
    for (const auto& object : objects) {
        if (!groupNames.insert(object.group).second) {
            throw std::runtime_error(
                "Combined output requires unique object group names; duplicate group: " +
                object.group);
        }
    }
}

void validateObjectFiles(
    const std::filesystem::path& folderPath,
    const std::vector<ObjectDefinition>& objects)
{
    for (const auto& object : objects) {
        const auto path = folderPath / object.filename;
        if (!std::filesystem::exists(path)) {
            throw std::runtime_error("Object file not found: " + path.string());
        }
        if (!std::filesystem::is_regular_file(path)) {
            throw std::runtime_error(
                "Object path is not a regular file: " + path.string());
        }
    }
}

void staircaseSharedConformalCells(std::vector<MeshedObject>& objects)
{
    const bool hasEnabledConformalObject = std::any_of(
        objects.begin(), objects.end(), [](const MeshedObject& object) {
            return !object.definition.ghost &&
                object.mesherType == conformal_mesher &&
                object.staircaseSharedCells;
        });
    const auto participatingObjectCount = std::count_if(
        objects.begin(), objects.end(), [](const MeshedObject& object) {
            return !object.definition.ghost;
        });
    if (!hasEnabledConformalObject || participatingObjectCount < 2) {
        return;
    }

    Mesh relativeCombined = mergeObjectMeshes(objects);
    utils::meshTools::convertToRelativeCoordinates(relativeCombined);
    std::set<GroupId> ghostGroups;
    for (GroupId groupId = 0; groupId < objects.size(); ++groupId) {
        if (objects[groupId].definition.ghost) {
            ghostGroups.insert(groupId);
        }
    }
    const auto sharedCells = meshlib::meshers::ConformalMesher::cellsSharedByGroups(
        relativeCombined, ghostGroups);
    if (sharedCells.empty()) {
        return;
    }

    for (auto& object : objects) {
        if (object.definition.ghost || object.mesherType != conformal_mesher ||
            !object.staircaseSharedCells) {
            continue;
        }

        Mesh relativeMesh = object.mesh;
        utils::meshTools::convertToRelativeCoordinates(relativeMesh);
        relativeMesh = meshlib::core::Staircaser{relativeMesh}.getSelectiveMesh(
            sharedCells, meshlib::core::Staircaser::GapsFillingType::Insert);
        const auto nonConformalCells = meshlib::meshers::ConformalMesher::findNonConformalCells(
            relativeMesh);
        if (!nonConformalCells.empty()) {
            relativeMesh = meshlib::core::Staircaser{relativeMesh}.getSelectiveMesh(
                nonConformalCells, meshlib::core::Staircaser::GapsFillingType::Insert);
        }
        utils::RedundancyCleaner::removeOverlappedDimensionOneAndLowerElementsAndEquivalentSurfaces(
            relativeMesh);
        utils::RedundancyCleaner::removeGeometricallyOverlappedDimensionOneAndLowerElements(
            relativeMesh);
        if (object.compress && !object.definition.isVolume) {
            meshlib::core::Compressor::compressSurfacesInMesh(relativeMesh);
            meshlib::core::Compressor::compressLinesInMesh(relativeMesh);
            utils::RedundancyCleaner::fuseCoords(relativeMesh);
            utils::RedundancyCleaner::removeOverlappedDimensionOneAndLowerElementsAndEquivalentSurfaces(
                relativeMesh);
            utils::RedundancyCleaner::removeGeometricallyOverlappedDimensionOneAndLowerElements(
                relativeMesh);
        }
        utils::RedundancyCleaner::cleanCoords(relativeMesh);
        object.mesh = std::move(relativeMesh);
        object.mesh.groups.front().name = object.definition.group;
        utils::meshTools::convertToAbsoluteCoordinates(object.mesh);
    }
}

} // namespace

bool hasSurfaceElements(const Mesh& mesh)
{
    for (const Group& group : mesh.groups) {
        for (const Element& element : group.elements) {
            if (element.isTriangle() || element.isQuad()) {
                return true;
            }
        }
    }
    return false;
}

bool hasLineElements(const Mesh& mesh)
{
    for (const Group& group : mesh.groups) {
        for (const Element& element : group.elements) {
            if (element.isLine()) {
                return true;
            }
        }
    }
    return false;
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

Mesh buildCombinedHostSurface(const std::vector<MeshedObject>& objects)
{
    Mesh combined;
    std::map<Coordinate, CoordinateId> coordinateIds;
    bool initialized = false;

    for (const MeshedObject& object : objects) {
        if (object.definition.ghost
            || object.definition.isVolume
            || object.mesherType != staircase_mesher
            || object.thinSlotMode != meshlib::meshers::ThinSlotMode::None
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

void applyThinSlotModes(std::vector<MeshedObject>& objects)
{
    const auto hasThinSlot = std::any_of(
        objects.begin(), objects.end(), [](const MeshedObject& object) {
            return object.thinSlotMode != meshlib::meshers::ThinSlotMode::None;
        });
    if (!hasThinSlot) {
        return;
    }

    const Mesh hostSurface = buildCombinedHostSurface(objects);
    if (hostSurface.countElems() == 0) {
        throw std::runtime_error(
            "thinSlot requires at least one non-ghost staircase surface object.");
    }

    for (MeshedObject& object : objects) {
        if (object.thinSlotMode == meshlib::meshers::ThinSlotMode::None) {
            continue;
        }

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

            const std::size_t surfaces = meshlib::utils::meshTools::countMeshElementsIf(
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

int launcher(int argc, const char* argv[])
{
    try {
        po::options_description desc("Allowed options");
        desc.add_options()
            ("help,h", "produce help message")
            ("input,i", po::value<std::string>(), "input file");

        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).
                options(desc).run(), vm);
        po::notify(vm);

        if (vm.count("help") || !vm.count("input")) {
            std::cout << desc << std::endl;
            return EXIT_SUCCESS;
        }

        std::string inputFileName = vm["input"].as<std::string>();
        std::cout << "-- Input file is: " << inputFileName << std::endl;

        const std::filesystem::path inputPath(inputFileName);
        if (!std::filesystem::exists(inputPath)) {
            throw std::runtime_error(
                "Input file not found: " + inputPath.string());
        }

        nlohmann::json inputFileData;
        {
            std::ifstream i(inputPath);
            if (!i) {
                throw std::runtime_error(
                    "Input file could not be opened: " + inputPath.string());
            }
            try {
                i >> inputFileData;
            } catch (const nlohmann::json::exception& e) {
                throw std::runtime_error(
                    "Could not parse input file '" + inputPath.string() +
                    "': " + e.what());
            }
        }

        std::vector<ObjectDefinition> objects = readObjectsFromJSON(inputFileData);
        std::filesystem::path outputFolder = getFolder(inputFileName);
        auto basename = getBasename(inputFileName);
        const bool singleFileOutput = readSingleFileOutputOption(inputFileData);
        if (singleFileOutput) {
            validateCombinedGroupNames(objects);
        }
        validateObjectFiles(outputFolder, objects);

        std::vector<MeshedObject> meshedObjects;
        meshedObjects.reserve(objects.size());

        for (const auto& objDef : objects) {
            std::cout << "\n-- Processing object: " << objDef.filename << " (group: " << objDef.group << ")" << std::endl;

            Mesh mesh = readMesh(inputFileData, outputFolder, objDef);

            auto mesher = buildMesher(mesh, inputFileData, objDef);
            Mesh resultMesh = mesher->mesh();
            if (resultMesh.groups.size() == 1) {
                resultMesh.groups.front().name = objDef.group;
            }

            const auto mesherType = readMesherType(inputFileData, objDef.mesherOverride);
            bool staircaseSharedCells = false;
            bool compress = false;
            meshlib::meshers::ThinSlotMode thinSlotMode =
                meshlib::meshers::ThinSlotMode::None;
            bool thinSlotAllowOnSurfacePerimeter = false;
            if (mesherType == conformal_mesher) {
                const auto conformalOptions = readConformalMesherOptions(
                    inputFileData, objDef.isVolume, objDef.mesherOverride);
                staircaseSharedCells = conformalOptions.staircaseSharedCells;
                compress = conformalOptions.compress;
            } else if (mesherType == staircase_mesher) {
                const auto& staircaseOptions =
                    dynamic_cast<const meshlib::meshers::StaircaseMesher&>(*mesher)
                        .getOptions();
                thinSlotMode = staircaseOptions.thinSlotMode;
                thinSlotAllowOnSurfacePerimeter =
                    staircaseOptions.thinSlotAllowOnSurfacePerimeter;
                if (thinSlotMode != meshlib::meshers::ThinSlotMode::None
                    && !objDef.isVolume
                    && !hasLineElements(resultMesh)
                    && hasSurfaceElements(resultMesh)) {
                    // A global thinSlot option also reaches the host surface
                    // objects. They are hosts, not slot objects.
                    thinSlotMode = meshlib::meshers::ThinSlotMode::None;
                }
            }
            meshedObjects.push_back({
                objDef,
                mesherType,
                readExtension(inputFileData, objDef.mesherOverride),
                staircaseSharedCells,
                compress,
                thinSlotMode,
                thinSlotAllowOnSurfacePerimeter,
                std::move(resultMesh)
            });
        }

        staircaseSharedConformalCells(meshedObjects);
        applyThinSlotModes(meshedObjects);

        if (singleFileOutput && !meshedObjects.empty()) {
            const auto outputFileName = basename + ".tessellator.vtk";
            exportMeshToVTU(
                outputFolder / outputFileName, mergeObjectMeshes(meshedObjects));
            std::cout << "-- Exported: " << outputFileName << std::endl;
        } else {
            for (const auto& object : meshedObjects) {
                const std::string outputFileName = object.definition.group +
                    ".tessellator." + object.extension + ".vtk";
                exportMeshToVTU(outputFolder / outputFileName, object.mesh);
                std::cout << "-- Exported: " << outputFileName << std::endl;
            }
        }

        if (!meshedObjects.empty() && readExportGridOption(inputFileData, std::nullopt)) {
            exportGridToVTU(
                outputFolder / (basename + ".tessellator.grid.vtk"),
                meshedObjects.front().mesh.grid);
            std::cout << "-- Exported grid: " << basename << ".tessellator.grid.vtk" << std::endl;
        }

        return EXIT_SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << std::endl << "-- Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}

}
