#include "launcher.h"
#include "vtkIO.h"

#include "meshers/MesherBase.h"
#include "meshers/StaircaseMesher.h"
#include "meshers/ConformalMesher.h"
#include "utils/GridTools.h"
#include "utils/MeshTools.h"

#include <boost/program_options.hpp>
#include <nlohmann/json.hpp>

#include <iostream>
#include <filesystem>
#include <fstream>
#include <array>
#include <memory>
#include <optional>

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
        if (fileData.contains("mesher")) {
            objDef.mesherOverride = fileData["mesher"];
        }
        objects.push_back(objDef);
    } else {
        throw std::runtime_error("No objects defined in input file");
    }

    return objects;
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
    if (isVolume){
        res.volumeGroups.insert(0);
    }
    if (mesherConfig.contains("options") && 
        mesherConfig["options"].contains("compress")) {
        res.compress = mesherConfig["options"]["compress"];
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
        res.snapperOptions.edgePoints = mesherConfig["options"]["edgePoints"];
        res.snapperOptions.forbiddenLength = mesherConfig["options"]["forbiddenLength"];
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
            4,
            readStaircaseMesherOptions(fileData, objDef.isVolume, objDef.mesherOverride)
        });
    } else if (mesherType == meshlib::app::conformal_mesher) {
        return std::make_unique<meshlib::meshers::ConformalMesher>(meshlib::meshers::ConformalMesher{in, readConformalMesherOptions(fileData, objDef.isVolume, objDef.mesherOverride)});
    } else {
        throw std::runtime_error("Unsupported mesher type");
    }
}

int launcher(int argc, const char* argv[])
{
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

    nlohmann::json inputFileData;
    {
        std::ifstream i(inputFileName);
        i >> inputFileData;
    }

    std::vector<ObjectDefinition> objects = readObjectsFromJSON(inputFileData);
    std::filesystem::path outputFolder = getFolder(inputFileName);
    auto basename = getBasename(inputFileName);

    Mesh firstMesh;
    bool first = true;

    for (const auto& objDef : objects) {
        std::cout << "\n-- Processing object: " << objDef.filename << " (group: " << objDef.group << ")" << std::endl;

        Mesh mesh = readMesh(inputFileData, outputFolder, objDef);

        auto mesher = buildMesher(mesh, inputFileData, objDef);
        Mesh resultMesh = mesher->mesh();

        if (first) {
            firstMesh = resultMesh;
            first = false;
        }

        auto extension = readExtension(inputFileData, objDef.mesherOverride);
        std::string outputFileName = objDef.group + ".tessellator." + extension + ".vtk";
        exportMeshToVTU(outputFolder / outputFileName, resultMesh);
        std::cout << "-- Exported: " << outputFileName << std::endl;
    }

    if (!first && readExportGridOption(inputFileData, std::nullopt)) {
        exportGridToVTU(outputFolder / (basename + ".tessellator.grid.vtk"), firstMesh.grid);
        std::cout << "-- Exported grid: " << basename << ".tessellator.grid.vtk" << std::endl;
    }

    return EXIT_SUCCESS;
}

}
