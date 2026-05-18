#include "launcher.h"
#include "vtkIO.h"

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
#include <algorithm>
#include <optional>

namespace meshlib::app {

using namespace vtkIO;


namespace po = boost::program_options;

Grid parseGridFromJSON(const nlohmann::json &j)
{
    if (j.find("planes") != j.end()) {
        return j["planes"];
    }
    else {
        std::array<int, 3> nCells = {
        j["numberOfCells"][0],
        j["numberOfCells"][1],
        j["numberOfCells"][2]
        };
        std::array<double, 3> min, max;
        min = j["boundingBox"][0];
        max = j["boundingBox"][1];

        return {
            utils::GridTools::linspace(min[0], max[0], nCells[0] + 1),
            utils::GridTools::linspace(min[1], max[1], nCells[1] + 1),
            utils::GridTools::linspace(min[2], max[2], nCells[2] + 1)
        };
    }
}

std::vector<ObjectDefinition> readObjectsFromJSON(const std::string& fn)
{
    nlohmann::json j;
    {
        std::ifstream i(fn);
        i >> j;
    }

    std::vector<ObjectDefinition> objects;

    if (j.contains("objects")) {
        for (const auto& obj : j["objects"]) {
            ObjectDefinition objDef;
            objDef.filename = obj["filename"].get<std::string>();
            objDef.group = obj.value("group", std::filesystem::path(objDef.filename).stem().string());
            if (obj.contains("mesher")) {
                objDef.mesherOverride = obj["mesher"];
            }
            objects.push_back(objDef);
        }
    } else if (j.contains("object")) {
        ObjectDefinition objDef;
        objDef.filename = j["object"]["filename"].get<std::string>();
        objDef.group = std::filesystem::path(objDef.filename).stem().string();
        if (j.contains("mesher")) {
            objDef.mesherOverride = j["mesher"];
        }
        objects.push_back(objDef);
    } else {
        throw std::runtime_error("No objects defined in input file");
    }

    return objects;
}

Mesh readMesh(const std::string& fn, const ObjectDefinition& objDef)
{
    nlohmann::json j;
    {
        std::ifstream i(fn);
        i >> j;
    }

    std::filesystem::path caseFolder = std::filesystem::path(fn).parent_path();
    std::filesystem::path meshObjectPath = caseFolder / objDef.filename;

    std::cout << "-- Reading mesh groups from: " << meshObjectPath;
    Mesh res = vtkIO::readInputMesh(meshObjectPath);
    std::cout << "....... [OK]" << std::endl;

    std::cout << "-- Reading grid from input file";
    res.grid = parseGridFromJSON(j["grid"]);
    std::cout << "....... [OK]" << std::endl;

    if (res.groups.empty()) {
        res.groups.push_back(Group{objDef.group, {}});
    } else {
        res = utils::meshTools::extractGroupsByName(res, {objDef.group});
        if (res.groups.empty()) {
            res.groups.push_back(Group{objDef.group, {}});
        } else {
            res.groups[0].name = objDef.group;
        }
    }

    return res;
}


std::string readMesherType(const std::string& fn, const std::optional<nlohmann::json>& override)
{
    nlohmann::json j;
    {
        std::ifstream i(fn);
        i >> j;
    }
    
    nlohmann::json mesherConfig;
    if (override.has_value()) {
        mesherConfig = *override;
    } else if (j.contains("mesher")) {
        mesherConfig = j["mesher"];
    } else {
        return meshlib::app::staircase_mesher;
    }
    
    if (mesherConfig.contains("type")) {
        return mesherConfig["type"];
    } else {
        return meshlib::app::staircase_mesher;
    }
}

std::string readExtension(const std::string& fn, const std::optional<nlohmann::json>& override)
{
    auto mesherType = readMesherType(fn, override);
    if (mesherType == meshlib::app::staircase_mesher) {
        return "str";
    } else if (mesherType == meshlib::app::conformal_mesher) {
        return "cmsh";
    } else {
        throw std::runtime_error("Unsupported mesher type");
    }
}

meshlib::meshers::StaircaseMesherOptions readStaircaseMesherOptions(const std::string &fn)
{
    nlohmann::json j;
    {
        std::ifstream i(fn);
        i >> j;
    }
    meshlib::meshers::StaircaseMesherOptions res;
    if (j["object"].contains("volume")) {
        res.isVolume = j["object"]["volume"];
    }
    if (j["mesher"].contains("options") && 
        j["mesher"]["options"].contains("compress")) {
        res.compress = j["mesher"]["options"]["compress"];
    }

    return res;
}

meshlib::meshers::ConformalMesherOptions readConformalMesherOptions(const std::string& fn, const std::optional<nlohmann::json>& override)
{
    nlohmann::json j;
    {
        std::ifstream i(fn);
        i >> j;
    }
    
    nlohmann::json mesherConfig;
    if (override.has_value()) {
        mesherConfig = *override;
    } else if (j.contains("mesher")) {
        mesherConfig = j["mesher"];
    }
    
    meshlib::meshers::ConformalMesherOptions res;
    if (mesherConfig.contains("options")) {
        res.snapperOptions.edgePoints = mesherConfig["options"]["edgePoints"];
        res.snapperOptions.forbiddenLength = mesherConfig["options"]["forbiddenLength"];
    }
    return res;
}

bool readStaircaseMesherCompressOption(const std::string& fn, const std::optional<nlohmann::json>& override)
{
    nlohmann::json j;
    {
        std::ifstream i(fn);
        i >> j;
    }
    
    nlohmann::json mesherConfig;
    if (override.has_value()) {
        mesherConfig = *override;
    } else if (j.contains("mesher")) {
        mesherConfig = j["mesher"];
    }
    
    if (mesherConfig.contains("options") && 
        mesherConfig["options"].contains("compress")) {
        return mesherConfig["options"]["compress"];
    }
    return false;
}

bool readExportGridOption(const std::string& fn, const std::optional<nlohmann::json>& override)
{
    nlohmann::json j;
    {
        std::ifstream i(fn);
        i >> j;
    }
    
    nlohmann::json mesherConfig;
    if (override.has_value()) {
        mesherConfig = *override;
    } else if (j.contains("mesher")) {
        mesherConfig = j["mesher"];
    }
    
    if (mesherConfig.contains("options") && 
        mesherConfig["options"].contains("exportGrid")) {
        return mesherConfig["options"]["exportGrid"];
    }
    return true;
}

std::unique_ptr<meshlib::meshers::MesherBase> buildMesher(const Mesh& in, const std::string& fn, const std::optional<nlohmann::json>& override)
{
    auto mesherType = readMesherType(fn, override);
    if (mesherType == meshlib::app::staircase_mesher) {
        auto staircasedOptions = readStaircaseMesherOptions(fn);
        staircasedOptions.compress = readStaircaseMesherCompressOption(fn, override);
        return std::make_unique<meshlib::meshers::StaircaseMesher>(meshlib::meshers::StaircaseMesher{in, 4, staircasedOptions});
    } else if (mesherType == meshlib::app::conformal_mesher) {
        return std::make_unique<meshlib::meshers::ConformalMesher>(meshlib::meshers::ConformalMesher{in, readConformalMesherOptions(fn, override)});
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

    std::string inputFilename = vm["input"].as<std::string>();
    std::cout << "-- Input file is: " << inputFilename << std::endl;

    std::vector<ObjectDefinition> objects = readObjectsFromJSON(inputFilename);
    std::filesystem::path outputFolder = getFolder(inputFilename);
    auto basename = getBasename(inputFilename);

    Mesh firstMesh;
    bool first = true;

    for (const auto& objDef : objects) {
        std::cout << "\n-- Processing object: " << objDef.filename << " (group: " << objDef.group << ")" << std::endl;

        Mesh mesh = readMesh(inputFilename, objDef);

        auto mesher = buildMesher(mesh, inputFilename, objDef.mesherOverride);
        Mesh resultMesh = mesher->mesh();

        if (first) {
            firstMesh = resultMesh;
            first = false;
        }

        auto extension = readExtension(inputFilename, objDef.mesherOverride);
        std::string outputFilename = objDef.group + ".tessellator." + extension + ".vtk";
        exportMeshToVTU(outputFolder / outputFilename, resultMesh);
        std::cout << "-- Exported: " << outputFilename << std::endl;
    }

    if (!first && readExportGridOption(inputFilename, std::nullopt)) {
        exportGridToVTU(outputFolder / (basename + ".tessellator.grid.vtk"), firstMesh.grid);
        std::cout << "-- Exported grid: " << basename << ".tessellator.grid.vtk" << std::endl;
    }

    return EXIT_SUCCESS;
}

}
