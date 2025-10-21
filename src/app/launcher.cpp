#include "launcher.h"
#include "vtkIO.h"

#include "meshers/StructuredMesher.h"
#include "meshers/ConformalMesher.h"
#include "utils/GridTools.h"

#include <boost/program_options.hpp>
#include <nlohmann/json.hpp>

#include <iostream>
#include <filesystem>
#include <fstream>
#include <array>
#include <memory>

namespace meshlib::app {

using namespace vtkIO;


namespace po = boost::program_options;

Grid parseGridFromJSON(const nlohmann::json &j)
{
    std::array<int,3> nCells = {
        j["numberOfCells"][0],
        j["numberOfCells"][1],
        j["numberOfCells"][2]
    };
    std::array<double,3> min, max;
    min = j["boundingBox"][0];
    max = j["boundingBox"][1];

    return {
        utils::GridTools::linspace(min[0], max[0], nCells[0]+1),
        utils::GridTools::linspace(min[1], max[1], nCells[1]+1),
        utils::GridTools::linspace(min[2], max[2], nCells[2]+1)
    };
}

Mesh readMesh(const std::string &fn)
{
    nlohmann::json j;

    {
        std::ifstream i(fn);
        i >> j;
    }

    std::filesystem::path caseFolder = std::filesystem::path(fn).parent_path();
    std::filesystem::path objPathFromInput = j["object"]["filename"];
    std::filesystem::path meshObjectPath = caseFolder / objPathFromInput;

    std::cout << "-- Reading mesh groups from: " << meshObjectPath;
    Mesh res = vtkIO::readInputMesh(meshObjectPath);
    std::cout << "....... [OK]" << std::endl;

    std::cout << "-- Reading grid from input file";
    res.grid = parseGridFromJSON(j["grid"]);
    std::cout << "....... [OK]" << std::endl;

    return res;
}

std::string readMesherType(const std::string &fn)
{
    nlohmann::json j;
    {
        std::ifstream i(fn);
        i >> j;
    }
    if (j["mesher"].contains("type")) {
        return j["mesher"]["type"];
    } else {
        return meshlib::app::structured_mesher;
    }
}

meshlib::meshers::ConformalMesherOptions readConformalMesherOptions(const std::string &fn)
{
    nlohmann::json j;
    {
        std::ifstream i(fn);
        i >> j;
    }
    meshlib::meshers::ConformalMesherOptions res;
    if (j["mesher"].contains("options")) {
        res.snapperOptions.edgePoints = j["mesher"]["options"]["edgePoints"];
        res.snapperOptions.forbiddenLength = j["mesher"]["options"]["forbiddenLength"];
    } else {
        res.snapperOptions.edgePoints = 4;
        res.snapperOptions.forbiddenLength = 0;
    }
    return res;
}
std::unique_ptr<meshlib::meshers::MesherBase> buildMesher(const Mesh &in, const std::string &fn)
{
    auto mesherType = readMesherType(fn);
    if (mesherType == meshlib::app::structured_mesher) {
        return std::make_unique<meshlib::meshers::StructuredMesher>(meshlib::meshers::StructuredMesher{in});
    } else if (mesherType == meshlib::app::conformal_mesher) {
        return std::make_unique<meshlib::meshers::ConformalMesher>(meshlib::meshers::ConformalMesher{in, readConformalMesherOptions(fn)});
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

    // Input
    std::string inputFilename = vm["input"].as<std::string>();
    std::cout << "-- Input file is: " << inputFilename << std::endl;

    Mesh mesh = readMesh(inputFilename);


    // Mesh
    auto mesher = buildMesher(mesh, inputFilename);
    Mesh resultMesh = mesher->mesh();

    std::filesystem::path outputFolder = getFolder(inputFilename);
    auto basename = getBasename(inputFilename);
    exportMeshToVTU(outputFolder / (basename + ".tessellator.str.vtk"), resultMesh);
    exportGridToVTU(outputFolder / (basename + ".tessellator.grid.vtk"), resultMesh.grid);

    return EXIT_SUCCESS;
}

}