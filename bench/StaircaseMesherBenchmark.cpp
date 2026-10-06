#include "meshers/StaircaseMesher.h"

#include "app/vtkIO.h"
#include "utils/GridTools.h"
#include "utils/MeshTools.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using meshlib::Mesh;
using meshlib::meshers::MesherPhaseTiming;
using meshlib::meshers::StaircaseMesher;
using meshlib::meshers::StaircaseMesherOptions;

constexpr double kGridMin = -100.0;
constexpr double kGridMax = 100.0;

struct CaseDefinition {
    std::string name;
    bool volume = false;
    int cells = 100;
    // Golden hexahedra count for the 50^3 volume sphere, matching
    // StaircaseMesherTest.fillsSphereAsSingleClosedUnitHexahedralVolume.
    std::size_t expectedHexahedra = 0;
};

struct TimingStats {
    double min = 0.0;
    double median = 0.0;
    double mean = 0.0;
};

struct CaseResult {
    CaseDefinition definition;
    TimingStats total;
    std::vector<MesherPhaseTiming> phases;
    std::size_t elements = 0;
    std::size_t coordinates = 0;
    std::size_t hexahedra = 0;
};

std::string formatSeconds(double seconds)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(4) << seconds;
    return stream.str();
}

TimingStats computeStats(std::vector<double> samples)
{
    TimingStats stats;
    if (samples.empty()) {
        return stats;
    }
    std::sort(samples.begin(), samples.end());
    stats.min = samples.front();
    stats.mean = std::accumulate(samples.begin(), samples.end(), 0.0) / samples.size();
    stats.median = samples[samples.size() / 2];
    return stats;
}

Mesh buildInputMesh(const Mesh& baseMesh, int cells)
{
    Mesh mesh = baseMesh;
    const std::size_t numberOfPlanes = static_cast<std::size_t>(cells) + 1;
    for (auto axis : {meshlib::X, meshlib::Y, meshlib::Z}) {
        mesh.grid[axis] = meshlib::utils::GridTools::linspace(kGridMin, kGridMax, numberOfPlanes);
    }
    return mesh;
}

StaircaseMesherOptions buildOptions(const CaseDefinition& definition)
{
    StaircaseMesherOptions options;
    if (definition.volume) {
        options.volumeGroups.insert(0);
        options.splitHexahedra = true;
    }
    return options;
}

CaseDefinition parseCase(const std::string& name, int cells)
{
    CaseDefinition definition;
    definition.name = name;
    definition.cells = cells;
    if (name == "surface") {
        definition.volume = false;
    } else if (name == "volume") {
        definition.volume = true;
        if (cells == 50) {
            definition.expectedHexahedra = 7967;
        }
    } else {
        throw std::runtime_error("Unknown case '" + name + "'. Use 'surface' or 'volume'.");
    }
    return definition;
}

std::vector<int> parseCells(const std::string& argument)
{
    std::vector<int> cells;
    std::stringstream stream(argument);
    std::string token;
    while (std::getline(stream, token, ',')) {
        cells.push_back(std::stoi(token));
    }
    return cells;
}

void printUsage()
{
    std::cout
        << "Usage: tessellator_benchmarks [options]\n"
        << "\n"
        << "Options:\n"
        << "  --case <surface|volume|all>  Cases to run (default: all)\n"
        << "  --cells <n[,n,...]>          Grid cells per axis per case. Defaults:\n"
        << "                               surface 100,200,400 / volume 50,100,200,300\n"
        << "  --repeats <n>                Timed repetitions per case (default: 3)\n"
        << "  --warmup <n>                 Warmup repetitions per case (default: 1)\n"
        << "  --json <file>                Write results as JSON to the given path\n"
        << "  --help                       Show this message\n";
}

CaseResult runCase(
    const Mesh& baseMesh,
    const CaseDefinition& definition,
    int warmup,
    int repeats)
{
    CaseResult result;
    result.definition = definition;

    const Mesh inputMesh = buildInputMesh(baseMesh, definition.cells);
    const StaircaseMesherOptions options = buildOptions(definition);

    for (int i = 0; i < warmup; ++i) {
        StaircaseMesher mesher(inputMesh, options);
        (void)mesher.mesh();
    }

    std::vector<double> samples;
    samples.reserve(repeats);
    std::vector<std::vector<MesherPhaseTiming>> phaseSamples;
    phaseSamples.reserve(repeats);
    std::size_t expectedElements = 0;
    std::size_t expectedCoordinates = 0;

    for (int i = 0; i < repeats; ++i) {
        const auto start = std::chrono::steady_clock::now();
        StaircaseMesher mesher(inputMesh, options);
        const auto end = std::chrono::steady_clock::now();
        samples.push_back(std::chrono::duration<double>(end - start).count());
        phaseSamples.push_back(mesher.getTimings());

        const Mesh output = mesher.mesh();
        const std::size_t elements = output.countElems();
        const std::size_t coordinates = output.coordinates.size();
        const std::size_t hexahedra = meshlib::utils::meshTools::countMeshElementsIf(
            output, meshlib::utils::meshTools::isHexahedron);

        if (i == 0) {
            expectedElements = elements;
            expectedCoordinates = coordinates;
            result.elements = elements;
            result.coordinates = coordinates;
            result.hexahedra = hexahedra;
        } else if (elements != expectedElements || coordinates != expectedCoordinates) {
            throw std::runtime_error(
                "Non-deterministic output for case '" + definition.name + "': run " +
                std::to_string(i) + " produced " + std::to_string(elements) +
                " elements / " + std::to_string(coordinates) + " coordinates, expected " +
                std::to_string(expectedElements) + " / " + std::to_string(expectedCoordinates));
        }
    }

    if (definition.expectedHexahedra != 0) {
        if (result.hexahedra != definition.expectedHexahedra) {
            throw std::runtime_error(
                "Golden check failed for case '" + definition.name + "': expected " +
                std::to_string(definition.expectedHexahedra) + " hexahedra, got " +
                std::to_string(result.hexahedra));
        }
        if (result.hexahedra != result.elements) {
            throw std::runtime_error(
                "Golden check failed for case '" + definition.name +
                "': expected only hexahedra in the output, got " +
                std::to_string(result.elements) + " elements");
        }
    }

    result.total = computeStats(samples);

    std::vector<std::string> phaseNames;
    for (const auto& phase : phaseSamples.front()) {
        phaseNames.push_back(phase.phase);
    }
    for (std::size_t p = 0; p < phaseNames.size(); ++p) {
        std::vector<double> phaseTimes;
        phaseTimes.reserve(phaseSamples.size());
        for (const auto& sample : phaseSamples) {
            if (p < sample.size() && sample[p].phase == phaseNames[p]) {
                phaseTimes.push_back(sample[p].seconds);
            }
        }
        const TimingStats stats = computeStats(phaseTimes);
        result.phases.push_back({phaseNames[p], stats.median});
    }

    return result;
}

void printResults(const std::vector<CaseResult>& results)
{
    std::cout << "\n"
              << std::left << std::setw(28) << "case"
              << std::right << std::setw(10) << "min (s)"
              << std::setw(10) << "median"
              << std::setw(10) << "mean"
              << std::setw(12) << "elements"
              << std::setw(12) << "coords" << "\n";
    std::cout << std::string(82, '-') << "\n";

    for (const auto& result : results) {
        std::cout << std::left << std::setw(28) << result.definition.name
                  << std::right << std::setw(10) << formatSeconds(result.total.min)
                  << std::setw(10) << formatSeconds(result.total.median)
                  << std::setw(10) << formatSeconds(result.total.mean)
                  << std::setw(12) << result.elements
                  << std::setw(12) << result.coordinates << "\n";
        for (const auto& phase : result.phases) {
            std::cout << "  " << std::left << std::setw(26) << phase.phase
                      << std::right << std::setw(10) << formatSeconds(phase.seconds) << "\n";
        }
        std::cout << "\n";
    }
}

void writeJson(const std::string& path, const std::vector<CaseResult>& results)
{
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("Could not open JSON output file: " + path);
    }
    out << std::fixed << std::setprecision(6);
    out << "{\n  \"cases\": [\n";
    for (std::size_t i = 0; i < results.size(); ++i) {
        const auto& result = results[i];
        out << "    {\n";
        out << "      \"name\": \"" << result.definition.name << "\",\n";
        out << "      \"cells\": " << result.definition.cells << ",\n";
        out << "      \"volume\": " << (result.definition.volume ? "true" : "false") << ",\n";
        out << "      \"elements\": " << result.elements << ",\n";
        out << "      \"coordinates\": " << result.coordinates << ",\n";
        out << "      \"min_seconds\": " << result.total.min << ",\n";
        out << "      \"median_seconds\": " << result.total.median << ",\n";
        out << "      \"mean_seconds\": " << result.total.mean << ",\n";
        out << "      \"phases\": [\n";
        for (std::size_t p = 0; p < result.phases.size(); ++p) {
            out << "        {\"phase\": \"" << result.phases[p].phase
                << "\", \"median_seconds\": " << result.phases[p].seconds << "}";
            out << (p + 1 < result.phases.size() ? ",\n" : "\n");
        }
        out << "      ]\n";
        out << "    }" << (i + 1 < results.size() ? "," : "") << "\n";
    }
    out << "  ]\n}\n";
}

} // namespace

int main(int argc, char* argv[])
{
    std::string caseSelection = "all";
    std::vector<int> cellsOverride;
    int repeats = 3;
    int warmup = 1;
    std::string jsonPath;

    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        auto requireValue = [&](const std::string& option) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for " + option);
            }
            return argv[++i];
        };

        if (argument == "--help" || argument == "-h") {
            printUsage();
            return EXIT_SUCCESS;
        } else if (argument == "--case") {
            caseSelection = requireValue(argument);
        } else if (argument == "--cells") {
            cellsOverride = parseCells(requireValue(argument));
        } else if (argument == "--repeats") {
            repeats = std::stoi(requireValue(argument));
        } else if (argument == "--warmup") {
            warmup = std::stoi(requireValue(argument));
        } else if (argument == "--json") {
            jsonPath = requireValue(argument);
        } else {
            std::cerr << "Unknown option: " << argument << "\n";
            printUsage();
            return EXIT_FAILURE;
        }
    }

    if (repeats < 1) {
        std::cerr << "--repeats must be at least 1\n";
        return EXIT_FAILURE;
    }
    if (warmup < 0) {
        std::cerr << "--warmup must be non-negative\n";
        return EXIT_FAILURE;
    }

    try {
        const std::filesystem::path spherePath =
            std::filesystem::path(TESSELLATOR_DATA_DIR) / "cases/sphere/sphere.stl";
        std::cout << "-- Reading sphere: " << spherePath << std::endl;
        const Mesh baseMesh = meshlib::vtkIO::readInputMesh(spherePath);

        std::vector<CaseDefinition> definitions;
        const bool runSurface = caseSelection == "all" || caseSelection == "surface";
        const bool runVolume = caseSelection == "all" || caseSelection == "volume";
        if (!runSurface && !runVolume) {
            std::cerr << "Unknown case selection: " << caseSelection << "\n";
            return EXIT_FAILURE;
        }

        if (runSurface) {
            const std::vector<int> cells = cellsOverride.empty()
                ? std::vector<int>{100, 200, 400}
                : cellsOverride;
            for (const int n : cells) {
                definitions.push_back(parseCase("surface", n));
            }
        }
        if (runVolume) {
            const std::vector<int> cells = cellsOverride.empty()
                ? std::vector<int>{50, 100, 200, 300}
                : cellsOverride;
            for (const int n : cells) {
                definitions.push_back(parseCase("volume", n));
            }
        }

        std::cout << "-- repeats: " << repeats << ", warmup: " << warmup << std::endl;

        std::vector<CaseResult> results;
        results.reserve(definitions.size());
        for (const auto& definition : definitions) {
            std::cout << "-- Running " << definition.name << " " << definition.cells
                      << "^3" << std::endl;
            results.push_back(runCase(baseMesh, definition, warmup, repeats));
        }

        printResults(results);

        if (!jsonPath.empty()) {
            writeJson(jsonPath, results);
            std::cout << "-- JSON written to " << jsonPath << std::endl;
        }
    } catch (const std::exception& exception) {
        std::cerr << "Benchmark failed: " << exception.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
