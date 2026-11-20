#pragma once

#include "types/Mesh.h"
#include "meshers/MesherBase.h"
#include <string>
#include <optional>
#include <memory>
#include <nlohmann/json.hpp>

namespace meshlib::app {

const std::string conformal_mesher ("conformal");
const std::string staircase_mesher ("staircase");

struct ObjectDefinition {
    std::string filename;
    std::string group;
    bool isVolume = false;
    bool ghost = false;
    std::optional<nlohmann::json> mesherOverride;
};

int launcher(int argc, const char* argv[]);
Grid parseGridFromJSON(const nlohmann::json& fileData);
std::vector<ObjectDefinition> readObjectsFromJSON(const nlohmann::json& fileData);
bool readSingleFileOutputOption(const nlohmann::json& fileData);
Mesh readMesh(const std::string& fn, const ObjectDefinition& objDef);
std::unique_ptr<meshlib::meshers::MesherBase> buildMesher(const Mesh& in, const nlohmann::json& fileData, const ObjectDefinition& objDef);

}
