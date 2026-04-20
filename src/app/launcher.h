#pragma once

#include "types/Mesh.h"
#include <string>
#include <nlohmann/json.hpp>

namespace meshlib::app {

const std::string conformal_mesher ("conformal");
const std::string staircase_mesher ("staircase");

int launcher(int argc, const char* argv[]);
Grid parseGridFromJSON(const nlohmann::json& j);

}