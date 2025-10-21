#pragma once

#include "types/Mesh.h"
#include <string>

namespace meshlib::app {

const std::string conformal_mesher ("conformal");
const std::string structured_mesher ("structured");

int launcher(int argc, const char* argv[]);

}