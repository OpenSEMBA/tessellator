#pragma once

#include <nlohmann/json.hpp>
#include <vector>

#include "MeshedObject.h"
#include "meshers/StaircaseMesherOptions.h"

namespace meshlib {
namespace app {

// Parses the staircase mesher's embedded options.
void readEmbeddedOptions(
    const nlohmann::json& options,
    meshlib::meshers::StaircaseMesherOptions& result);

// Applies embedded processing to already-meshed objects. This includes host
// surface discovery, embedding/perimeter validation and output conversion.
void applyEmbeddedModes(std::vector<MeshedObject>& objects);

}
}
