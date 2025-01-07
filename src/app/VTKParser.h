#pragma once

#include "types/Mesh.h"

namespace meshlib
{
    Mesh readVTK(const std::string &fileName);
}