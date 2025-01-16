#pragma once

#include "types/Mesh.h"

namespace meshlib::vtkIO
{
    Mesh readMesh(const std::string &fileName);

    void exportMeshToVTP(const std::string& fn, const Mesh& mesh);
    void exportGridToVTP(const std::string& fn, const Grid& grid);
    
}