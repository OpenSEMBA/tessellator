#pragma once

#include "types/Mesh.h"

#include <filesystem>

namespace meshlib::vtkIO
{
    std::string getBasename(const std::filesystem::path& fn);
    std::filesystem::path getFolder(const std::filesystem::path& fn);
    
    Mesh readInputMesh(const std::filesystem::path& fileName);

    void exportMeshToVTP(const std::filesystem::path& fn, const Mesh& mesh);
    void exportMeshToVTU(const std::filesystem::path& fn, const Mesh& mesh);
    void exportGridToVTU(const std::filesystem::path& fn, const Grid& grid);
    
}