#pragma once

#include "types/Mesh.h"
#include "utils/Types.h"
#include "ConformalMesherOptions.h"

namespace meshlib::meshers {

class ConformalMesher {
public:
    ConformalMesher(const Mesh& in, ConformalMesherOptions opts = ConformalMesherOptions()) : 
        inputMesh_{ in }, 
        opts_{opts} 
    {};
    virtual ~ConformalMesher() = default;
    
    Mesh mesh() const;
    std::set<Cell> findNonConformalCells(const Mesh& mesh) const;

private:
    Mesh inputMesh_;
    ConformalMesherOptions opts_;
};

}
