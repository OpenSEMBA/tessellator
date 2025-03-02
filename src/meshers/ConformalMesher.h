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
    
    static std::set<Cell> findNonConformalCells(const Mesh& mesh);
    static std::set<Cell> cellsWithMoreThanAVertexInsideEdge(const Mesh& mesh);
    static std::set<Cell> cellsWithMoreThanAPathPerFace(const Mesh& mesh);
private:
    Mesh inputMesh_;
    ConformalMesherOptions opts_;
};

}
