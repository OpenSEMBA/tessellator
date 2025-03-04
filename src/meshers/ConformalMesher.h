#pragma once

#include "types/Mesh.h"
#include "utils/Types.h"
#include "ConformalMesherOptions.h"
#include "MesherBase.h"

namespace meshlib::meshers {

class ConformalMesher : public MesherBase {
public:
    ConformalMesher(const Mesh& in, ConformalMesherOptions opts = ConformalMesherOptions()) : 
        inputMesh_{ in }, 
        opts_{opts},
        MesherBase(in)
    {};
    virtual ~ConformalMesher() = default;
    
    Mesh mesh() const;
    
    static std::set<Cell> findNonConformalCells(const Mesh& mesh);
    static std::set<Cell> cellsWithMoreThanAVertexInsideEdge(const Mesh& mesh);
    static std::set<Cell> cellsWithMoreThanAPathPerFace(const Mesh& mesh);
    static std::set<Cell> cellsWithInteriorDisconnectedPatches(const Mesh& mesh);
    static std::set<Cell> cellsWithAVertexInAnEdgeForbiddenRegion(const Mesh& mesh);
private:
    Mesh inputMesh_;
    ConformalMesherOptions opts_;

    void process(Mesh& mesh) const {};
};

}
