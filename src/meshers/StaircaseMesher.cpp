#include "StaircaseMesher.h"

#include <iostream>
#include <unordered_map>
#include <cstdint>
#include <tuple>

#include "core/Slicer.h"
#include "core/Collapser.h"
#include "core/Staircaser.h"
#include "core/Compressor.h"
#include "core/VolumeFiller.h"
#include "core/VolumeShellExtractor.h"

#include "utils/RedundancyCleaner.h"
#include "utils/MeshTools.h"
#include "utils/GridTools.h"

namespace meshlib::meshers {

using namespace utils;
using namespace core;
using namespace meshTools;

std::vector<std::string> getGroupNames(const Groups& groups);
void copyGroupNames(Mesh& mesh, const std::vector<std::string>& names);

/*
  tessellator was providing duplicated nodes for quadrilaterals and hexahedra,so we must collapse them to act like old zmesher
  must remove in coordinates the duplicated nodes (same x,y,z) and renumber the connectivities of the elements in groups vertices
*/
// Custom hash for a 3D grid cell coordinate tuple
struct GridKey {
  int64_t x;
  int64_t y;
  int64_t z;

  bool operator==(const GridKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridKeyHash {
  std::size_t operator()(const GridKey& k) const noexcept {
    // Spatial hashing / bit-mixing (prime multipliers for 3D coordinates)
    std::size_t h1=std::hash<int64_t>{}(k.x);
    std::size_t h2=std::hash<int64_t>{}(k.y);
    std::size_t h3=std::hash<int64_t>{}(k.z);
    return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2)) ^ (h3 * 0x85ebca6b);
  }
};

bool StaircaseMesher::collapse_nodes(Mesh& z_output_mesh,const double tolerance) {
  bool fail=false;

  const size_t num_coordinates=z_output_mesh.coordinates.size();
  if(num_coordinates <= 1) {
    return fail;
  }

  const double inv_tol=1.0 / tolerance;

  // Lambda to map continuous 3D floating-point coords to integer grid indices
  auto make_key=[inv_tol](const Coordinate& coord) -> GridKey {
    return GridKey{
      static_cast<int64_t>(std::round(coord[0] * inv_tol)),
      static_cast<int64_t>(std::round(coord[1] * inv_tol)),
      static_cast<int64_t>(std::round(coord[2] * inv_tol))
    };
    };

  std::vector<CoordinateId> old_to_new_coordinate_id(num_coordinates);

  // Unordered map for O(1) expected average lookup
  std::unordered_map<GridKey,CoordinateId,GridKeyHash> unique_coordinates;
  unique_coordinates.reserve(num_coordinates);

  std::vector<Coordinate> collapsed_coordinates;
  collapsed_coordinates.reserve(num_coordinates);

  for(size_t i_coordinate=0; i_coordinate < num_coordinates; ++i_coordinate) {
    const auto& coordinate=z_output_mesh.coordinates[i_coordinate];
    GridKey key=make_key(coordinate);

    // Single lookup and insertion step using try_emplace (C++17)
    const CoordinateId next_id=static_cast<CoordinateId>(collapsed_coordinates.size());
    auto [it,inserted]=unique_coordinates.try_emplace(key,next_id);

    if(inserted) {
      old_to_new_coordinate_id[i_coordinate]=next_id;
      collapsed_coordinates.push_back(coordinate);
    } else {
      old_to_new_coordinate_id[i_coordinate]=it->second;
    }
  }

  // Early return if no duplicates were found and collapsed
  if(collapsed_coordinates.size() == num_coordinates) {
    return fail;
  }

  // Update element connectivity referencing new coordinate IDs
  for(auto& group : z_output_mesh.groups) {
    for(auto& element : group.elements) {
      for(auto& vertex_id : element.vertices) {
        const auto old_id=static_cast<size_t>(vertex_id);
        if(old_id >= num_coordinates) {
          fail=true;
        } else {
          vertex_id=old_to_new_coordinate_id[old_id];
        }
      }
    }
  }

  z_output_mesh.coordinates.swap(collapsed_coordinates);

  return fail;
}

StaircaseMesher::StaircaseMesher(const Mesh& inputMesh, StaircaseMesherOptions opts) :
    MesherBase(inputMesh),
    opts_(opts)
{
    log("Preparing surfaces.");
    surfaceMesh_ = MesherBase::buildSurfaceMesh(inputMesh, opts_.volumeGroups);
    log("Processing surface mesh.");
    process(surfaceMesh_);

    log("Preparing volumes");
    volumeMesh_ = MesherBase::buildVolumeMesh(inputMesh, opts_.volumeGroups);
    if (!volumeMesh_.emptyOfElements()) {
        volumeMesh_ = VolumeShellExtractor(volumeMesh_).getMesh();

        log("Processing volume shell.");
        process(volumeMesh_, false);
        log("Filling volume shell with hexahedra.");
        volumeMesh_ = VolumeFiller(volumeMesh_, opts_.splitHexahedra).getMesh();
        logNumberOfHexahedra(countMeshElementsIf(volumeMesh_, isHexahedron));
    }

    mergeMesh(surfaceMesh_, volumeMesh_);
    RedundancyCleaner::cleanCoords(surfaceMesh_);

    log("collapsing nodes");
    collapse_nodes(surfaceMesh_,1e-8);
    
    log("Mesh built succesfully.", 1);
}

Mesh StaircaseMesher::buildSurfaceMesh(const Mesh& inputMesh, const Mesh & volumeSurface)
{
    auto resultMesh = buildMeshFilteringElements(inputMesh, isNotTetrahedron);
    mergeMesh(resultMesh, volumeSurface);
    return resultMesh;
}

std::vector<std::string> getGroupNames(const Groups& groups){
    std::vector<std::string> names;
    names.reserve(groups.size());
    for (auto gId{0}; gId < groups.size(); ++gId) {        
        names.push_back(groups[gId].name);
    }
    return names;
}

void copyGroupNames(Mesh& m, const std::vector<std::string>& names){
    for (auto gId{0}; gId < m.groups.size(); ++gId) {        
        m.groups[gId].name = names[gId];
    }
}

static Mesh toAbsolute(const Mesh& m) 
{
    auto r{ m };
    r.coordinates = 
        utils::GridTools{ m.grid }.relativeToAbsolute(m.coordinates);
    return r;
}

void StaircaseMesher::process(Mesh& mesh) const
{
    process(mesh, opts_.compress);
}

void StaircaseMesher::process(Mesh& mesh, bool compress) const
{
    const auto groupNames = getGroupNames(mesh.groups);
    const auto slicingGrid{ buildSlicingGrid(originalGrid_, enlargedGrid_) };
    if (mesh.countElems() == 0) {
        // mesh.grid = slicingGrid;
        return;
    }

    auto dimensions = getHighestDimensionByGroup(mesh);

    log("Slicing.", 1);
    mesh.grid = slicingGrid;
    mesh = Slicer{ mesh, dimensions }.getMesh();
    
    logNumberOfTriangles(countMeshElementsIf(mesh, isTriangle));

    log("Collapsing.", 1);
    mesh = Collapser(mesh, opts_.decimalPlacesInCollapser, dimensions).getMesh();

    logNumberOfTriangles(countMeshElementsIf(mesh, isTriangle));
    
    log("Staircasing.", 1);
    mesh = Staircaser(mesh).getMesh();

    logNumberOfQuads(countMeshElementsIf(mesh, isQuad));
    logNumberOfLines(countMeshElementsIf(mesh, isLine));

    log("Removing repeated and overlapping elements.", 1);   
    RedundancyCleaner::removeOverlappedElementsByDimension(mesh, dimensions);

    logNumberOfQuads(countMeshElementsIf(mesh, isQuad));
    logNumberOfLines(countMeshElementsIf(mesh, isLine));

    if (compress) {
        log("Compressing surfaces.", 1);
        std::size_t beforeQuads = countMeshElementsIf(mesh, isQuad);
        std::size_t merged = Compressor::compressSurfacesInMesh(mesh);
        std::size_t afterQuads = countMeshElementsIf(mesh, isQuad);
        log("Compressed " + std::to_string(beforeQuads) + 
            " -> " + std::to_string(afterQuads) + 
            " quads (merged " + std::to_string(merged) + " surfaces)", 1);
        
        log("Compressing lines.", 1);
        std::size_t beforeLines = countMeshElementsIf(mesh, isLine);
        merged = Compressor::compressLinesInMesh(mesh, dimensions);
        std::size_t afterLines = countMeshElementsIf(mesh, isLine);
        log("Compressed " + std::to_string(beforeLines) + 
            " -> " + std::to_string(afterLines) + 
            " lines (merged " + std::to_string(merged) + " segments)", 1);
    }
    
    log("Recovering original grid size.", 1);
    reduceGrid(mesh, originalGrid_);

    log("Converting relative to absolute coordinates.", 1);
    utils::meshTools::convertToAbsoluteCoordinates(mesh);
    
    logNumberOfQuads(countMeshElementsIf(mesh, isQuad));
    logNumberOfLines(countMeshElementsIf(mesh, isLine));

    copyGroupNames(mesh, groupNames);

}


Mesh StaircaseMesher::mesh() const
{
    return surfaceMesh_;
}

}
