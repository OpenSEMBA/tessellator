#include <gtest/gtest.h>
#include "core/Compressor.h"
#include "core/Splitter.h"
#include "MeshFixtures.h"
#include "utils/MeshTools.h"

using namespace meshlib;
using namespace meshlib::utils::meshTools;

namespace meshlib::tests {

class CompressorTest : public ::testing::Test {
protected:
    void SetUp() override {
        grid_ = {
            std::vector<double>{0, 1, 2, 3, 4},
            std::vector<double>{0, 1, 2, 3, 4},
            std::vector<double>{0, 1, 2, 3, 4}
        };
    }

    Grid grid_;
};

TEST_F(CompressorTest, Compress2x2QuadsIntoOneSurface) {
    // Create 4 quads arranged in a 2x2 pattern on the same plane
    // They should be merged into a single surface
    
    Mesh mesh;
    mesh.grid = grid_;
    
    // Quad 1: bottom-left (cells 0,0 to 1,1)
    addQuad(mesh, {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0});
    // Quad 2: bottom-right (cells 1,0 to 2,1)
    addQuad(mesh, {1, 0, 0}, {2, 0, 0}, {2, 1, 0}, {1, 1, 0});
    // Quad 3: top-left (cells 0,1 to 1,2)
    addQuad(mesh, {0, 1, 0}, {1, 1, 0}, {1, 2, 0}, {0, 2, 0});
    // Quad 4: top-right (cells 1,1 to 2,2)
    addQuad(mesh, {1, 1, 0}, {2, 1, 0}, {2, 2, 0}, {1, 2, 0});
    
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 4u);
    
    auto merged = core::Compressor::compressSurfaces(mesh);
    
    EXPECT_EQ(merged, 3u);
    ASSERT_EQ(mesh.groups.size(), 1u);
    ASSERT_EQ(mesh.groups[0].elements.size(), 1u);
    
    EXPECT_EQ(
        CoordinateIds({0, 4, 8, 7}), 
        mesh.groups[0].elements[0].vertices
    );

}

TEST_F(CompressorTest, DoesNotCompressNonCoplanarQuads) {
    // Create quads on different planes - should not be merged
    
    Mesh mesh;
    mesh.grid = grid_;
    
    // Quad on z=0 plane
    addQuad(mesh, {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0});
    // Quad on z=1 plane (different plane)
    addQuad(mesh, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1});
    
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 2u);
    
    auto merged = core::Compressor::compressSurfaces(mesh);
    
    EXPECT_EQ(merged, 0u);
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 2u);
}

TEST_F(CompressorTest, DoesNotCompressDisconnectedQuads) {
    // Create quads on same plane but not connected - should not be merged
    
    Mesh mesh;
    mesh.grid = grid_;
    
    // Quad at bottom-left
    addQuad(mesh, {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0});
    // Quad at top-right (disconnected)
    addQuad(mesh, {3, 3, 0}, {4, 3, 0}, {4, 4, 0}, {3, 4, 0});
    
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 2u);
    
    auto merged = core::Compressor::compressSurfaces(mesh);
    
    EXPECT_EQ(merged, 0u);
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 2u);
}

TEST_F(CompressorTest, CompressWithHoleCreatesInnerContour) {
    // Create 8 quads forming a ring with a hole in the middle
    // The ring decomposes into 4 rectangles (left col, right col, top center, bottom center)
    
    Mesh mesh;
    mesh.grid = grid_;
    
    // Outer ring of quads (leaving center 1,1 to 2,2 empty)
    // Layout:
    //    x=0  x=1  x=2  x=3
    // y=3  [5]   [8]   [6]
    // y=2  [3]   hole  [4]
    // y=1  [1]   [7]   [2]
    // y=0  
    // Bottom row
    addQuad(mesh, {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0});
    addQuad(mesh, {2, 0, 0}, {3, 0, 0}, {3, 1, 0}, {2, 1, 0});
    // Middle row (sides only)
    addQuad(mesh, {0, 1, 0}, {1, 1, 0}, {1, 2, 0}, {0, 2, 0});
    addQuad(mesh, {2, 1, 0}, {3, 1, 0}, {3, 2, 0}, {2, 2, 0});
    // Top row
    addQuad(mesh, {0, 2, 0}, {1, 2, 0}, {1, 3, 0}, {0, 3, 0});
    addQuad(mesh, {2, 2, 0}, {3, 2, 0}, {3, 3, 0}, {2, 3, 0});
    // Corners to complete the ring
    addQuad(mesh, {1, 0, 0}, {2, 0, 0}, {2, 1, 0}, {1, 1, 0});
    addQuad(mesh, {1, 3, 0}, {2, 3, 0}, {2, 2, 0}, {1, 2, 0});
    
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 8u);
    
    auto merged = core::Compressor::compressSurfaces(mesh);

    auto finalCount = countMeshElementsIf(mesh, isQuad);
    
    // Optimal decomposition: 4 rectangles
    // - Left column (quads 1,3,5): cells x=0, y=0-3
    // - Right column (quads 2,4,6): cells x=2-3, y=0-3
    // - Top center (quad 8): cell x=1-2, y=2-3
    // - Bottom center (quad 7): cell x=1-2, y=0-1
    EXPECT_EQ(finalCount, 4u);
    EXPECT_EQ(merged, 4u); // 8 - 4 = 4 surfaces merged
}

TEST_F(CompressorTest, DoesNotCompressQuadsWithDifferentNormals) {
    // Create quads with different normal directions - should not be merged
    
    Mesh mesh;
    mesh.grid = grid_;
    
    // Quad on z=0 plane, normal pointing +z
    addQuad(mesh, {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0});
    // Quad on y=0 plane, normal pointing +y (different orientation)
    addQuad(mesh, {0, 0, 0}, {1, 0, 0}, {1, 0, 1}, {0, 0, 1});
    
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 2u);
    
    auto merged = core::Compressor::compressSurfaces(mesh);
    
    EXPECT_EQ(merged, 0u);
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 2u);
}

TEST_F(CompressorTest, CompressAndSplit2x2GridRoundTrip) {
    // Create 4 quads in 2x2 grid, compress to 1 surface, split back to 4 quads
    
    Mesh mesh;
    mesh.grid = grid_;
    
    // 2x2 grid of quads
    addQuad(mesh, {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0});
    addQuad(mesh, {1, 0, 0}, {2, 0, 0}, {2, 1, 0}, {1, 1, 0});
    addQuad(mesh, {0, 1, 0}, {1, 1, 0}, {1, 2, 0}, {0, 2, 0});
    addQuad(mesh, {1, 1, 0}, {2, 1, 0}, {2, 2, 0}, {1, 2, 0});
    
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 4u);
    
    // Compress: 4 quads -> 1 surface
    auto merged = core::Compressor::compressSurfaces(mesh);
    EXPECT_EQ(merged, 3u);
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 1u);
    
    // Split: 1 surface -> 4 quads
    auto splitCount = core::Splitter::splitSurfaces(mesh);
    EXPECT_EQ(splitCount, 4u);
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 4u);
}

TEST_F(CompressorTest, CompressAndSplitRingRoundTrip) {
    // Create ring of 8 quads, compress, split back
    
    Mesh mesh;
    mesh.grid = grid_;
    
    // Ring of 8 quads (same as CompressWithHoleCreatesInnerContour)
    addQuad(mesh, {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0});
    addQuad(mesh, {2, 0, 0}, {3, 0, 0}, {3, 1, 0}, {2, 1, 0});
    addQuad(mesh, {0, 1, 0}, {1, 1, 0}, {1, 2, 0}, {0, 2, 0});
    addQuad(mesh, {2, 1, 0}, {3, 1, 0}, {3, 2, 0}, {2, 2, 0});
    addQuad(mesh, {0, 2, 0}, {1, 2, 0}, {1, 3, 0}, {0, 3, 0});
    addQuad(mesh, {2, 2, 0}, {3, 2, 0}, {3, 3, 0}, {2, 3, 0});
    addQuad(mesh, {1, 0, 0}, {2, 0, 0}, {2, 1, 0}, {1, 1, 0});
    addQuad(mesh, {1, 3, 0}, {2, 3, 0}, {2, 2, 0}, {1, 2, 0});
    
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 8u);
    
    // Compress: 8 quads -> 4 surfaces (left col, right col, top center, bottom center)
    auto merged = core::Compressor::compressSurfaces(mesh);
    EXPECT_EQ(merged, 4u); // 8 - 4 = 4 surfaces merged
    auto compressedCount = countMeshElementsIf(mesh, isQuad);
    EXPECT_EQ(compressedCount, 4u);
    
    // Split: 4 surfaces -> 8 quads
    auto splitCount = core::Splitter::splitSurfaces(mesh);
    EXPECT_EQ(splitCount, 8u);
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 8u);
}

TEST_F(CompressorTest, CompressAndSplit3x3GridRoundTrip) {
    // Create 9 quads in 3x3 grid, compress to 1 surface, split back to 9 quads
    
    Mesh mesh;
    mesh.grid = grid_;
    
    // 3x3 grid of quads
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            addQuad(mesh, 
                {i, j, 0}, {i+1, j, 0}, 
                {i+1, j+1, 0}, {i, j+1, 0});
        }
    }
    
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 9u);
    
    // Compress: 9 quads -> 1 surface
    auto merged = core::Compressor::compressSurfaces(mesh);
    EXPECT_EQ(merged, 8u);
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 1u);
    
    // Split: 1 surface -> 9 quads
    auto splitCount = core::Splitter::splitSurfaces(mesh);
    EXPECT_EQ(splitCount, 9u);
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 9u);
}

}
