#include <gtest/gtest.h>
#include "core/Compressor.h"
#include "MeshFixtures.h"

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
    
    EXPECT_EQ(merged, 1u);
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 1u);
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

TEST_F(CompressorTest, CompressLShapeIntoOneSurface) {
    // Create 3 quads in L-shape - should be merged into one surface
    
    Mesh mesh;
    mesh.grid = grid_;
    
    // Quad 1: bottom-left
    addQuad(mesh, {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0});
    // Quad 2: bottom-right
    addQuad(mesh, {1, 0, 0}, {2, 0, 0}, {2, 1, 0}, {1, 1, 0});
    // Quad 3: top-left (forming L-shape)
    addQuad(mesh, {0, 1, 0}, {1, 1, 0}, {1, 2, 0}, {0, 2, 0});
    
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 3u);
    
    auto merged = core::Compressor::compressSurfaces(mesh);
    
    EXPECT_EQ(merged, 1u);
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 1u);
}

TEST_F(CompressorTest, CompressWithHoleCreatesInnerContour) {
    // Create 8 quads forming a ring with a hole in the middle
    // Should be merged into one surface with inner contour
    
    Mesh mesh;
    mesh.grid = grid_;
    
    // Outer ring of quads (leaving center 1,1 to 2,2 empty)
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
    
    EXPECT_EQ(merged, 1u);
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 1u);
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

}
