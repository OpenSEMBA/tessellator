#include <gtest/gtest.h>
#include "core/Compressor.h"
#include "MeshFixtures.h"
#include "utils/MeshTools.h"

using namespace meshlib;
using namespace meshlib::utils::meshTools;

namespace meshlib::tests {

class CompressorTest : public ::testing::Test {
protected:
    void SetUp() override {
        grid_ = {
            std::vector<double>{0, 1, 2, 3, 4, 5, 6},
            std::vector<double>{0, 1, 2, 3, 4, 5, 6},
            std::vector<double>{0, 1, 2, 3, 4, 5, 6}
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

TEST_F(CompressorTest, CompresRepeatedQuadsIntoOneSurface) {
    // Create 4 quads arranged in a 2x2 pattern on the same plane
    // They should be merged into a single surface
    
    Mesh mesh;
    mesh.grid = grid_;
    
    addQuad(mesh, {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0});
    addQuad(mesh, {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0});
    
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 2u);
    
    auto merged = core::Compressor::compressSurfaces(mesh);
    
    EXPECT_EQ(merged, 1u);
    ASSERT_EQ(mesh.groups.size(), 1u);
    ASSERT_EQ(mesh.groups[0].elements.size(), 1u);
    
    EXPECT_EQ(
        CoordinateIds({0, 1, 2, 3}), 
        mesh.groups[0].elements[0].vertices
    );

}

TEST_F(CompressorTest, DoesNotCompressQuadsWithDifferentOrientation) {
    // Create quads on different planes - should not be merged
    
    Mesh mesh;
    mesh.grid = grid_;
    
    // Counter clock-wise quad
    addQuad(mesh, {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0});
    // Adjacent clock-wise quad
    addQuad(mesh, {1, 0, 0}, {1, 1, 0}, {2, 1, 0}, {1, 0, 0});
    
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 2u);
    
    auto merged = core::Compressor::compressSurfaces(mesh);
    
    EXPECT_EQ(merged, 0u);
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 2u);
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
    //    x=0  x=1   x=2   x=3
    // y=3
    //      [5]   [8]   [6]
    // y=2
    //      [3]   hole  [4]
    // y=1
    //      [1]   [7]   [2]
    // y=0    
    // Bottom row
    addQuad(mesh, {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0});  //  0,  1,  2,  3
    addQuad(mesh, {2, 0, 0}, {3, 0, 0}, {3, 1, 0}, {2, 1, 0});  //  4,  5,  6,  7
    // Middle row (sides only) 
    addQuad(mesh, {0, 1, 0}, {1, 1, 0}, {1, 2, 0}, {0, 2, 0});  //  3,  2,  8,  9
    addQuad(mesh, {2, 1, 0}, {3, 1, 0}, {3, 2, 0}, {2, 2, 0});  //  7,  6, 10, 11
    // Top row 
    addQuad(mesh, {0, 2, 0}, {1, 2, 0}, {1, 3, 0}, {0, 3, 0});  //  9,  8, 12, 13
    addQuad(mesh, {2, 2, 0}, {3, 2, 0}, {3, 3, 0}, {2, 3, 0});  // 11, 10, 14, 15
    // Corners to complete the ring
    addQuad(mesh, {1, 0, 0}, {2, 0, 0}, {2, 1, 0}, {1, 1, 0});  //  1,  4,  7,  2
    addQuad(mesh, {1, 3, 0}, {1, 2, 0}, {2, 2, 0}, {2, 3, 0});  // 12,  8, 11, 15
    
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 8u);
    
    auto merged = core::Compressor::compressSurfaces(mesh);

    auto finalCount = countMeshElementsIf(mesh, isQuad);
    
    // Optimal decomposition: 4 rectangles
    // - Bottom row (quads 1,7,2): cells x=0-3, y=0-1
    // - Left center (quad 3): cell x=0-1, y=1-2
    // - Top row (quads 5, 8, 6): cell x=0-3, y=2-3
    // - Right center (quad 4): cell x=2-3, y=1-2
    EXPECT_EQ(finalCount, 4u);
    EXPECT_EQ(merged, 4u); // 8 - 4 = 4 surfaces merged
    
    EXPECT_EQ(
        CoordinateIds({0, 5, 6, 3}), 
        mesh.groups[0].elements[0].vertices
    );
    
    EXPECT_EQ(
        CoordinateIds({3, 2, 8, 9}), 
        mesh.groups[0].elements[1].vertices
    );
    
    EXPECT_EQ(
        CoordinateIds({9, 10, 14, 13}), 
        mesh.groups[0].elements[2].vertices
    );
    
    EXPECT_EQ(
        CoordinateIds({7, 6, 10, 11}), 
        mesh.groups[0].elements[3].vertices
    );
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

TEST_F(CompressorTest, CompressRingRoundTrip) {
    // Create ring of 8 quads, compress
    
    Mesh mesh;
    mesh.grid = grid_;
    
    // Outer ring of quads (leaving center 1,1 to 2,2 empty)
    // Layout:
    //    x=0  x=1   x=2   x=3   x=4
    // y=4
    //      [10]   [09]   [08]   [07]
    // y=3                        
    //      [11]                 [06] 
    // y=2                           
    //      [12]                 [05] 
    // y=1                        
    //      [01]   [02]   [03]   [04]
    // y=0    
    // Bottom row
    
    addQuad(mesh, {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0});  // 01
    addQuad(mesh, {1, 0, 0}, {2, 0, 0}, {2, 1, 0}, {1, 1, 0});  // 02
    addQuad(mesh, {2, 0, 0}, {3, 0, 0}, {3, 1, 0}, {2, 1, 0});  // 03
    addQuad(mesh, {3, 0, 0}, {4, 0, 0}, {4, 1, 0}, {3, 1, 0});  // 04
    addQuad(mesh, {3, 1, 0}, {4, 1, 0}, {4, 2, 0}, {3, 2, 0});  // 05
    addQuad(mesh, {3, 2, 0}, {4, 2, 0}, {4, 3, 0}, {3, 3, 0});  // 06
    addQuad(mesh, {3, 3, 0}, {4, 3, 0}, {4, 4, 0}, {3, 4, 0});  // 07
    addQuad(mesh, {2, 3, 0}, {3, 3, 0}, {3, 4, 0}, {2, 4, 0});  // 08
    addQuad(mesh, {1, 4, 0}, {1, 3, 0}, {2, 3, 0}, {2, 4, 0});  // 09
    addQuad(mesh, {0, 3, 0}, {1, 3, 0}, {1, 4, 0}, {0, 4, 0});  // 10
    addQuad(mesh, {0, 2, 0}, {1, 2, 0}, {1, 3, 0}, {0, 3, 0});  // 11
    addQuad(mesh, {0, 1, 0}, {1, 1, 0}, {1, 2, 0}, {0, 2, 0});  // 12
    
    EXPECT_EQ(countMeshElementsIf(mesh, isQuad), 12u);
    
    // Compress: 12 quads -> 4 surfaces ()
    auto merged = core::Compressor::compressSurfaces(mesh);
    EXPECT_EQ(merged, 8u); // 12 - 4 = 4 surfaces merged
    auto compressedCount = countMeshElementsIf(mesh, isQuad);
    EXPECT_EQ(compressedCount, 4u);
}

TEST_F(CompressorTest, Compress3x3GridRoundTrip) {
    // Create 9 quads in 3x3 grid, compress to 1 surface
    
    Mesh mesh;
    mesh.grid = grid_;
    
    // 3x3 grid of quads
    // Layout:
    //    x=0  x=1   x=2   x=3
    // y=3
    //      [3]   [6]   [9]
    // y=2
    //      [2]   [5]   [8]
    // y=1
    //      [1]   [4]   [7]
    // y=0
    //  
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
    
    EXPECT_EQ(
        CoordinateIds({0, 12, 15, 7}), 
        mesh.groups[0].elements[0].vertices
    );
}

// ============== Line Compression Tests ==============

TEST_F(CompressorTest, Compress2CollinearLinesIntoOne) {
    Mesh mesh;
    mesh.grid = grid_;
    
    addLine(mesh, {0, 0, 0}, {1, 0, 0});
    addLine(mesh, {1, 0, 0}, {2, 0, 0});
    
    EXPECT_EQ(countMeshElementsIf(mesh, isLine), 2u);
    
    auto merged = core::Compressor::compressLines(mesh);
    
    EXPECT_EQ(merged, 1u);
    EXPECT_EQ(countMeshElementsIf(mesh, isLine), 1u);
    
    ASSERT_EQ(mesh.groups[0].elements.size(), 1u);
    const auto& line = mesh.groups[0].elements[0];
    EXPECT_EQ(line.type, Element::Type::Line);
    EXPECT_EQ(CoordinateIds({0, 2}), line.vertices);
}

TEST_F(CompressorTest, CompressRepeatedLinesIntoOne) {
    Mesh mesh;
    mesh.grid = grid_;
    
    addLine(mesh, {0, 0, 0}, {1, 0, 0});
    addLine(mesh, {0, 0, 0}, {1, 0, 0});
    
    EXPECT_EQ(countMeshElementsIf(mesh, isLine), 2u);
    
    auto merged = core::Compressor::compressLines(mesh);
    
    EXPECT_EQ(merged, 1u);
    EXPECT_EQ(countMeshElementsIf(mesh, isLine), 1u);
    
    ASSERT_EQ(mesh.groups[0].elements.size(), 1u);
    const auto& line = mesh.groups[0].elements[0];
    EXPECT_EQ(line.type, Element::Type::Line);
    EXPECT_EQ(CoordinateIds({0, 1}), line.vertices);
}

TEST_F(CompressorTest, DoesNotCompressNonCollinearLines) {
    Mesh mesh;
    mesh.grid = grid_;
    
    addLine(mesh, {0, 0, 0}, {1, 0, 0});
    addLine(mesh, {0, 0, 0}, {0, 1, 0});
    
    EXPECT_EQ(countMeshElementsIf(mesh, isLine), 2u);
    
    auto merged = core::Compressor::compressLines(mesh);
    
    EXPECT_EQ(merged, 0u);
    EXPECT_EQ(countMeshElementsIf(mesh, isLine), 2u);
}

TEST_F(CompressorTest, DoesNotCompressDisconnectedLines) {
    Mesh mesh;
    mesh.grid = grid_;
    
    addLine(mesh, {0, 0, 0}, {1, 0, 0});
    addLine(mesh, {3, 0, 0}, {4, 0, 0});
    
    EXPECT_EQ(countMeshElementsIf(mesh, isLine), 2u);
    
    auto merged = core::Compressor::compressLines(mesh);
    
    EXPECT_EQ(merged, 0u);
    EXPECT_EQ(countMeshElementsIf(mesh, isLine), 2u);
}

TEST_F(CompressorTest, DoesNotCompressOverlappingOppositeDirectionLines) {
    Mesh mesh;
    mesh.grid = grid_;
    
    addLine(mesh, {0, 0, 0}, {1, 0, 0});
    addLine(mesh, {1, 0, 0}, {0, 0, 0});
    
    EXPECT_EQ(countMeshElementsIf(mesh, isLine), 2u);
    
    auto merged = core::Compressor::compressLines(mesh);
    
    EXPECT_EQ(merged, 0u);
    EXPECT_EQ(countMeshElementsIf(mesh, isLine), 2u);
}

TEST_F(CompressorTest, DoesNotCompressConnectedOppositeDirectionLines) {
    Mesh mesh;
    mesh.grid = grid_;
    
    addLine(mesh, {0, 0, 0}, {1, 0, 0});
    addLine(mesh, {2, 0, 0}, {1, 0, 0});
    
    EXPECT_EQ(countMeshElementsIf(mesh, isLine), 2u);
    
    auto merged = core::Compressor::compressLines(mesh);
    
    EXPECT_EQ(merged, 0u);
    EXPECT_EQ(countMeshElementsIf(mesh, isLine), 2u);
}

TEST_F(CompressorTest, Compress3LinesIntoOne) {
    Mesh mesh;
    mesh.grid = grid_;
    
    addLine(mesh, {0, 0, 0}, {1, 0, 0});
    addLine(mesh, {1, 0, 0}, {2, 0, 0});
    addLine(mesh, {2, 0, 0}, {3, 0, 0});
    
    EXPECT_EQ(countMeshElementsIf(mesh, isLine), 3u);
    
    auto merged = core::Compressor::compressLines(mesh);
    
    EXPECT_EQ(merged, 2u);
    EXPECT_EQ(countMeshElementsIf(mesh, isLine), 1u);
    EXPECT_EQ(CoordinateIds({0, 3}), mesh.groups[0].elements[0].vertices);
}

TEST_F(CompressorTest, Compress5LineRoundTrip) {
    Mesh mesh;
    mesh.grid = grid_;
    
    addLine(mesh, {0, 0, 0}, {0, 1, 0});
    addLine(mesh, {0, 1, 0}, {0, 2, 0});
    addLine(mesh, {0, 2, 0}, {0, 3, 0});
    addLine(mesh, {0, 3, 0}, {0, 4, 0});
    addLine(mesh, {0, 4, 0}, {0, 5, 0});
    
    EXPECT_EQ(countMeshElementsIf(mesh, isLine), 5u);
    
    core::Compressor::compressLines(mesh);
    EXPECT_EQ(countMeshElementsIf(mesh, isLine), 1u);
    EXPECT_EQ(CoordinateIds({0, 5}), mesh.groups[0].elements[0].vertices);
}

TEST_F(CompressorTest, CompressMixedDirections) {
    Mesh mesh;
    mesh.grid = grid_;
    
    // Layout:
    //            z
    //           v6
    //            |l6 
    //           v5
    //            |l5     
    //           v0———v1———v2    x
    //        l3 /  l1   l2
    //         v3
    //      l4 /
    //       v4
    //      y

    addLine(mesh, {0, 0, 0}, {1, 0, 0});
    addLine(mesh, {1, 0, 0}, {2, 0, 0});
    addLine(mesh, {0, 0, 0}, {0, 1, 0});
    addLine(mesh, {0, 1, 0}, {0, 2, 0});
    addLine(mesh, {0, 0, 0}, {0, 0, 1});
    addLine(mesh, {0, 0, 1}, {0, 0, 2});
    
    EXPECT_EQ(countMeshElementsIf(mesh, isLine), 6u);
    
    auto merged = core::Compressor::compressLines(mesh);
    
    EXPECT_EQ(merged, 3u);
    EXPECT_EQ(countMeshElementsIf(mesh, isLine), 3u);

    EXPECT_EQ(CoordinateIds({0, 2}), mesh.groups[0].elements[0].vertices);
    EXPECT_EQ(CoordinateIds({0, 4}), mesh.groups[0].elements[1].vertices);
    EXPECT_EQ(CoordinateIds({0, 6}), mesh.groups[0].elements[2].vertices);
}

}
