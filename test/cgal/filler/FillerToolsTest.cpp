#include "gtest/gtest.h"
#include "MeshFixtures.h"

#include "filler/FillerTools.h"

using namespace meshlib;
using namespace tessellator;
using namespace filler;
using namespace cgal;

class FillerToolsTest : public ::testing::Test {
public:
    
    static std::vector<const CDT::Face*> getView(const CDT& tris)
    {
        std::vector<const CDT::Face*> r;
        for (const auto& f : tris.finite_face_handles()) {
            r.push_back(&*f);
        }
        return r;
    }
};

TEST_F(FillerToolsTest, build_polygons_from_relatively_simple_triangulation)
{
    Polygon p1;
    p1.push_back({ 0.0, 0.0 });
    p1.push_back({ 1.0, 0.0 });
    p1.push_back({ 0.0, 1.0 });
    
    Polygon p2;
    p2.push_back({ 2.0, 0.0 });
    p2.push_back({ 1.0, 1.0 });
    p2.push_back({ 1.0, 0.0 });

    HPolygonSet pS{ p1 };
    pS.join(p2);
    
    auto cdts{ buildCDTsFromPolygonSet(pS) };
    EXPECT_EQ(2, cdts.size());
    
    auto P{ buildPolygonSetFromCDT(getView(cdts.front())) };
    EXPECT_EQ(1, P.size());
}

TEST_F(FillerToolsTest, build_polygons_from_triangulation)
{
    Polygon p1;
    p1.push_back({ 7.00699997, 7.00614977 });
    p1.push_back({ 7.01317692, 7.00000000 });
    p1.push_back({ 7.01317787, 7.00000000 });
    Polygon p2;
    p2.push_back({ 7.00699997, 7.00614977 });
    p2.push_back({ 7.00000000, 7.01311111 });
    p2.push_back({ 7.00000000, 7.01311016 });

    HPolygonSet pS{ p1 };
    pS.join(p2);

    auto cdt{ buildCDTsFromPolygonSet(pS) };
    ASSERT_EQ(2, cdt.size());

    auto r{ buildPolygonSetFromCDT(getView(cdt.front())) };
    EXPECT_EQ(1, r.size());
}

TEST_F(FillerToolsTest, buildCDT_from_polygon)
{
    Polygon p;
    p.push_back({ 0.0, 0.0 });
    p.push_back({ 1.0, 0.0 });
    p.push_back({ 0.0, 1.0 });
    p.push_back({ 0.0, 1.0+std::numeric_limits<KType>::epsilon()});

    HPolygonSet pS{ p };
    auto cdt{ buildCDTsFromPolygonSet(pS) };

    EXPECT_EQ(1, cdt.size());
}

TEST_F(FillerToolsTest, intersectionTriWithCell)
{
    EXPECT_TRUE(
        isCellCrossedByTriangle(
            Triangle2{
                {0.0, 0.0},
                {1.0, 0.0},
                {0.0, 1.0}
            },
            {0, 0}
        )
    );
}

TEST_F(FillerToolsTest, intersectionTriWithCell_point)
{
    EXPECT_FALSE(
        isCellCrossedByTriangle(
            Triangle2{
                { 0.0,  0.0},
                {-1.0,  0.0},
                { 0.0, -1.0}
            },
            { 0, 0 }
        )
    );
}

TEST_F(FillerToolsTest, intersectionTriWithCell_degenerate)
{
    EXPECT_FALSE(
        isCellCrossedByTriangle(
            Triangle2{
                { 0.0,  0.0},
                { 0.0,  0.0},
                { 0.0,  0.0}
            },
            { 0, 0 }
        )
    );
}