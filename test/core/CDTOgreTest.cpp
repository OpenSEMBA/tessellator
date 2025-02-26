#include "gtest/gtest.h"

#include "CDT.h"
#include "utils/Types.h"
#include "utils/Geometry.h"

namespace meshlib::core {

using namespace CDT; // Artem Ogre CDT.

std::vector<V2d<double>> coordinatesToV2d(const Coordinates& cs) 
{
    auto n = utils::Geometry::getLSFPlaneNormal(cs);    
    utils::Geometry::rotateToXYPlane(cs.begin(), cs.end(), n);
    
    std::vector<V2d<double>> res;
    for (const auto& c : cs) {
        res.push_back(V2d<double>({c[0], c[1]}));
    }
    return res;
}

std::vector<Edge> polygonToEdges(const CoordinateIds& polygon) 
{
    std::vector<Edge> edges(polygon.size() + 1);
    for (std::size_t i = 0; i < polygon.size() + 1; i++) {
        edges[i] = Edge(polygon[i], polygon[(i + 1) % polygon.size()]);
    }
    return edges;
}

Elements triangulate(const Coordinates& globalCoords, const std::vector<CoordinateIds>& polygons) 
{
    Coordinates cs;
    CoordinateIds originalIds;
    for (auto const& polygon : polygons) {
        for (auto const& id : polygon) {
            cs.push_back(globalCoords[id]);
            originalIds.push_back(id);
        }
    }
    auto points = coordinatesToV2d(cs);


    auto cdt = Triangulation<double>();
    cdt.insertVertices(points);
    for (auto const& polygon : polygons) {
        cdt.insertEdges(polygonToEdges(polygon));
    }
    cdt.eraseOuterTrianglesAndHoles();

    std::vector<Element> res;
    for (const auto& tri : cdt.triangles) {
        Element resTri;
        resTri.vertices = {
            originalIds[tri.vertices[0]],
            originalIds[tri.vertices[1]],
            originalIds[tri.vertices[2]]
        };
        resTri.type = Element::Type::Surface;
        res.push_back(resTri);
    }
}

class CDTOgreTest : public ::testing::Test {
protected:
    static Coordinates buildCoordinates()
	{
		return Coordinates {
			Coordinate({0.00, 0.00, 0.00}),
			Coordinate({0.00, 1.00, 0.00}),
			Coordinate({1.00, 1.00, 0.00}),
			Coordinate({1.00, 0.00, 0.00}),
			Coordinate({5.00, 5.00, 5.00}),
			Coordinate({0.00, 1.00, 0.00}),
			Coordinate({0.75, 0.25, 0.00})
		};
	}

	static Coordinates buildPathCoordinates()
	{
		return Coordinates{
			Coordinate({0.0, 0.0, 0.0}),
			Coordinate({0.0, 1.0, 0.0}),
			Coordinate({1.0, 1.0, 0.0}),
			Coordinate({1.0, 0.0, 0.0}),
			Coordinate({0.25, 0.75, 0.0}),
			Coordinate({0.75, 0.75, 0.0}),
			Coordinate({5.0, 5.0, 5.0}),
		};
	}
};

TEST_F(CDTOgreTest, mesh_one_triangle_with_constraining_polygon)
{
	auto coords = buildCoordinates();
	auto tris = triangulate(coords, { {0, 1, 2} });

	EXPECT_EQ(1, tris.size());
	for (auto const& tri : tris) {
		EXPECT_EQ(Element::Type::Surface, tri.type);
		EXPECT_EQ(3, tri.vertices.size());
	}

}

// TEST_F(CDTOgreTest, mesh_bowtie)
// {
// 	std::vector<Coordinate> coords(5);
// 	coords [0] = Coordinate{{0.0, 0.0  , 0.0 }};
// 	coords [1] = Coordinate{{0.25, 0.25, 0.0 }};
// 	coords [2] = Coordinate{{0.5, 0.0  , 0.0 }};
// 	coords [3] = Coordinate{{0.75, 0.25, 0.0 }};
// 	coords [4] = Coordinate{{1.0, 0.0  , 0.0 }};


// 	Triangulation cdt(&coords);
// 	IdSet cIds = { 0,1,2,3,4,2 };
// 	Triangulation::Polygon boundary = { 0,1,2,3,4,2 };
// 	auto tris = cdt.mesh({}, { boundary });

// 	EXPECT_EQ(2, tris.size());
// 	for (auto const& tri : tris) {
// 		EXPECT_EQ(Element::Type::Surface, tri.type);
// 		EXPECT_EQ(3, tri.vertices.size());
// 	}
// }
// TEST_F(CDTOgreTest, mesh_bowtie_2)
// {
// 	std::vector<Coordinate> coords(6);
// 	coords [0] = Coordinate{{0.0, 0.0, 0.0 }};
// 	coords [1] = Coordinate{{1.0, 1.0, 0.0 }};
// 	coords [2] = Coordinate{{0.0, 1.0, 0.0 }};
// 	coords [3] = Coordinate{{1.0, 0.0, 0.0 }};
// 	coords [4] = Coordinate{{0.0, 0.25, 0.0 }};
// 	coords [5] = Coordinate{{0.2, 0.2, 0.0 }};


// 	Triangulation cdt(&coords);
// 	Triangulation::Polygon boundary = { 0,5,4,1,2,4 };
// 	auto tris = cdt.mesh({}, { boundary });

// 	EXPECT_EQ(2, tris.size());
// 	for (auto const& tri : tris) {
// 		EXPECT_EQ(Element::Type::Surface, tri.type);
// 		EXPECT_EQ(3, tri.vertices.size());
// 	}
// }

// TEST_F(CDTOgreTest, mesh_tri_with_hole)
// {
// 	std::vector<Coordinate> coords(6);
// 	coords[0] = Coordinate{ {0.0,  0.0, 0.0 } };
// 	coords[1] = Coordinate{ {0.3,  0.0, 0.0 } };
// 	coords[2] = Coordinate{ {0.15, 0.3, 0.0 } };
// 	coords[3] = Coordinate{ {0.1,  0.1, 0.0 } };
// 	coords[4] = Coordinate{ {0.2,  0.1, 0.0 } };
// 	coords[5] = Coordinate{ {0.15, 0.2, 0.0 } };


// 	Triangulation cdt(&coords);
// 	Triangulation::Polygon outer_boundary = { 0,1,2};
// 	Triangulation::Polygon inner_boundary = { 3,4,5 };
// 	auto tris = cdt.mesh({}, { outer_boundary, inner_boundary });

// 	EXPECT_EQ(6, tris.size());
// 	for (auto const& tri : tris) {
// 		EXPECT_EQ(Element::Type::Surface, tri.type);
// 		EXPECT_EQ(3, tri.vertices.size());
// 	}
// }

// TEST_F(CDTOgreTest, mesh_one_triangle_other_call)
// {
// 	auto coords = buildCoordinates();
// 	Triangulation cdt(&coords);
// 	IdSet cIds = { 0,1,2 };
// 	Triangulation::Polygon boundary = { 0,1,2 };
// 	auto tris = cdt.mesh(cIds, {boundary});

// 	EXPECT_EQ(1, tris.size());
// 	for (auto const& tri : tris) {
// 		EXPECT_EQ(Element::Type::Surface, tri.type);
// 		EXPECT_EQ(3, tri.vertices.size());
// 	}
// }

// TEST_F(CDTOgreTest, mesh_with_invalid_constraining_polygon)
// {
// 	std::vector<Coordinate> coords(5);
// 	coords[0] = Coordinate{ {0.25, 0.00, 0.00} };
// 	coords[1] = Coordinate{ {0.50, 0.00, 0.00} };
// 	coords[2] = Coordinate{ {1.00, 0.00, 0.50} };
// 	coords[3] = Coordinate{ {0.25, 0.00, 1.00} };
// 	coords[4] = Coordinate{ {1.00, 0.00, 1.00} };

// 	Triangulation cdt(&coords);
// 	{
// 		std::vector<CoordinateId> constrainingPolygon = { 0,1,2,3,4 };
// 		EXPECT_ANY_THROW(auto tris = cdt.mesh({}, { constrainingPolygon }));
// 	}
// 	{
// 		std::vector<CoordinateId> constrainingPolygon = { 0,1,2,4,3 };
// 		auto tris = cdt.mesh({}, { constrainingPolygon });
// 		for (const auto& tri : tris) {
// 			for (const auto& vertex : tri.vertices) {
// 				EXPECT_TRUE(find(
// 					constrainingPolygon.begin(),
// 					constrainingPolygon.end(),
// 					vertex) != constrainingPolygon.end());
// 			}
// 		}
// 	}
// }
// TEST_F(CDTOgreTest, mesh_two_triangles)
// {
// 	auto coords = buildCoordinates();
// 	Triangulation cdt(&coords);

// 	auto tris = cdt.mesh({}, { {0, 1, 2, 3} });

// 	EXPECT_EQ(2, tris.size());
// 	for (auto const& tri : tris) {
// 		EXPECT_EQ(Element::Type::Surface, tri.type);
// 		EXPECT_EQ(3, tri.vertices.size());
// 	}
// }

// TEST_F(CDTOgreTest, mesh_example_path)
// {
// 	auto coords = buildPathCoordinates();
// 	Triangulation cdt(&coords);

// 	auto tris = cdt.mesh({}, { { 0, 1, 4, 5, 2, 3 } });

// 	EXPECT_EQ(4, tris.size());
// 	for (auto const& tri : tris) {
// 		EXPECT_EQ(Element::Type::Surface, tri.type);
// 		EXPECT_EQ(3, tri.vertices.size());
// 	}
// }

// TEST_F(CDTOgreTest, mesh_polygons_cw_or_ccw) {
// 	Coordinates c{
// 			Coordinate({0.0, 0.0, 0.0}),
// 			Coordinate({0.0, 1.0, 0.0}),
// 			Coordinate({1.0, 1.0, 0.0}),
// 			Coordinate({0.1, 0.2, 0.0}),
// 			Coordinate({0.1, 0.9, 0.0}),
// 			Coordinate({0.8, 0.9, 0.0})
// 	};

// 	Triangulation cdt(&c);
// 	Triangulation::Polygon p1 = { 0, 1, 2 };

// 	auto trisCW  = cdt.mesh({}, { p1, { 3, 4, 5 } });
// 	auto trisCCW = cdt.mesh({}, { p1, { 5, 4, 3 } });
	
// 	EXPECT_EQ(trisCW.size(), trisCCW.size());
// }

// TEST_F(CDTOgreTest, when_not_aligned)
// {
// 	auto coords = buildCoordinates();
// 	Triangulation cdt(&coords);

// 	EXPECT_NO_THROW(cdt.mesh({ 0, 1, 2, 4 }));
// }

// TEST_F(CDTOgreTest, throw_when_coordinateId_is_out_of_range)
// {
// 	auto coords = buildCoordinates();
// 	Triangulation cdt(&coords);

// 	EXPECT_ANY_THROW(cdt.mesh({ 0, 1, 2, 350 }));
// }



}