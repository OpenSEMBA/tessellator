#include <algorithm>
#include "gtest/gtest.h"
#include <cmath>

#include "Geometry.h"
#include "Tools.h"


using namespace meshlib;
using namespace utils;

class GeometryTest : public ::testing::Test {

};

//generate random tri B, slanted +-0.5deg, check alignment
TEST_F(GeometryTest, approx_Aligned_half_deg_true) {
	TriV A;
	{
		Coordinate v0({ 0.0, 0.0, 0.0 });
		Coordinate v1({ 1.0, 0.0, 0.0 });
		Coordinate v2({ 0.0, 1.0, 0.0 });
		A = { v0, v1, v2 };
	}

	const double pi = atan(1) * 4.0;
	srand(8);
	for (int i = 0; i < 100; i++) {

		TriV B;
		{
			double rAng = double((rand() % 2) - 1.0) * 0.5;
			Coordinate v0({ 1.0, 1.0, 0.5*sqrt(2)*tan(rAng / 360.0 * 2 * pi) });
			Coordinate v1({ 0.0, 1.0, 0.0 });
			Coordinate v2({ 1.0, 0.0, 0.0 });
			B = { v0, v1, v2 };
		}
		EXPECT_TRUE(Geometry::approximatelyAligned(A, B, 0.5 / 360.0 * 2 * pi));
	}
}
//generate random tri B, slanted mora than +-0.5deg, check misalignment
TEST_F(GeometryTest, approx_Aligned_half_deg_false) {
	TriV A;
	{
		Coordinate v0({ 0.0, 0.0, 0.0 });
		Coordinate v1({ 1.0, 0.0, 0.0 });
		Coordinate v2({ 0.0, 1.0, 0.0 });
		A = { v0, v1, v2 };
	}

	const double pi = atan(1) * 4.0;
	srand(8);
	for (int i = 0; i < 100; i++) {

		TriV B;
		{
			double rAng = double((rand() % 179) - 89.5);
			if (rAng <= 0)
				rAng -= 0.5;
			if (rAng > 0)
				rAng += 0.5;
			
			Coordinate v0({ 1.0, 1.0, 0.5 * sqrt(2) * tan(rAng / 360.0 * 2 * pi) });
			Coordinate v1({ 0.0, 1.0, 0.0 });
			Coordinate v2({ 1.0, 0.0, 0.0 });
			B = { v0, v1, v2 };
		}
		EXPECT_FALSE(Geometry::approximatelyAligned(A, B, 0.5 / 360.0 * 2 * pi));
	}
}

TEST_F(GeometryTest, approx_Aligned_true) {
	Coordinate v0({ 0.0, 0.0, 0.0 });
	Coordinate v1({ 1.0, 0.0, 0.0 });
	Coordinate v2({ 0.0, 1.0, 0.0 });
	TriV A = { v0, v1, v2 };
	TriV B = A;

	const double pi = atan(1) * 4.0;
	EXPECT_TRUE(Geometry::approximatelyAligned(A, B, 1 / 360.0 * 2 * pi));
}

TEST_F(GeometryTest, approx_Aligned_plane) {
	TriV A;
	{
		Coordinate v0({ 0.25, 1.0, 1.0 });
		Coordinate v1({ 0.75, 1.0, 1.0 });
		Coordinate v2({ 0.25, 2.0, 1.0 });
		A = { v0, v1, v2 };
	}
	TriV B;
	{
		Coordinate v0({ 0.75, 1.0, 1.0 });
		Coordinate v1({ 0.75, 2.0, 1.0 });
		Coordinate v2({ 0.25, 2.0, 1.0 });
		B = { v0, v1, v2 };
	}



	const double pi = atan(1) * 4.0;
	EXPECT_TRUE(Geometry::approximatelyAligned(A, B, 0.25 / 360.0 * 2 * pi));
}

TEST_F(GeometryTest, approx_Aligned_false) 
{
	TriV A;
	{
		Coordinate v0({ 0.0, 0.0, 0.0 });
		Coordinate v1({ 1.0, 0.0, 0.0 });
		Coordinate v2({ 0.0, 1.0, 0.0 });
		A = { v0, v1, v2 }; 
	}

	TriV B;
	{
		Coordinate v0({ 0.0, 0.0, 0.0 });
		Coordinate v1({ 1.0, 0.0, 5.0 });
		Coordinate v2({ 0.0, 1.0, 0.0 });
		B = { v0, v1, v2 };
	}

	const double pi = atan(1) * 4.0;
	EXPECT_FALSE(Geometry::approximatelyAligned(A, B, 1 / 360.0 * 2 * pi));
}

TEST_F(GeometryTest, normal_test) {
	
	TriV triangle;
	Coordinate v0({ 0.0,0.0,0.0});
	Coordinate v1({ 0.0,1.0,0.0 });
	Coordinate v2({ 0.0,0.0,1.0 });

	triangle = { v0, v1, v2 };
	VecD out = Geometry::normal(triangle);
	VecD expected({ 1.0,0.0,0.0 });
	for (std::size_t i = 0; i < 3; i++) {
		EXPECT_EQ(out[i], expected[i]);
	}
}

TEST_F(GeometryTest, rotateToXYPlane)
{
	Coordinates cs = {
		Coordinate({4.0, 4.0, 0.0}),
		Coordinate({3.0, 3.0, 0.0}),
		Coordinate({4.0, 3.0, 0.0}),
		Coordinate({3.0, 4.0, 0.0}),
	};
	EXPECT_EQ(VecD({ 0.0, 0.0, 1.0 }), Geometry::getNormal(cs, 0.5));
	Coordinates rot = cs;
	Geometry::rotateToXYPlane(rot.begin(), rot.end(), Geometry::getNormal(cs, 0.5));
	EXPECT_EQ(cs, rot);
	
}

TEST_F(GeometryTest, area_test)
{

	srand(8);
	for (std::size_t i = 0; i < 100; i++) {

		TriV triangle;
		
		Coordinate v0({ double((rand() % 2) - 1.0),double((rand() % 2) - 1.0),double((rand() % 2) - 1.0) });
		Coordinate v1({ double((rand() % 2) - 1.0),double((rand() % 2) - 1.0),double((rand() % 2) - 1.0) });
		Coordinate v2({ double((rand() % 2) - 1.0),double((rand() % 2) - 1.0),double((rand() % 2) - 1.0) });
		triangle = { v0, v1, v2 };
		
		double length01 = 0;
		double length12 = 0;
		double length20 = 0;
		for (std::size_t j = 0; j < 3; j++) {
			length01 += std::pow(v0[j] - v1[j], 2);
			length12 += std::pow(v1[j] - v2[j], 2);
			length20 += std::pow(v2[j] - v0[j], 2);
		}

		length01 = std::sqrt(length01);
		length12 = std::sqrt(length12);
		length20 = std::sqrt(length20);

		double lengthSemiSum = 0.5 * (length01 + length12 + length20);
		double heronFormulaArea = std::sqrt(lengthSemiSum * (lengthSemiSum - length01) * 
															(lengthSemiSum - length12) * 
															(lengthSemiSum - length20));

		EXPECT_NEAR(Geometry::area(triangle), heronFormulaArea, 1e-07);
	}

}

TEST_F(GeometryTest, disjointSet_bowTie_test) {

	std::vector<Coordinate> coords = {
			Coordinate({ 0.00, 0.00, 0.00}),
			Coordinate({ 0.00, 1.00, 0.00}),
			Coordinate({ 1.00, 0.00, 0.00}),
			Coordinate({ 1.00, 1.00, 0.00}),
			Coordinate({ 0.50, 0.50, 0.00})
	};
	Elements elems;
	Element tri;
	tri.type = Element::Type::Surface;
	tri.vertices = { 0  ,      4 ,       1 };
	elems.push_back(tri);
	tri.vertices = { 4  ,      2 ,       3 };
	elems.push_back(tri);
	ElementsView elemsView;
	for (const auto& element : elems) {
		elemsView.push_back(&element);
	}

	auto ds = Geometry::buildDisjointSmoothSets(elemsView, coords, 80);
	EXPECT_EQ(2, ds.size());

}

TEST_F(GeometryTest, disjointSet_coplanarOrNot_test) {

	Elements elems;
	Element tri;
	tri.type = Element::Type::Surface;
	tri.vertices = { 0, 1, 2 };
	elems.push_back(tri);
	tri.vertices = { 1, 3, 2 };
	elems.push_back(tri);
	{
		std::vector<Coordinate> coords = {
			Coordinate({ 0.00, 0.00, 0.00}),
			Coordinate({ 1.00, 0.00, 0.00}),
			Coordinate({ 0.00, 1.00, 0.00}),
			Coordinate({ 0.55, 0.55, 1.00}) 
		};
		ElementsView elemsView;
		for (const auto& element : elems) {
			elemsView.push_back(&element);
		}
		auto ds = Geometry::buildDisjointSmoothSets(elemsView, coords, 80);
		EXPECT_EQ(2, ds.size());
	}
	{
		std::vector<Coordinate> coords = {
			Coordinate({ 0.00, 0.00, 0.00}),
			Coordinate({ 1.00, 0.00, 0.00}),
			Coordinate({ 0.00, 1.00, 0.00}),
			Coordinate({ 1.00, 1.00, 0.00})
		};
		ElementsView elemsView;
		for (const auto& element : elems) {
			elemsView.push_back(&element);
		}
		auto ds = Geometry::buildDisjointSmoothSets(elemsView, coords, 80);
		EXPECT_EQ(1, ds.size());
	}
}

TEST_F(GeometryTest, disjointSet_localSmoothing_test) 
{

	Coordinates coords = {
		Coordinate({ 1.00, 0.00, 0.00}),
		Coordinate({ 1.00, 1.00, 0.00}),
		Coordinate({ 0.50, 0.00, 0.00}),
		Coordinate({ 0.50, 1.00, 0.00}),
		Coordinate({ 0.00, 0.00, 0.50}),
		Coordinate({ 0.00, 1.00, 0.50}),
		Coordinate({ 0.00, 0.00, 1.00}),
		Coordinate({ 0.00, 1.00, 1.00})
	};

	Elements elems;
	Element tri;
	tri.type = Element::Type::Surface;
	tri.vertices = { 0, 1, 2 };
	elems.push_back(tri);
	tri.vertices = { 2, 1, 3 };
	elems.push_back(tri);
	tri.vertices = { 2, 3, 4 };
	elems.push_back(tri);
	tri.vertices = { 3, 5, 4 };
	elems.push_back(tri);
	tri.vertices = { 4, 5, 7 };
	elems.push_back(tri);
	tri.vertices = { 4, 7, 6 };
	elems.push_back(tri);

	auto ds = Geometry::buildDisjointSmoothSets( getView(elems), coords, 80);
	EXPECT_EQ(1, ds.size());

}

TEST_F(GeometryTest, disjointSets_topological_orientation)
{
	Coordinates cs = {
		Coordinate({0.0, 0.0, 0.0}),
		Coordinate({1.0, 0.0, 0.0}),
		Coordinate({0.0, 1.0, 0.0}),
		Coordinate({0.0, 0.0, 1.0})
	};
	{
		Elements es = {
			Element({0, 1, 2}, Element::Type::Surface),
			Element({1, 0, 3}, Element::Type::Surface)
		};
		EXPECT_EQ(2, Geometry::buildDisjointSmoothSets(getView(es), cs, 80.0).size());
		EXPECT_EQ(1, Geometry::buildDisjointSmoothSets(getView(es), cs, 100.0).size());
	}
	{
		Elements es = {
			Element({0, 1, 2}, Element::Type::Surface),
			Element({0, 1, 3}, Element::Type::Surface)
		};
		EXPECT_EQ(2, Geometry::buildDisjointSmoothSets(getView(es), cs, 80.0).size());
		EXPECT_EQ(2, Geometry::buildDisjointSmoothSets(getView(es), cs, 100.0).size());
	}

}

TEST_F(GeometryTest, areAdjacent_for_tris) 
{
	// 0 -> 1 <-  3 
	//   \  |  /  |
	//      2  -> 4 -> 5
	//            |  /
	//            6
	Element t1({ 0, 1, 2 }, Element::Type::Surface);
	Element t2({ 1, 2, 3 }, Element::Type::Surface);
	Element t3({ 3, 2, 4 }, Element::Type::Surface);
	Element t4({ 4, 5, 6 }, Element::Type::Surface);
	
	EXPECT_FALSE(Geometry::areAdjacentWithSameTopologicalOrientation(t1, t2));
	EXPECT_TRUE (Geometry::areAdjacentWithSameTopologicalOrientation(t2, t3));
	EXPECT_FALSE(Geometry::areAdjacentWithSameTopologicalOrientation(t1, t3));
	EXPECT_FALSE(Geometry::areAdjacentWithSameTopologicalOrientation(t3, t4));

	auto t = t2;
	for (std::size_t i = 0; i < 3; i++) {
		std::rotate(t.vertices.begin(), t.vertices.begin() +1, t.vertices.end());
		EXPECT_FALSE(Geometry::areAdjacentWithSameTopologicalOrientation(t1, t));
		EXPECT_TRUE(Geometry::areAdjacentWithSameTopologicalOrientation(t, t3));
	}
}

TEST_F(GeometryTest, areAdjacent_for_lines) 
{
	Element l1({ 0, 1 }, Element::Type::Line);
	Element l2({ 1, 2 }, Element::Type::Line);
	Element l3({ 2, 3 }, Element::Type::Line);

	EXPECT_TRUE (Geometry::areAdjacentLines(l1, l2));
	EXPECT_TRUE (Geometry::areAdjacentLines(l2, l3));
	EXPECT_FALSE(Geometry::areAdjacentLines(l1, l3));
}