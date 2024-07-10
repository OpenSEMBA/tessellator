#include "gtest/gtest.h"

#include "cgal/HPolygonSet.h"
	
using namespace meshlib;
using namespace cgal;

class HPolygonSetTest : public ::testing::Test {
public:
	static bool areEqual(const Polygon& p, const Polygon& q)
	{
		return std::equal(p.begin(), p.end(), q.begin(), q.end());
	}

	static Polygon buildBowtiePolygon()
	{
		//  X X
		//  |\|\
		//  X-X-X
		Polygon rSP;
		rSP.push_back({ 0.0, 0.0 });
		rSP.push_back({ 1.0, 0.0 });
		rSP.push_back({ 2.0, 0.0 });
		rSP.push_back({ 1.0, 1.0 });
		rSP.push_back({ 1.0, 0.0 });
		rSP.push_back({ 0.0, 1.0 });
		return rSP;
	}
};

TEST_F(HPolygonSetTest, empty_ctor_no_throws)
{
	ASSERT_NO_THROW(HPolygonSet{});
	EXPECT_EQ(0, HPolygonSet{}.size());
}

TEST_F(HPolygonSetTest, builds_from_relative_simple_polygon)
{
	HPolygonSet pS{ buildBowtiePolygon() };
	ASSERT_EQ(2, pS.size());
	{
		Polygon p;
		p.push_back({ 0.0, 0.0 });
		p.push_back({ 1.0, 0.0 });
		p.push_back({ 0.0, 1.0 });
		EXPECT_EQ(p, pS.getPolygonsWithHoles()[0]);
	}
	{
		Polygon p;
		p.push_back({ 1.0, 0.0 });
		p.push_back({ 2.0, 0.0 });
		p.push_back({ 1.0, 1.0 });
		EXPECT_EQ(p, pS.getPolygonsWithHoles()[1]);
	}
}

TEST_F(HPolygonSetTest, builds_from_collinear_polygon)
{
	Polygon p;
	p.push_back({ 0.0, 0.0 });
	p.push_back({ 1.0, 0.0 });
	p.push_back({ 0.0, 1.0 });
	p.push_back({ 0.0, 2.0 });

	HPolygonSet pS;
	ASSERT_NO_THROW(pS = HPolygonSet{ p });

	Polygon r;
	r.push_back({ 0.0, 0.0 });
	r.push_back({ 1.0, 0.0 });
	r.push_back({ 0.0, 1.0 });
	
	EXPECT_EQ(1, pS.size());
	EXPECT_EQ(r, pS.getPolygonsWithHoles()[0].outer_boundary());
}

TEST_F(HPolygonSetTest, splitToSimplePolygons_1)
{
	//  X X
	//  |\|\
	//  X-X-X
	//   /|
	//  X-X
	Polygon rSP;
	rSP.push_back({ 1.0,  0.0 });
	rSP.push_back({ 0.0,  1.0 });
	rSP.push_back({ 0.0,  0.0 });
	rSP.push_back({ 1.0,  0.0 });
	rSP.push_back({ 2.0,  0.0 });
	rSP.push_back({ 1.0,  1.0 });
	rSP.push_back({ 1.0,  0.0 });
	rSP.push_back({ 0.0, -1.0 });
	rSP.push_back({ 1.0, -1.0 });

	const HPolygonSet pS{ rSP };
	ASSERT_EQ(3, pS.size());
	{
		Polygon p;
		p.push_back({ 0.0, -1.0 });
		p.push_back({ 1.0, -1.0 });
		p.push_back({ 1.0,  0.0 });
		EXPECT_EQ(p, pS.getPolygonsWithHoles()[0].outer_boundary());
	}
	{
		Polygon p;
		p.push_back({ 1.0, 0.0 });
		p.push_back({ 2.0, 0.0 });
		p.push_back({ 1.0, 1.0 });
		EXPECT_EQ(p, pS.getPolygonsWithHoles()[1].outer_boundary());
	}
	{
		Polygon p;
		p.push_back({ 0.0, 0.0 });
		p.push_back({ 1.0, 0.0 });
		p.push_back({ 0.0, 1.0 });
		EXPECT_EQ(p, pS.getPolygonsWithHoles()[2].outer_boundary());
	}
}

TEST_F(HPolygonSetTest, splitToSimplePolygons_2)
{
	//  X-X
	//  |/  
	//  X X
	//  |\|\
	//  X-X-X
	Polygon rSP;
	rSP.push_back({ 0.0, 2.0 });
	rSP.push_back({ 0.0, 1.0 });
	rSP.push_back({ 0.0, 0.0 });
	rSP.push_back({ 1.0, 0.0 });
	rSP.push_back({ 2.0, 0.0 });
	rSP.push_back({ 1.0, 1.0 });
	rSP.push_back({ 1.0, 0.0 });
	rSP.push_back({ 0.0, 1.0 });
	rSP.push_back({ 1.0, 2.0 });

	const HPolygonSet pS{ rSP };
	ASSERT_EQ(3, pS.size());

	{
		Polygon p;
		p.push_back({ 0.0, 0.0 });
		p.push_back({ 1.0, 0.0 });
		p.push_back({ 0.0, 1.0 });
		EXPECT_EQ(p, pS.getPolygonsWithHoles()[0].outer_boundary());
	}
	{
		Polygon p;
		p.push_back({ 1.0, 0.0 });
		p.push_back({ 2.0, 0.0 });
		p.push_back({ 1.0, 1.0 });
		EXPECT_EQ(p, pS.getPolygonsWithHoles()[1].outer_boundary());
	}
}

TEST_F(HPolygonSetTest, join)
{

	Polygon p;
	p.push_back({ 0.00000000,  0.000000000 });
	p.push_back({ 4.00000000,  0.000000000 });
	p.push_back({ 3.20000005,  0.800000012 });
	p.push_back({ 0.800000012, 0.800000012 });
	p.push_back({ 0.800000012, 3.20000005 });
	p.push_back({ 0.00000000,  4.00000000 });

	HPolygonSet S;
	S.join(p);

	auto pwh{ S.getPolygonsWithHoles() };

	ASSERT_EQ(1, pwh.size());
	EXPECT_EQ(p, pwh.front().outer_boundary());
}

TEST_F(HPolygonSetTest, join_2)
{
	Polygon p;
	p.push_back({ 0.0,  0.0 });
	p.push_back({ 1.0,  0.0 });
	p.push_back({ 0.0,  1.0 });

	Polygon q;
	q.push_back({ 0.0,  0.0 });
	q.push_back({ 1.0,  0.0 });
	q.push_back({ 0.9,  0.9 });

	HPolygonSet S;
	S.join(p);
	S.join(q);

	auto pwh{ S.getPolygonsWithHoles() };

	ASSERT_EQ(1, pwh.size());
}

TEST_F(HPolygonSetTest, join_touching_vertex)
{
	Polygon p;
	p.push_back({ 0.0,  0.0 });
	p.push_back({ 1.0,  0.0 });
	p.push_back({ 0.0,  1.0 });

	Polygon q;
	q.push_back({ 1.0,  0.0 });
	q.push_back({ 2.0,  0.0 });
	q.push_back({ 0.0,  1.0 });

	HPolygonSet S;
	S.join(p);
	S.join(q);

	auto pwh{ S.getPolygonsWithHoles() };

	ASSERT_EQ(1, pwh.size());
}

TEST_F(HPolygonSetTest, join_and_substract)
{
	Polygon p;
	p.push_back({ 0.0,  0.0 });
	p.push_back({ 1.0,  0.0 });
	p.push_back({ 0.0,  1.0 });

	HPolygonSet S{ p };
	
	ASSERT_EQ(1, S.size());
	EXPECT_EQ(3, S.getPolygonsWithHoles().front().outer_boundary().size());

	S.difference(p);
	EXPECT_EQ(0, S.size());
}

TEST_F(HPolygonSetTest, degenerated_polygon)
{
	Polygon p;
	p.push_back({ 0.0,  0.0 });
	p.push_back({ 1.0,  0.0 });
	p.push_back({ 0.0,  1.0 });
	
	Polygon q;
	q.push_back({ 1.0,  1.0 });
	q.push_back({ 0.0,  1.0 });
	q.push_back({ 1.0,  0.0 });

	HPolygonSet S{p};
	S.intersection(q);

	ASSERT_EQ(0, S.getPolygonsWithHoles().size());
}

TEST_F(HPolygonSetTest, degenerated_polygon_2)
{
	Polygon p;
	p.push_back({ 19.0,               7.0 });
	p.push_back({ 20.0,               7.0 });
	p.push_back({ 20.0,               7.3372999999999999 });
	p.push_back({ 20.0,               8.0 });
	p.push_back({ 19.0020193911317,   8 });
	p.push_back({ 19.0,               7.9905239130434733 });
	p.push_back({ 19.0,               7.8846909090909003 });
	p.push_back({ 19.001999999999999, 7.8335999999999997 });
	p.push_back({ 19.0,               7.8846909090909003 });

	EXPECT_EQ(1, HPolygonSet{ p }.size());
}


TEST_F(HPolygonSetTest, simplify_collinears)
{
	Polygon p1;
	p1.push_back({  0.0, 0.0 });
	p1.push_back({  1.0, 0.0 });
	p1.push_back({  0.0, 1.0 });
	Polygon p2;
	p2.push_back({  0.0, 0.0 });
	p2.push_back({  0.0, 1.0 });
	p2.push_back({ -1.0, 0.0 });

	HPolygonSet pS{ p1 };
	pS.join(p2);

	ASSERT_EQ(1, pS.size());
	EXPECT_EQ(4, pS.getPolygonsWithHoles()[0].outer_boundary().size());

	pS = pS.simplifyCollinears();
	ASSERT_EQ(1, pS.size());
	EXPECT_EQ(3, pS.getPolygonsWithHoles()[0].outer_boundary().size());
}