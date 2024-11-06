#include "ConvexHull.h"
#include "utils/Geometry.h"

#include <numeric>

namespace meshlib::utils {

struct Point {
	double x, y;
	bool operator== (const Point& t) const 
	{
		return x == t.x && y == t.y;
	}

	bool operator<(const Point& r) const 
	{
		if (x < r.x) {
			return true;
		}
		else if (x > r.x) {
			return false;
		}
		else {
			return y < r.y;
		}
	}
};

int orientation(Point a, Point b, Point c) 
{
	double v = a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y);
	if (v < 0) return -1; // clockwise
	if (v > 0) return +1; // counter-clockwise
	return 0;
}

bool cw(Point a, Point b, Point c, bool include_collinear) 
{
	int o = orientation(a, b, c);
	return o < 0 || (include_collinear && o == 0);
}

bool collinear(Point a, Point b, Point c) 
{ 
	return orientation(a, b, c) == 0; 
}

void grahamScan(std::vector<Point>& a, bool include_collinear = false) 
{
	Point p0 = *std::min_element(
		a.begin(), 
		a.end(), [](Point a, Point b) {
			return std::make_pair(a.y, a.x) < std::make_pair(b.y, b.x);
		}
	);

	std::sort(a.begin(), a.end(), [&p0](const Point& a, const Point& b) {
		int o = orientation(p0, a, b);
		if (o == 0)
			return (p0.x - a.x) * (p0.x - a.x) + (p0.y - a.y) * (p0.y - a.y)
			< (p0.x - b.x) * (p0.x - b.x) + (p0.y - b.y) * (p0.y - b.y);
		return o < 0;
		});
	if (include_collinear) {
		int i = (int)a.size() - 1;
		while (i >= 0 && collinear(p0, a[i], a.back())) i--;
		reverse(a.begin() + i + 1, a.end());
	}

	std::vector<Point> st;
	for (int i = 0; i < (int)a.size(); i++) {
		while (st.size() > 1 && !cw(st[st.size() - 2], st.back(), a[i], include_collinear)) {
			st.pop_back();
		}
		st.push_back(a[i]);
	}

	if (include_collinear == false && st.size() == 2 && st[0] == st[1])
		st.pop_back();

	a = st;
}

std::map<Point, CoordinateId> buildPointsInIndex(
	const Coordinates& globalCoords,
	const IdSet& inIds,
	const VecD& normalVec)
{
	Coordinates cs;
	cs.reserve(inIds.size());
	std::vector<CoordinateId> originalIds;
	originalIds.reserve(inIds.size());

	for (auto const& id : inIds) {
		cs.push_back(globalCoords[id]);
		originalIds.push_back(id);
	}
	Geometry::rotateToXYPlane(cs.begin(), cs.end(), normalVec);

	std::map<Point, CoordinateId> res;
	for (std::size_t i = 0; i < cs.size(); i++) {
		res.emplace(Point{ cs[i](0), cs[i](1) }, originalIds[i]);
	}
	return res;
}

ConvexHull::ConvexHull(const Coordinates* global)
{
	if (global == nullptr) {
		throw std::runtime_error("Global list of coordinates must be defined");
	}
	globalCoords_ = global;
}

std::vector<CoordinateId> ConvexHull::get(const IdSet& ids, const VecD& normalVec) const
{
	if (ids.size() <= 2) {
		return std::vector<CoordinateId>(ids.begin(), ids.end());
	}

	auto pointsIndex{ buildPointsInIndex(*globalCoords_, ids, normalVec) };
	if (pointsIndex.size() <= 2) {
		std::vector<CoordinateId> res;
		for (const auto& point : pointsIndex) {
			res.push_back(point.second);
		}
		return res;
	}
		

	std::vector<Point> points(pointsIndex.size());
	auto it{ pointsIndex.begin() };
	for (std::size_t i{ 0 }; i < points.size(); ++i) {
		points[i] = it->first;
		++it;
	}

	grahamScan(points, true);

	std::vector<CoordinateId> res;
	res.reserve(points.size());
	for (const auto& point : points) {
		CoordinateId id{ pointsIndex.find(point)->second };
		res.push_back(id);
	}

	return res;
}


}