#include "ConvexHull.h"
#include "utils/Geometry.h"

#include <numeric>

namespace meshlib::utils {

struct Vector2D {
	double x;
	double y;

	Vector2D operator-(Vector2D r) {
		return { x - r.x, y - r.y };
	}
	double operator*(const Vector2D& r) const {
		return x * r.x + y * r.y;
	}
	Vector2D rotate90() {  // Rotate 90 degrees counter-clockwise
		return { -y, x };
	}
	double manhattan_length() {
		return abs(x) + abs(y);
	}
	bool operator==(const Vector2D& r) const {
		return x == r.x && y == r.y;
	}
	bool operator!=(const Vector2D& r) const {
		return x != r.x || y != r.y;
	}
	bool operator<(const Vector2D& r) const {
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

std::vector<Vector2D> graham_scan(std::vector<Vector2D> points) {
	Vector2D first_point = *std::min_element(points.begin(), points.end(), [](Vector2D& left, Vector2D& right) {
		return std::make_tuple(left.y, left.x) < std::make_tuple(right.y, right.x);
		});  // Find the lowest and leftmost point

	std::sort(points.begin(), points.end(), [&](Vector2D& left, Vector2D& right) {
		if (left == first_point) {
			return right != first_point;
		}
		else if (right == first_point) {
			return false;
		}
		double dir = (left - first_point).rotate90() * (right - first_point);
		if (dir == 0) {  // If the points are on a line with first point, sort by distance (manhattan is equivalent here)
			return (left - first_point).manhattan_length() < (right - first_point).manhattan_length();
		}
		return dir > 0;
		// Alternative approach, closer to common algorithm formulation but inferior:
		// return atan2(left.y - first_point.y, left.x - first_point.x) < atan2(right.y - first_point.y, right.x - first_point.x);
		});  // Sort the points by angle to the chosen first point

	std::vector<Vector2D> result;
	for (auto pt : points) {
		// For as long as the last 3 points cause the hull to be non-convex, discard the middle one
		while (result.size() >= 2 &&
			(result[result.size() - 1] - result[result.size() - 2]).rotate90() * (pt - result[result.size() - 1]) <= 0) {
			result.pop_back();
		}
		result.push_back(pt);
	}
	return result;
}

std::map<Vector2D, CoordinateId> buildPointsInIndex(
	const Coordinates& globalCoords,
	const IdSet& inIds)
{
	Coordinates cs;
	cs.reserve(inIds.size());
	std::vector<CoordinateId> originalIds;
	originalIds.reserve(inIds.size());

	for (auto const& id : inIds) {
		cs.push_back(globalCoords[id]);
		originalIds.push_back(id);
	}
	utils::Geometry::rotateToXYPlane(cs.begin(), cs.end());

	std::map<Vector2D, CoordinateId> res;
	for (std::size_t i = 0; i < cs.size(); i++) {
		res.emplace(Vector2D{ cs[i](0), cs[i](1) }, originalIds[i]);
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

std::vector<CoordinateId> ConvexHull::get(const IdSet& ids) const
{
	assert(ids.size() > 1);
	auto pointsIndex{ buildPointsInIndex(*globalCoords_, ids) };

	std::vector<CoordinateId> res;

	return res;
}


}