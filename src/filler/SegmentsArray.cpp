#include "SegmentsArray.h"

namespace meshlib {
namespace tessellator {
namespace filler {

using namespace cgal;

Segment1 buildCellEdge(const CellDir& cD)
{
	Point1 ini{ (KType) cD };
	Point1 end{ (KType) cD + 1 };
	return Segment1{ ini, end };
}

Segment1 buildSegmentE1FromOverlap(const Segment1& q, const Segment1& s)
{
	Segment1 res;
	q[0] < s[0]?
		res[0] = s[0] :
		res[0] = q[0];
	q[1] < s[1]?
		res[1] = q[1] :
		res[1] = s[1];
	return res;
}

bool doOverlap(const Segment1& q, const Segment1& s)
{
	if (q[1] < s[0]) {
		return false;
	}
	else if (q[0] > s[1]) {
		return false;
	}
	return true;
}

EdgeFilling Segments::getEdgeFilling(const CellDir& c) const 
{
	auto q{ buildCellEdge(c) };
	EdgeFilling res;
	for (const auto& [pr, segs] : prSeg_) {
		for (const auto& s : segs) {
			if (doOverlap(q, s)) {
				auto lin{ buildSegmentE1FromOverlap(q, s) };
				if (lin[0] != lin[1]) {
					res.lins[pr].push_back(buildSegmentE1FromOverlap(q, s));
				}
			}
		}
	}
	return res;
}

bool pointIsInSegment(
	const Point1 & p,
	const Segment1 & s)
{
	return p >= std::min(s[0], s[1]) && p <= std::max(s[0], s[1]);
}

std::set<Segment1> intersectSegmentWithPoints(
	const Segment1& seg,
	const std::vector<Point1>& points)
{
	std::set<Segment1> res;
	Point1 start{ seg[0] };
	for (const auto& p : points) {
		if (pointIsInSegment(p, seg) && p != seg[0]) {
			res.insert(Segment1{start, p});
			start = p;
		}
	}
	return res; 
}

std::vector<Point1> pointsOnEdge(
	const std::map<Priority, Segments1, std::greater<Priority>>& prSeg)
{
	auto comp = [](KType c1, KType c2) {return c1 < c2;};
	std::set<KType, decltype(comp)> idsOnEdge(comp);
	for (const auto& [p, segs] : prSeg) {
		for (const auto& s : segs) {
			idsOnEdge.insert(s[0]);
			idsOnEdge.insert(s[1]);			
		}
	}
	std::vector<Point1> res;
	for (const auto& c : idsOnEdge) {
		res.push_back(c);
	}
	return res;
}

Segments1 collapseAdjacentSegments(const std::set<Segment1>& segs)
{
	if (segs.empty()) {
		return Segments1{};
	}
	Segments1 res;
	auto start{ (*segs.begin())[0] };
	auto end{ (*segs.begin())[1] };
	for (auto it{ ++segs.begin() }; it != segs.end(); it++){
		if (end == (*it)[0]) {
			end = (*it)[1];
		}
		else {
			res.push_back(Segment1{ start,end });
			start = (*it)[0];
			end = (*it)[1];
		}
	}
	res.push_back(Segment1{ start,end });
	return res;
}

void Segments::add(const Priority& pr, const Segments1& nS)
{
	prSeg_.emplace(pr, nS);
	std::map < Priority, Segments1, std::greater<Priority>> orderedPrSeg{ prSeg_.begin(), prSeg_.end() };
	std::map<Priority, std::set<Segment1> > prInSeg;
	for (const auto& [p, segs] : orderedPrSeg) {
		for (const auto& s : segs) {
			auto intersectedSeg{ intersectSegmentWithPoints(s, pointsOnEdge(orderedPrSeg)) };
			auto intersectedSegToClip{ intersectedSeg };
			for (const auto& mapLine : prInSeg) {
				for (const auto& newSeg : intersectedSeg) {
					if (mapLine.second.find(newSeg) != mapLine.second.end()) {
						intersectedSegToClip.erase(newSeg);
					}
				}
			}
			auto it{ prInSeg.find(p) };
			if (it == prInSeg.end()) {
				prInSeg.emplace(p, intersectedSegToClip);
			}
			else {
				prInSeg[p].insert(intersectedSegToClip.begin(), intersectedSegToClip.end());
			}

		}
	}
	for (const auto& [p, segs] : prInSeg) {
		Segments1 coll{ collapseAdjacentSegments(segs) };
		prSeg_[p] = Segments1{ coll.begin(), coll.end() };
	}
}

}
}
}