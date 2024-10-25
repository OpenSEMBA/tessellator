#pragma once

#include "FillerTools.h"
#include "cgal/HPolygonSet.h"

#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>

namespace meshlib {
namespace tessellator {
namespace filler {

using namespace cgal;

enum class FillingType {
	Empty,
	Partial,
	Full
};

class FillingState {
public:
	FillingType type;

	FillingState(const FillingType& t);
	FillingState(const Priority& p);
	Priority getPriority() const;

	bool empty() const { return type == FillingType::Empty; }
	bool partial() const { return type == FillingType::Partial; }
	bool full() const { return type == FillingType::Full; }
private:
	Priority priority_;
};


struct FaceFilling {
	std::map<Priority, Polylines2> lins;
	std::map<Priority, HPolygonSet> tris;

	bool operator==(const FaceFilling& rhs) const {
		return lins == rhs.lins 
			&& tris == rhs.tris;
	}

	HPolygonSet allSurfaces() const { 
		HPolygonSet res;
		for (const auto& t: tris) {
			res.join(t.second);
		}
		return res;
	}
	HPolygonSet metalSurfaces() const { 
		HPolygonSet res;
		for (const auto& [pr, pols] : tris) {
			if (pr > 0) {
				res.join(pols);
			}
		}
		return res;
	}
	HPolygonSet dielectricSurfaces() const { 
		HPolygonSet res;
		for (const auto& [pr, pols] : tris) {
			if (pr < 0) {
				res.join(pols);
			}
		}
		return res;
	}
};

class Slice {
public:
	using ContourIndexSet = boost::unordered_set<ArrayIndex>;
	using SurfaceMap = boost::unordered_map<ArrayIndex, std::vector<const CDT::Face*>>;
	using LineMap = boost::unordered_map<ArrayIndex, std::vector<Polyline2::const_iterator>>;
	using LineMaps = std::vector<LineMap>;

	Slice() = default;
	Slice& operator=(const Slice&) = delete;

	FaceFilling getFaceFilling(const ArrayIndex&) const;
	LinVs buildAllLinVs(const Priority&, Axis, Height) const;
	TriVs buildAllTriVs(const Priority&, Axis, Height) const;
	FillingState getFillingState(const ArrayIndex&) const;

	void add(const Polylines2&, const Priority&);
	void addAsPolygon(const Polylines2&, const Priority&);
	void add(const HPolygonSet&, const Priority&);
	void mergeLines(const Slice& lhs);
	void buildSearchMap();
	void buildTriangulations();
	void simplifySurfaces();
	void cleanSurfaces();
private:
	struct SliceData {
		Polylines2 lines;
		HPolygonSet surfaces;
		
		CDTs triangulations;
		SurfaceMap trianglesMaps;
		LineMaps lineMaps;

		SliceData() = default;
		SliceData& operator=(const SliceData&) = delete;
		void buildSurfaceMaps();
		void buildLineMaps();
		void buildMaps();
		bool isEmpty() const;
	};

	std::map<Priority, SliceData> data_;
	ContourIndexSet nonEdgeAlignedContourIndices_;
		
	void fillSurfaces(FaceFilling&, const ArrayIndex&) const;
	void fillLines(FaceFilling&, const ArrayIndex&) const;
	void removeInSuperiorPriorities(const Priority& pr);
};

}
}
}
