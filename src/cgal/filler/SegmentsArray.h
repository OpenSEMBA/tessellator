#pragma once

#include "FillerTools.h"

namespace meshlib {
namespace tessellator {
namespace filler {

using namespace cgal;

using PrSegmentsMap = std::map<Priority, Segments1>;

struct EdgeFilling {
	PrSegmentsMap lins;
	bool operator==(const EdgeFilling& rhs) const {
		return lins == rhs.lins;
	}
};

class Segments {
public:
	EdgeFilling getEdgeFilling(const CellDir&) const;

	void add(const Priority& p, const Segments1& s);
private:
	PrSegmentsMap prSeg_;
};

}
}
}