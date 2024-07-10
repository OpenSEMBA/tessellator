#pragma once

#include "Types.h"

#include <functional>

namespace meshlib {

template<class T>
std::vector<const T*> getView(const std::vector<T>& elems) {
	std::vector<const T*> elemPtrs;
    elemPtrs.reserve(elems.size());
	for (auto const& elem : elems) {
		elemPtrs.push_back(&elem);
	}
	return elemPtrs;
}


std::pair<IdSet, IdSet> classifyIds(
    const IdSet& ids,
    const std::function<bool(const CoordinateId&)> condition);

IdSet mergeIds(const IdSet& s1, const IdSet& s2);

template <class T>
IdSet intersectWithIdSet(const IdSet& aSet, const T& bSet)
{
    IdSet r;
    for (auto const& a : aSet) {
        if (bSet.count(a) != 0) {
            r.insert(a);
        }
    }
    return r;
}

}

