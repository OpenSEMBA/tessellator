#pragma once

#include <vector>

#include "utils/Types.h"

namespace meshlib {

struct Map {
    typedef std::vector<CoordinateId> Coordinate;
    typedef std::vector<ElementId>    Element;

    std::size_t countElems() const {
        std::size_t res = 0;
        for (auto const& g: groups) {
            res += g.elements.size();
        }
        return res;
     }

    struct Group {
        std::vector<Element> elements;
    private:
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int version) {
            ar& elements;
        }
    };
    std::vector<Coordinate> coordinates;
    std::vector<Group>      groups;
private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar& groups;
        ar& coordinates;
    }
};

}

