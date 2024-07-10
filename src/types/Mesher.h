#pragma once

#include "Mesh.h"
#include "Map.h"

namespace meshlib {

class Mesher {
public:
    Mesher() = default;
    virtual ~Mesher() = default;

    virtual Mesh mesh() const = 0;
    virtual Map getMap() const = 0;
    virtual bool isStructured() const = 0;
    
    static Mesh getNotMeshedElems(const Mesh& input, const Map& map) 
    {
        Mesh res;
        res.coordinates = input.coordinates;
        res.groups.resize(input.groups.size());
        for (GroupId g = 0; g < input.groups.size(); g++) {
            for (ElementId e = 0; e < input.groups[g].elements.size(); e++) {
                if (map.groups[g].elements[e].empty()) {
                    res.groups[g].elements.push_back(input.groups[g].elements[e]);
                }
            }
        }
        return res;
    }
};

}