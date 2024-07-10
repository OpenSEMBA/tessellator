#pragma once

#include <map>
#include <set>
#include <vector>
#include <algorithm>
#include <functional>

#include <boost/serialization/array.hpp>
#include <boost/serialization/vector.hpp>

#include "Vector.h"

namespace meshlib {

typedef Vector<double>                Coordinate;
typedef Coordinate::Type              CoordinateDir;
typedef std::size_t                   CoordinateId;
typedef std::vector<Coordinate> Coordinates;

typedef std::array<std::vector<CoordinateDir>, 3> Grid;

struct Element {
    enum class Type {
        None,
        Node,
        Line,
        Surface,
        Volume
    };
    
    std::vector<CoordinateId> vertices;
    Type type = Type::None;

    Element() = default;
    Element(const std::vector<CoordinateId>& v, const Type& t = Type::Surface) :
        vertices(v),
        type(t)
    {
    }

    bool isNone() const
    {
        return type == Type::None && vertices.size() == 0;
    }


    bool isLine() const 
    {
        return type == Type::Line && vertices.size() == 2;
    }

    bool isTriangle() const 
    {
        return type == Type::Surface && vertices.size() == 3;
    }

    bool isTetrahedron() const 
    {
        return type == Type::Volume && vertices.size() == 4;
    }

    bool sharesVertices(const Element& rhs) 
    {
        bool res = true;
        for (const auto& cId : vertices) {
            auto it = std::find(rhs.vertices.begin(), rhs.vertices.end(), cId);
            res &= (it != rhs.vertices.end());
        }
        return res;
    }

    bool operator==(const Element& rhs) const {
        bool res = true;
        res &= type == rhs.type;
        res &= vertices == rhs.vertices;
        return res;
    }

    bool operator<(const Element& rhs) const {
        if (vertices[0] < rhs.vertices[0]) {
            return true;
        }
        else if (vertices[0] == rhs.vertices[0]) {
            if (vertices[1] < rhs.vertices[1]) {
                return true;
            }
            else if (vertices[1] == rhs.vertices[1]) {
                if (vertices[2] < rhs.vertices[2]) {
                    return true;
                }
                else {
                    return false;
                }

            }
            else 
            { 
                return false; 
            }

        }
        else {
            return false;
        }
        
    }

    struct compareVertices {
        bool operator()(Element a, Element b) const {
            return a.vertices < b.vertices;
        }
    };

private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar& type;
        ar& vertices;
    }

};
typedef std::size_t ElementId;
typedef std::vector<Element> Elements;

struct Group {
    std::vector<Element> elements;

    bool operator==(const Group& rhs) const {
        return elements == rhs.elements;
    }

    std::map<CoordinateId, std::vector<ElementId>> buildCoordToElemMap() const {
        std::map<CoordinateId, std::vector<ElementId>> vToElem;
        for (auto const& e : elements) {
            ElementId eId = &e - &elements.front();
            for (auto const& vId : e.vertices) {
                vToElem[vId].push_back(eId);
            }
        }
        return vToElem;
    }

private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar& elements;
    }
};
typedef std::size_t GroupId;
typedef std::pair<GroupId, ElementId> GroupElementId;
typedef std::vector<Group> Groups;

struct Mesh {
    Grid grid;
    Coordinates coordinates;
    Groups groups;

    bool operator==(const Mesh& rhs) const {
        bool res = true;
        res &= grid == rhs.grid;
        res &= coordinates == rhs.coordinates;
        res &= groups == rhs.groups;
        return res;
    }

    bool emptyOfElements() const 
    {
        return countElems() == 0;
    }

    std::size_t countElems() const {
        std::size_t res = 0;
        for (auto const& g: groups) {
            res += g.elements.size();
        }
        return res;
    }

    std::size_t countElemsWithCondition(std::function<bool(const Element&)> condition) const {
        std::size_t res = 0;
        for (auto const& g : groups) {
            res += std::count_if(g.elements.begin(), g.elements.end(), condition);
        }
        return res;
    }

    std::size_t countTriangles() const 
    {
        std::size_t res = 0;
        for (auto const& g: groups) {
            for (auto const& e : g.elements) {
                if (e.vertices.size() == 3 && e.type == Element::Type::Surface) {
                    res++;
                }
            }
        }
        return res;
    }
    std::size_t countLines() const 
    {
        std::size_t res = 0;
        for (auto const& g : groups) {
            for (auto const& e : g.elements) {
                if (e.vertices.size() == 2 && e.type == Element::Type::Line) {
                    res++;
                }
            }
        }
        return res;
    }

    std::map<CoordinateId, std::vector<GroupElementId>> buildCoordToElemMap() const {
        std::map<CoordinateId, std::vector<GroupElementId>> vToElem;
        for (auto const& g : groups) {
            GroupId gId = &g - &groups.front();
            for (auto const& e : g.elements) {
                ElementId eId = &e - &g.elements.front();
                for (auto const& vId : e.vertices) {
                    vToElem[vId].push_back(std::make_pair(gId, eId));
                }
            }
        }
        return vToElem;
    }

private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar& grid;
        ar& coordinates;
        ar& groups;
    }

};

}

