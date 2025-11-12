#include "utils/GridTools.h"


namespace meshlib {
namespace core {

class Splicer : public utils::GridTools{
public:
    Splicer(const Mesh&);

    Mesh getMesh() const { return this->mesh_; };
private:
    void spliceTrianglesInGroup(Group& elementGroup, const Group& inputGroup);
    std::vector<std::map<double, CoordinateId>> getInnerPointIds(const Element&);
    bool isBetween(const Coordinate& leftExtreme, const Coordinate& rightExtreme, const Coordinate& betweenPoint);
    Elements spliceTriangleIntoList(const Element& triangle, std::size_t vertexPosition, const std::map<double, CoordinateId>& innerPoints);

    Mesh mesh_;
};

}
}

