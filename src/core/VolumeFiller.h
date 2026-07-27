#pragma once

#include "utils/GridTools.h"

namespace meshlib{
namespace core{

class VolumeFiller : public utils::GridTools{
public:

    VolumeFiller(const Mesh&);
    Mesh getMesh();
private:
    Mesh mesh_;
};

}
}