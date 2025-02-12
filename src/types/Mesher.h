#pragma once

#include "Mesh.h"

namespace meshlib {

class Mesher {
public:
    Mesher() = default;
    virtual ~Mesher() = default;

    virtual Mesh mesh() const = 0;
    virtual bool isStructured() const = 0;  

};

}