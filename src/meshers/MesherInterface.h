#pragma once

#include "types/Mesh.h"

namespace meshlib::meshers {

    class MesherInterface {
    public:
        virtual ~MesherInterface() = default;
        virtual Mesh mesh() const = 0;

    protected:
        virtual void process(Mesh&) const = 0;

    };

}