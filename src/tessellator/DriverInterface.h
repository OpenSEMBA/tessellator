#pragma once

#include "types/Mesh.h"

namespace meshlib {
    namespace tessellator {

        class DriverInterface {
        public:
            virtual ~DriverInterface() = default;
            virtual Mesh mesh() const abstract = 0;

        protected:
            virtual void process(Mesh&) const abstract = 0;

        };

    }
}