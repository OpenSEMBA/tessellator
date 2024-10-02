#pragma once

#include "types/Mesh.h"

namespace meshlib {
    namespace tessellator {

        class StructuredDriver {
        public:
            StructuredDriver(const Mesh& in, int decimalPlacesInCollapser = 4);
            virtual ~StructuredDriver() = default;
            Mesh mesh() const;

        private:
            int decimalPlacesInCollapser_;

            Mesh surfaceMesh_;
            Grid originalGrid_;
            Grid enlargedGrid_;

            void process(Mesh&) const;

        };

    }
}
