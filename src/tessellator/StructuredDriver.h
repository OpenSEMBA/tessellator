#pragma once

#include "types/Mesh.h"
#include "DriverBase.h"

namespace meshlib {
    namespace tessellator {

        class StructuredDriver: public DriverBase {
        public:
            StructuredDriver(const Mesh& in, int decimalPlacesInCollapser = 4);
            virtual ~StructuredDriver() = default;
            Mesh mesh() const;

        private:
            int decimalPlacesInCollapser_;

            Mesh surfaceMesh_;

            virtual Mesh buildSurfaceMesh(const Mesh& inputMesh, const Mesh& volumeSurface);
            void process(Mesh&) const;

        };

    }
}
