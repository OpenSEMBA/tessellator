#include "MeshTest.h"

#ifdef TESSELLATOR_BOOST
TEST_F(MeshTest, serialization_deserialization) {
    
    Mesh get = buildMesh();
    const char* filename = "serialization_deserialization.txt";

    {
        std::ofstream ofs(filename);
        boost::archive::text_oarchive oa(ofs);
        oa << get;
    }

    Mesh readMesh;
    {
        std::ifstream ifs(filename);
        boost::archive::text_iarchive ia(ifs);
        ia >> readMesh;
    }

    EXPECT_EQ(get, readMesh);
}
#endif

TEST_F(MeshTest, identifiesHexahedron) {
    Element hexahedron(
        {0, 1, 2, 3, 4, 5, 6, 7},
        Element::Type::Volume);
    Element surface(
        {0, 1, 2, 3, 4, 5, 6, 7},
        Element::Type::Surface);

    EXPECT_TRUE(hexahedron.isHexahedron());
    EXPECT_FALSE(surface.isHexahedron());
}

