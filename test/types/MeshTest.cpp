#include "MeshTest.h"

#ifdef TESSELLATOR_BOOST
TEST_F(DMesheRTypesMeshTest, serialization_deserialization) {
    
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


