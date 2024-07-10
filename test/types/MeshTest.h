#ifndef MESH_TEST_H_
#define MESH_TEST_H_

#include <fstream>

#include "gtest/gtest.h"

#include "Mesh.h"

using namespace meshlib;

class DMesheRTypesMeshTest : public ::testing::Test {
protected:
	static Mesh buildMesh() {
        Grid grid;
        grid[0] = { 0.0, 1.0 };
        grid[1] = { 0.0, 1.0 };
        grid[2] = { 0.0, 1.0 };

        std::vector<Coordinate> coords(3);
        coords[0] = Coordinate{ {0.2, 0.2, 0.1} };
        coords[1] = Coordinate{ {0.8, 0.4, 0.1} };
        coords[2] = Coordinate{ {0.8, 0.2, 0.1} };

        std::vector<Group> groups(1);
        groups[0] = {
                std::vector<Element>(1)
        };
        Element tri;
        tri.vertices = { 0, 1, 2 };
        groups[0].elements[0] = tri;

        return Mesh{ grid, coords, groups };
	}

};

#endif
