#include "gtest/gtest.h"
#include "MeshFixtures.h"

#include "cgal/PolyhedronTools.h"
#include "MeshTools.h"

#include <CGAL/Surface_mesh.h>

#include <CGAL/Cartesian.h>
#include <CGAL/Arr_non_caching_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_naive_point_location.h>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arr_segment_traits_2.h>

using namespace meshlib;

using namespace meshFixtures;

using namespace cgal;

class ArrangmentTest : public ::testing::Test {
public:
    using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
    using Number_type = Kernel::FT;
    
    using Traits = CGAL::Arr_segment_traits_2<Kernel>;
    using Point = Traits::Point_2;
    using Segment = Traits::X_monotone_curve_2;
    
    using Arrangement = CGAL::Arrangement_2<Traits>;
    using Vertex_handle = Arrangement::Vertex_handle;
    using Halfedge_handle = Arrangement::Halfedge_handle;
    using Face_handle = Arrangement::Face_handle;
    using Vertex_const_handle = Arrangement::Vertex_const_handle;
    using Halfedge_const_handle = Arrangement::Halfedge_const_handle;

    Coordinate point2ToCoordinate(const Point& p)
    {
        return Coordinate{ {CGAL::to_double(p[0]),CGAL::to_double(p[1])} };
    }

    Coordinates print_ccb(Arrangement::Ccb_halfedge_const_circulator circ) {
        Coordinates face;
        auto curr = circ;
        do {
            face.push_back(point2ToCoordinate(curr->target()->point()));
        } while (++curr != circ);
        return face;
    }

    Coordinates print_face(Arrangement::Face_const_handle f) {
        if (f->is_unbounded()) std::cout << "Unbounded face.\n";
        else {
            std::cout << "Outer boundary: ";
            return print_ccb(f->outer_ccb());
        }
        return Coordinates{};
    }
};

TEST_F(ArrangmentTest, arrangment_4faces_complex)
{

//            4 
//          /   \  
//         /     |_ 
//     2  /        |_
//      \/    5      \
//      /\___/ ______ 7 
//     /    /\/      /
//    /    /_/ \_   /
//   /    3      \ /
//  1-------------6


    Point p1(1, 1), p2(1, 4), p3(2, 2), p4(3, 7), p5(4, 4), p6(7, 1), p7(9, 3);
    Arrangement arr;
    Segment s(p1, p6);
    Arrangement::Halfedge_handle e = insert_non_intersecting_curve(arr, s);
    Arrangement::Vertex_handle v0 = e->source();
    insert(arr, Segment(p1, p4));  insert(arr, Segment(p2, p6));
    insert(arr, Segment(p3, p7));  insert(arr, Segment(p3, p5));
    insert(arr, Segment(p6, p7));  insert(arr, Segment(p4, p7));

    ASSERT_EQ(1, arr.number_of_unbounded_faces());
    auto numOfBoundedFaces{ arr.number_of_faces() - arr.number_of_unbounded_faces() };
    EXPECT_EQ(4, numOfBoundedFaces);
    ASSERT_EQ(10, arr.number_of_vertices());

    std::vector<Coordinates> faces;

    for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
        if (!fit->is_unbounded()) {
            faces.push_back(print_face(fit));
        }
    }
    EXPECT_EQ(4, faces.size());


}

TEST_F(ArrangmentTest, arrangment_3faces) {
    // 2           3
    // +------------+
    // | \        / |
    // |  |      |  |
    // |   \    /   |
    // |    |  |    |
    // |     \/     |
    // +------------+
    // 1      5     4

    Arrangement arr;
    Point p1(1, 1), p2(1, 2), p3(2, 2), p4(2, 1), p5(1.5,1);
    Segment cv[] = {
        Segment(p1, p2),
        Segment(p2, p3),
        Segment(p3, p4),
        Segment(p4, p1),
        Segment(p5, p2),
        Segment(p5, p3) };
    CGAL::insert(arr, &cv[0], &cv[sizeof(cv) / sizeof(Segment)]);

    ASSERT_EQ(1, arr.number_of_unbounded_faces());
    auto numOfBoundedFaces{ arr.number_of_faces() - arr.number_of_unbounded_faces() };
    EXPECT_EQ(3, numOfBoundedFaces);
    ASSERT_EQ(5, arr.number_of_vertices());

    std::vector<Coordinates> faces;
    for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
        if (!fit->is_unbounded()) {
            faces.push_back(print_face(fit));
        }
    }
    EXPECT_EQ(3, faces.size());
    EXPECT_EQ(3, faces[0].size());
    EXPECT_EQ(3, faces[1].size());
    EXPECT_EQ(3, faces[2].size());

}

TEST_F(ArrangmentTest, arrangment_1face) {
    // 2            3
    // +------------+
    // |            |
    // |            |
    // |            |
    // |            |
    // |            |
    // +------------+
    // 1            4

    Arrangement arr;
    Point p1(1, 1), p2(1, 2), p3(2, 2), p4(2, 1);
    Segment cv[] = {
        Segment(p1, p2),
        Segment(p2, p3),
        Segment(p3, p4),
        Segment(p4, p1) };
    CGAL::insert(arr, &cv[0], &cv[sizeof(cv) / sizeof(Segment)]);

    ASSERT_EQ(1, arr.number_of_unbounded_faces());
    auto numOfBoundedFaces{ arr.number_of_faces() - arr.number_of_unbounded_faces() };
    EXPECT_EQ(1, numOfBoundedFaces);
    ASSERT_EQ(4, arr.number_of_vertices());

    std::vector<Coordinates> faces;
    for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
        if (!fit->is_unbounded()) {
            faces.push_back(print_face(fit));
        }
    }
    EXPECT_EQ(1, faces.size());
    EXPECT_EQ(4, faces.front().size());


 

}