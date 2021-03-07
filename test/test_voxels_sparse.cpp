#include <gtest/gtest.h>

#include "VoxelSparse.h"

using namespace std;

TEST(VoxelSparse, construction)
{
    CGAL::Bbox_3 bbox(0., 0., 0., 1., 1., 1.);
    double voxelSide = 0.5;

    VoxelSparse sparseTensor(bbox, voxelSide);

    // Test points and normals
    Point p1(0.75, 0.25, 0.25);
    Vector n1(1., 0., 0.);

    Point p2(0.75, 0.25, 0.75);
    Vector n2(0., 0., 1.);

    Point p3(0.80, 0.26, 0.24);
    Vector n3(0., 1., 0.);

    vector<Point> pointCloud = {p1, p2, p3};
    vector<Vector> normals = {n1, n2, n3};
    sparseTensor.computeFeaturesFromPointCloud(pointCloud, normals);

    VoxelSparse::SparseVal features = sparseTensor.normalizedValues();

    ASSERT_EQ(sparseTensor.coord().size(), 2);
    ASSERT_EQ(get<0>(sparseTensor.coord()[0]), 1);
    ASSERT_EQ(get<1>(sparseTensor.coord()[0]), 0);
    ASSERT_EQ(get<2>(sparseTensor.coord()[0]), 0);
    ASSERT_EQ(get<0>(sparseTensor.coord()[1]), 1);
    ASSERT_EQ(get<1>(sparseTensor.coord()[1]), 0);
    ASSERT_EQ(get<2>(sparseTensor.coord()[1]), 1);
    ASSERT_DOUBLE_EQ(features[0][0], 1./sqrt(2.));
    ASSERT_DOUBLE_EQ(features[0][1], 1./sqrt(2.));
    ASSERT_DOUBLE_EQ(features[0][2], 0.);
    ASSERT_DOUBLE_EQ(features[1][0], 0.);
    ASSERT_DOUBLE_EQ(features[1][1], 0.);
    ASSERT_DOUBLE_EQ(features[1][2], 1.);
}