#include <gtest/gtest.h>

#include "sparseToDense.h"

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

    Point p4(1.80, 0.26, 0.24);
    Vector n4(0., 1., 0.);

    vector<Point> pointCloud = {p1, p2, p3, p4};
    vector<Vector> normals = {n1, n2, n3, n4};
    sparseTensor.computeFeaturesFromPointCloud(pointCloud, normals);

    VoxelSparse::SparseVal features = sparseTensor.normalizedValues();

    ASSERT_EQ(sparseTensor.coord().size(), 2);
    ASSERT_EQ(sparseTensor.coord()[0][0], 1);
    ASSERT_EQ(sparseTensor.coord()[0][1], 0);
    ASSERT_EQ(sparseTensor.coord()[0][2], 0);
    ASSERT_EQ(sparseTensor.coord()[1][0], 1);
    ASSERT_EQ(sparseTensor.coord()[1][1], 0);
    ASSERT_EQ(sparseTensor.coord()[1][2], 1);
    ASSERT_DOUBLE_EQ(features[0][0], 1./sqrt(2.));
    ASSERT_DOUBLE_EQ(features[0][1], 1./sqrt(2.));
    ASSERT_DOUBLE_EQ(features[0][2], 0.);
    ASSERT_DOUBLE_EQ(features[1][0], 0.);
    ASSERT_DOUBLE_EQ(features[1][1], 0.);
    ASSERT_DOUBLE_EQ(features[1][2], 1.);
}

TEST(SparseToDense, sparseBbox)
{
    CGAL::Bbox_3 denseBbox(0., 0., 0., 1., 1., 1.);
    double voxelSide = 0.5;
    int nbSparseVoxelSide = 4;

    SparseToDense stdStructure(denseBbox, voxelSide, nbSparseVoxelSide);

    CGAL::Bbox_3 sparseBbox = stdStructure.sparseBbox();

    ASSERT_DOUBLE_EQ(sparseBbox.xmin(), -0.5);
    ASSERT_DOUBLE_EQ(sparseBbox.ymin(), -0.5);
    ASSERT_DOUBLE_EQ(sparseBbox.zmin(), -0.5);
    ASSERT_DOUBLE_EQ(sparseBbox.xmax(), 1.5);
    ASSERT_DOUBLE_EQ(sparseBbox.ymax(), 1.5);
    ASSERT_DOUBLE_EQ(sparseBbox.zmax(), 1.5);
}

TEST(SparseToDense, Split)
{
    CGAL::Bbox_3 bbox(0., 0., 0., 1., 1., 1.);
    double voxelSide = 0.25;
    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);

    // Features
    vector<Point> points;
    vector<Point> pointOfViews;
    vector<Vector> normals;
    points.emplace_back(0.6, 0.2, 0.2);
    pointOfViews.emplace_back(0.2, 0.2, 0.8);
    normals.emplace_back(1., 0., 0.);

    // Labels
    const string testObjPath = (string) TEST_DIR + "simplecube.obj";
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");
    auto allTrees =  loadTreesFromObj(testObjPath, classesWithColor);

    const string outPath = (string) TEST_DIR + "/testSplit_";
    int nbVoxelsAlongAxisDense = 2;
    int nbVoxelsAlongAxisSparse = 4;
    bool verbose = false;

    int nbSplits = splitArrangementInVoxelsDts(allTrees, pointOfViews, points, normals, voxelSide,
                                               classesWithColor.size(), outPath, nbVoxelsAlongAxisDense,
                                               nbVoxelsAlongAxisSparse, verbose);

    cout.clear();
    cerr.clear();
    ASSERT_EQ(nbSplits, 2);
}