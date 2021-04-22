#include <gtest/gtest.h>

#include "ImplicitRepresentation.h"

using namespace std;

TEST(ImplicitRepresentation, computeSurfacicFromPointCloud)
{
    CGAL::Bbox_3 bbox(0., 0., 0., 1., 1., 1.);

    const int nbFiles = 10;
    const int nbSurfacicPerFiles = 76000;
    const int nbVolumicPointsPerFile = 100000;

    ImplicitRepresentation impRep(bbox, nbFiles, nbSurfacicPerFiles, nbVolumicPointsPerFile);

    vector<Point> pointCloud;
    vector<Vector> normals;

    pointCloud.emplace_back(0.5, 0.5, 0.5);
    normals.emplace_back(1., 0., 0.);
    pointCloud.emplace_back(1.5, 0.5, 0.5);
    normals.emplace_back(1., 0., 0.);

    impRep.computeSurfacicFromPointCloud(pointCloud, normals);

    ASSERT_EQ(impRep.getSurfacicPoints().size(), 1);
    ASSERT_EQ(impRep.getSurfacicNormals().size(), 1);

    // Test with empty cloud
    vector<Point> emptyPointCloud;
    vector<Vector> emptyNormals;

    ImplicitRepresentation impRep2(bbox, nbFiles, nbSurfacicPerFiles, nbVolumicPointsPerFile);
    impRep2.computeSurfacicFromPointCloud(emptyPointCloud, emptyNormals);
    const string impRepPath = (string) TEST_DIR + "impRep/";

    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    impRep2.save(impRepPath);
    cout.clear();
    cerr.clear();
}

TEST(ImplicitRepresentation, computeVolumicPoints)
{
    CGAL::Bbox_3 bbox(0., 0., 0., 1., 1., 1.);

    const int nbFiles = 10;
    const int nbSurfacicPerFiles = 76000;
    const int nbVolumicPointsPerFile = 100000;

    ImplicitRepresentation impRep(bbox, nbFiles, nbSurfacicPerFiles, nbVolumicPointsPerFile);
    const string testObjPath = (string) TEST_DIR + "simplecube.obj";
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");
    auto allTrees = loadTreesFromObj(testObjPath, classesWithColor);

    vector<Point> sampledPoints;
    sampledPoints.emplace_back(0.25, 0.25, 0.25);
    sampledPoints.emplace_back(0.75, 0.25, 0.25);

    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    impRep.computeVolumicPoints(allTrees, classesWithColor.size(), sampledPoints, false);
    cout.clear();
    cerr.clear();

    const auto &volumicPoints = impRep.getVolumicPoints();
    const auto &occupancies = impRep.getOccupancies();

    const int partitionIdx = 5;
    const int voidIdx = -1;

    ASSERT_EQ(occupancies.size(), 2);
    ASSERT_EQ(occupancies[0], partitionIdx);
    ASSERT_EQ(occupancies[1], voidIdx);
}



TEST(ImplicitRepresentation, save)
{
    CGAL::Bbox_3 bbox(0., 0., 0., 1., 1., 1.);

    const int nbFiles = 10;
    const int nbSurfacicPerFiles = 76000;
    const int nbVolumicPointsPerFile = 100000;

    ImplicitRepresentation impRep(bbox, nbFiles, nbSurfacicPerFiles, nbVolumicPointsPerFile);
    const string testObjPath = (string) TEST_DIR + "simplecube.obj";
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");
    auto allTrees = loadTreesFromObj(testObjPath, classesWithColor);

    vector<Point> pointCloud;
    vector<Vector> normals;

    pointCloud.emplace_back(0.5, 0.5, 0.5);
    normals.emplace_back(1., 0., 0.);
    pointCloud.emplace_back(0.8, 0.5, 0.6);
    normals.emplace_back(1., 0., 0.);
    pointCloud.emplace_back(1.5, 0.5, 0.5);
    normals.emplace_back(1., 0., 0.);

    impRep.computeSurfacicFromPointCloud(pointCloud, normals);

    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    impRep.generateRandomVolumicPoints(allTrees, classesWithColor.size(), false);
    cout.clear();
    cerr.clear();

    // Test normalization
    impRep.normalizeClouds();
    auto surfacicPoints = impRep.getSurfacicPoints();
    ASSERT_DOUBLE_EQ(surfacicPoints[0][0], 0.);
    ASSERT_DOUBLE_EQ(surfacicPoints[1][0], 0.6);

    impRep.save((string) TEST_DIR + "implicitChunk");
}
