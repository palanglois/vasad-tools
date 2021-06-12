#include <gtest/gtest.h>

#include "ImplicitRepresentation.h"

using namespace std;

TEST(ImplicitRepresentation, computeSurfacicFromPointCloud)
{
    CGAL::Bbox_3 bbox(0., 0., 0., 1., 1., 1.);

    const int nbFiles = 10;
    const int nbSurfacicPerFiles = 76000;
    const int nbVolumicPointsPerFile = 100000;
    const int nbClasses = 5;

    ImplicitRepresentation impRep(bbox, nbFiles, nbSurfacicPerFiles, nbVolumicPointsPerFile, nbClasses);

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

    ImplicitRepresentation impRep2(bbox, nbFiles, nbSurfacicPerFiles, nbVolumicPointsPerFile, nbClasses);
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

    const string testObjPath = (string) TEST_DIR + "simplecube.obj";
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");
    auto allTrees = loadTreesFromObj(testObjPath, classesWithColor);
    ImplicitRepresentation impRep(bbox, nbFiles, nbSurfacicPerFiles, nbVolumicPointsPerFile, classesWithColor.size());

    vector<Point> sampledPoints;
    sampledPoints.emplace_back(0.25, 0.25, 0.25);
    sampledPoints.emplace_back(0.75, 0.25, 0.25);

    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    impRep.computeVolumicPoints(allTrees, sampledPoints, false);
    cout.clear();
    cerr.clear();

    const auto &volumicPoints = impRep.getVolumicPoints();
    const auto &occupancies = impRep.getOccupancies();

    const int partitionIdx = 5;
    const int voidIdx = classesWithColor.size();

    ASSERT_EQ(occupancies.size(), 2);
    ASSERT_EQ(occupancies[0], partitionIdx);
    ASSERT_EQ(occupancies[1], voidIdx);
}


TEST(ImplicitRepresentation, save) {
    CGAL::Bbox_3 bbox(0., 0., 0., 1., 1., 1.);

    const int nbFiles = 10;
    const int nbSurfacicPerFiles = 76000;
    const int nbVolumicPointsPerFile = 100000;

    const string testObjPath = (string) TEST_DIR + "simplecube.obj";
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");
    auto allTrees = loadTreesFromObj(testObjPath, classesWithColor);
    ImplicitRepresentation impRep(bbox, nbFiles, nbSurfacicPerFiles, nbVolumicPointsPerFile, classesWithColor.size());

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
    impRep.generateRandomVolumicPoints(allTrees, -1, false);
    cout.clear();
    cerr.clear();

    // Test normalization
    impRep.normalizeClouds();
    auto surfacicPoints = impRep.getSurfacicPoints();
    ASSERT_DOUBLE_EQ(surfacicPoints[0][0], 0.);
    ASSERT_DOUBLE_EQ(surfacicPoints[1][0], 0.3);

    impRep.save((string) TEST_DIR + "implicitChunk");
}

TEST(ImplicitRepresentation, computeBoxes)
{
    CGAL::Bbox_3 bbox(0., 0., 0., 1., 1., 1.);

    const int nbFiles = 10;
    const int nbSurfacicPerFiles = 76000;
    const int nbVolumicPointsPerFile = 100000;

    const string testObjPath = (string) TEST_DIR + "simplecube2.obj";
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");
    auto allTrees = loadTreesFromObj(testObjPath, classesWithColor);
    ImplicitRepresentation impRep(bbox, nbFiles, nbSurfacicPerFiles, nbVolumicPointsPerFile, classesWithColor.size());

    const int nbShoots = 50;

    vector<Point> sampledPoints;

    sampledPoints.emplace_back(0.25, 0.25, 0.5);
    sampledPoints.emplace_back(0.8, 0.5, 0.6);
    sampledPoints.emplace_back(1.5, 0.5, 0.5);

    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    impRep.computeBoxes(allTrees, sampledPoints, nbShoots);
    cout.clear();
    cerr.clear();
    impRep.normalizeClouds();

    vector<vector<double>> boxes = impRep.getBoxes();
    vector<double>& boxOne = boxes[0];

    // Check rotation
    Eigen::Quaternionf q(boxOne[3], boxOne[4], boxOne[5], boxOne[6]);
    Eigen::Matrix3f mat = q.normalized().toRotationMatrix();
    ASSERT_NEAR(mat(0, 0), 0., 1e-6);
    ASSERT_NEAR(mat(0, 1), 1., 1e-6);
    ASSERT_NEAR(mat(0, 2), 0., 1e-6);
    ASSERT_NEAR(mat(1, 0), 1., 1e-6);
    ASSERT_NEAR(mat(1, 1), 0., 1e-6);
    ASSERT_NEAR(mat(1, 2), 0., 1e-6);
    ASSERT_NEAR(mat(2, 0), 0., 1e-6);
    ASSERT_NEAR(mat(2, 1), 0., 1e-6);
    ASSERT_NEAR(mat(2, 2), -1., 1e-6);

    // Check scaling
    ASSERT_NEAR(boxOne[0], 0.5, 1e-6);
    ASSERT_NEAR(boxOne[1], 0.6, 1e-6);
    ASSERT_NEAR(boxOne[2], 1., 1e-6);
    Eigen::Vector3f scale(boxOne[0], boxOne[1], boxOne[2]);

    // Check translation
    ASSERT_NEAR(boxOne[7], 0.3, 1e-6);
    ASSERT_NEAR(boxOne[8], 0.25, 1e-6);
    ASSERT_NEAR(boxOne[9], 0.5, 1e-6);
    Eigen::Vector3f translation(boxOne[7], boxOne[8], boxOne[9]);

    // Check whole transformation
    Eigen::Vector3f xMin(-0.5, 0., 0.);
    Eigen::Vector3f xMax(0.5, 0., 0.);
    Eigen::Vector3f xMinT = mat*scale.cwiseProduct(xMin) + translation;
    Eigen::Vector3f xMaxT = mat*scale.cwiseProduct(xMax) + translation;
    ASSERT_NEAR((xMaxT - xMinT).norm(), 0.5, 1e-6);

    Eigen::Vector3f yMin(0., -0.5, 0.);
    Eigen::Vector3f yMax(0., 0.5, 0.);
    Eigen::Vector3f yMinT = mat*scale.cwiseProduct(yMin) + translation;
    Eigen::Vector3f yMaxT = mat*scale.cwiseProduct(yMax) + translation;
    ASSERT_NEAR((yMaxT - yMinT).norm(), 0.6, 1e-6);

    Eigen::Vector3f zMin(0., 0., -0.5);
    Eigen::Vector3f zMax(0., 0.,0.5);
    Eigen::Vector3f zMinT = mat*scale.cwiseProduct(zMin) + translation;
    Eigen::Vector3f zMaxT = mat*scale.cwiseProduct(zMax) + translation;
    ASSERT_NEAR((zMaxT - zMinT).norm(), 1., 1e-6);

}
