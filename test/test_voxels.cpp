#include <gtest/gtest.h>

#include "VoxelArrangement.h"

using namespace std;

TEST(VoxelArrangement, PlaneCreation)
{
    CGAL::Bbox_3 bbox(0., 0., 0., 1., 1., 1.);
    double voxelSize1 = 0.5;
    double voxelSize2 = 1.1;

    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    ASSERT_EQ(VoxelArrangement(bbox, voxelSize1).planes().size(), 9);
    ASSERT_EQ(VoxelArrangement(bbox, voxelSize2).planes().size(), 6);
    cout.clear();
    cerr.clear();
}

TEST(VoxelArrangement, PointQuery)
{
    CGAL::Bbox_3 bbox(0., 0., 0., 1., 1., 1.);
    double voxelSize1 = 0.5;
    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    auto voxArr = VoxelArrangement(bbox, voxelSize1);

    Kernel2::Point_3 query1(0., 0., 0.5);
    auto projQuery1 = voxArr.planeFromFacetHandle(voxArr.closestFacet(query1)).projection(query1);
    double projDistance1 = CGAL::to_double((projQuery1 - query1).squared_length());
    ASSERT_EQ(projDistance1, 0.);

    Kernel2::Point_3 query2(0.6, 0.25, 0.25);
    auto projQuery2 = voxArr.planeFromFacetHandle(voxArr.closestFacet(query2)).projection(query2);
    double projDistance2 = CGAL::to_double((projQuery2 - query2).squared_length());
    ASSERT_DOUBLE_EQ(projDistance2, pow(0.1, 2));

    Kernel2::Point_3 query3(0.9, 0.25, 0.25);
    auto projQuery3 = voxArr.planeFromFacetHandle(voxArr.closestFacet(query3)).projection(query3);
    double projDistance3 = CGAL::to_double((projQuery3 - query3).squared_length());
    ASSERT_DOUBLE_EQ(projDistance3, pow(0.1, 2));

    Kernel2::Point_3 query4(0.9, 0.9, 0.25);
    auto projQuery4 = voxArr.planeFromFacetHandle(voxArr.closestFacet(query4)).projection(query4);
    double projDistance4 = CGAL::to_double((projQuery4 - query4).squared_length());
    ASSERT_DOUBLE_EQ(projDistance4, pow(0.1, 2));
    cout.clear();
    cerr.clear();

}

TEST(VoxelArrangement, Labeling)
{
    const string testObjPath = (string) TEST_DIR + "simplecube.obj";
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");
    auto allTrees =  loadTreesFromObj(testObjPath, classesWithColor);

    CGAL::Bbox_3 bbox(0., 0., 0., 1., 1., 1.);
    double voxelSize1 = 0.5;
    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    auto voxArr = VoxelArrangement(bbox, voxelSize1);
    voxArr.assignLabel(allTrees, classesWithColor.size(), false);
    cout.clear();
    cerr.clear();

    VoxelArrangement::LabelTensor labels = voxArr.labels();
    const int partitionIdx = 5;
    const int voidIdx = -1;
    int nbPartition = 0;
    int nbVoid = 0;
    for(int i=0; i < labels.size(); i++)
        for(int j=0; j < labels[i].size(); j++)
            for(int k=0; k < labels[i][j].size(); k++)
            {
                if(labels[i][j][k] == partitionIdx)
                    nbPartition++;
                else if(labels[i][j][k] == voidIdx)
                    nbVoid++;
            }
    ASSERT_EQ(nbPartition, 2);
    ASSERT_EQ(nbVoid, 6);
    ASSERT_EQ(labels[0][0][0], partitionIdx);
    ASSERT_EQ(labels[0][0][1], partitionIdx);
}

TEST(VoxelArrangement, Features)
{
    CGAL::Bbox_3 bbox(0., 0., 0., 1., 1., 1.);
    double voxelSize1 = 0.5;
    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    auto voxArr = VoxelArrangement(bbox, voxelSize1);

    int nbClasses = 3;
    vector<Point> points;
    vector<Point> pointOfViews;
    vector<int> labels;

    // 1st point
    points.emplace_back(0.6, 0.25, 0.25);
    pointOfViews.emplace_back(0.9, 0.6, 0.25);
    labels.push_back(1);

    voxArr.computeFeatures(points, pointOfViews, labels, nbClasses, false);
    cout.clear();
    cerr.clear();

    VoxelArrangement::FeatTensor features = voxArr.features();
    ASSERT_DOUBLE_EQ(features[0][0][0][1], 1.);
    ASSERT_DOUBLE_EQ(features[1][1][0][nbClasses], 1.);
    ASSERT_DOUBLE_EQ(features[1][0][0][nbClasses], 1.);
}

TEST(VoxelArrangement, VoxelInOut)
{
    CGAL::Bbox_3 bbox(0., 0., 0., 1., 1., 1.);
    double voxelSize1 = 0.5;
    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);

    auto voxArr = VoxelArrangement(bbox, voxelSize1);

    // Labels
    const string testObjPath = (string) TEST_DIR + "simplecube.obj";
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");
    auto allTrees =  loadTreesFromObj(testObjPath, classesWithColor);
    voxArr.assignLabel(allTrees, classesWithColor.size(), false);

    // Features
    int nbClasses = classesWithColor.size();
    vector<Point> points;
    vector<Point> pointOfViews;
    vector<int> pointLabels;
    points.emplace_back(0.6, 0.25, 0.25);
    pointOfViews.emplace_back(0.9, 0.6, 0.25);
    pointLabels.push_back(1);
    voxArr.computeFeatures(points, pointOfViews, pointLabels, nbClasses, false);

    // Output
    string outputPath = (string) TEST_DIR + "voxelOutput.json";
    string outputPlyPath = (string) TEST_DIR + "voxelOutput.ply";
    string outputFeaturesPath = (string) TEST_DIR + "voxelFeatures.ply";
    voxArr.saveAsJson((string) TEST_DIR + "voxelOutput.json");

    //Input
    VoxelArrangement newVoxArr(outputPath);

    // Ply output
    newVoxArr.saveAsPly(outputPlyPath, classesWithColor);

    // Feature output
    newVoxArr.saveFeaturesAsPly(outputFeaturesPath, classesWithColor);

    cout.clear();
    cerr.clear();

    VoxelArrangement::FeatTensor features = newVoxArr.features();
    ASSERT_DOUBLE_EQ(features[0][0][0][1], 1.);
    ASSERT_EQ(voxArr.width(), newVoxArr.width());
    ASSERT_EQ(voxArr.height(), newVoxArr.height());
    ASSERT_EQ(voxArr.depth(), newVoxArr.depth());
}

TEST(VoxelArrangement, Split)
{
    CGAL::Bbox_3 bbox(0., 0., 0., 1., 1., 1.);
    double voxelSide = 0.25;
    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);

    // Features
    vector<Point> points;
    vector<Point> pointOfViews;
    vector<int> pointLabels;
    points.emplace_back(0.6, 0.25, 0.25);
    pointOfViews.emplace_back(0.9, 0.6, 0.25);
    pointLabels.push_back(1);

    // Labels
    const string testObjPath = (string) TEST_DIR + "simplecube.obj";
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");
    auto allTrees =  loadTreesFromObj(testObjPath, classesWithColor);

    const string outPath = (string) TEST_DIR + "/testSplit_";
    int maxNodes = 8;
    bool verbose = false;

    int nbSplits = splitArrangementInVoxels(allTrees, pointOfViews, points, pointLabels, voxelSide,
                                            classesWithColor.size(), outPath, maxNodes, verbose);

    cout.clear();
    cerr.clear();
    ASSERT_EQ(nbSplits, 2);
}
