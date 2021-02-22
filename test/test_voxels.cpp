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
    VoxelArrangement voxAr1(bbox, voxelSize1);
    VoxelArrangement voxAr2(bbox, voxelSize2);
    ASSERT_EQ(voxAr1.planes().size(), 3);
    ASSERT_EQ(voxAr2.planes().size(), 0);
    ASSERT_EQ(voxAr1.numberOfCells(), 8);
    ASSERT_EQ(voxAr2.numberOfCells(), 1);
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



TEST(VoxelArrangement, FeaturesRegular)
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

    // 1st point
    points.emplace_back(0.7, 0.25, 0.25);
    pointOfViews.emplace_back(0.4, 0.25, 1.1);
    labels.push_back(0);

    voxArr.computeFeaturesRegular(points, pointOfViews, labels, nbClasses, false);
    cout.clear();
    cerr.clear();

    VoxelArrangement::FeatTensor features = voxArr.features();
    for(int feat: features[0][0][0])
        ASSERT_DOUBLE_EQ(feat, 0.);
    for(int feat: features[0][1][0])
        ASSERT_DOUBLE_EQ(feat, 0.);
    for(int feat: features[0][1][1])
        ASSERT_DOUBLE_EQ(feat, 0.);
    for(int feat: features[1][1][1])
        ASSERT_DOUBLE_EQ(feat, 0.);
    ASSERT_DOUBLE_EQ(features[1][0][0][0], 1./2.);
    ASSERT_DOUBLE_EQ(features[1][0][0][1], 1./2.);
    ASSERT_DOUBLE_EQ(features[1][1][0][nbClasses], 1.);
    ASSERT_DOUBLE_EQ(features[0][0][1][nbClasses], 1.);
    ASSERT_DOUBLE_EQ(features[1][0][1][nbClasses], 1.);
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
    voxArr.computeFeaturesRegular(points, pointOfViews, pointLabels, nbClasses, false);

    // Output
    string outputJsonPath = (string) TEST_DIR + "voxelOutput.json";
    string outputPlyPath = (string) TEST_DIR + "voxelOutput.ply";
    string outputHdfPath = (string) TEST_DIR + "voxelOutput.h5";
    string outputFeaturesPath = (string) TEST_DIR + "voxelFeatures.ply";
    voxArr.saveAsJson(outputJsonPath);
    voxArr.saveAsHdf(outputHdfPath);

    //Input
    VoxelArrangement newVoxArr(outputJsonPath);
    VoxelArrangement h5VoxArr(outputHdfPath);

    // Ply output
    newVoxArr.saveAsPly(outputPlyPath, classesWithColor);

    // Feature output
    newVoxArr.saveFeaturesAsPly(outputFeaturesPath, classesWithColor);

    cout.clear();
    cerr.clear();

    VoxelArrangement::FeatTensor features = newVoxArr.features();
    ASSERT_DOUBLE_EQ(features[1][0][0][1], 1.);
    ASSERT_EQ(voxArr.width(), newVoxArr.width());
    ASSERT_EQ(voxArr.height(), newVoxArr.height());
    ASSERT_EQ(voxArr.depth(), newVoxArr.depth());
    ASSERT_EQ(voxArr.width(), h5VoxArr.width());
    ASSERT_EQ(voxArr.height(), h5VoxArr.height());
    ASSERT_EQ(voxArr.depth(), h5VoxArr.depth());
    ASSERT_EQ(voxArr.bbox(), h5VoxArr.bbox());
    for(int i=0; i < voxArr.planes().size(); i++) {
        ASSERT_EQ(voxArr.planes()[i].inlier, h5VoxArr.planes()[i].inlier);
        ASSERT_EQ(voxArr.planes()[i].normal, h5VoxArr.planes()[i].normal);
        ASSERT_EQ(voxArr.planes()[i].faces, h5VoxArr.planes()[i].faces);
        ASSERT_EQ(voxArr.planes()[i].cumulatedPercentage, h5VoxArr.planes()[i].cumulatedPercentage);
    }
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

    double nbVoxelsAlongAxis = 2.;
    int nbSplitsRegular = splitArrangementInVoxelsRegular(allTrees, pointOfViews, points, pointLabels, voxelSide,
                                                          classesWithColor.size(), outPath, nbVoxelsAlongAxis, verbose);

    int nbSplits = splitArrangementInVoxels(allTrees, pointOfViews, points, pointLabels, voxelSide,
                                            classesWithColor.size(), outPath, maxNodes, verbose);


    cout.clear();
    cerr.clear();
    ASSERT_EQ(nbSplits, 2);
    ASSERT_EQ(nbSplitsRegular, 2);
}

TEST(VoxelArrangement, SplitBbox)
{
    CGAL::Bbox_3 bbox(0., 0., 0., 10., 10., 10.);
    int nbVoxelsAlongAxis = 4;
    double voxelSide = 1.;
    vector<CGAL::Bbox_3> bboxes = splitBigBbox(bbox, nbVoxelsAlongAxis, voxelSide);
    ASSERT_EQ((int) bboxes.size(), 27);
    ASSERT_EQ(bboxes[0].xmin(), -1.);
    ASSERT_EQ(bboxes[0].xmax(), 3.);
}

TEST(VoxelArrangement, randomSplit)
{
    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    // Labels
    const string testObjPath = (string) TEST_DIR + "simplecube.obj";
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");
    auto allTrees =  loadTreesFromObj(testObjPath, classesWithColor);

    // Compute global bbox
    CGAL::Bbox_3 bboxGlobal;
    for(const auto& shape: allTrees)
        for(const auto& triangle: get<0>(shape))
            bboxGlobal += triangle.bbox();

    int nbVoxelsAlongAxis = 4;
    double voxelSide = 0.0435;
    cout.clear();
    cerr.clear();
    vector<CGAL::Bbox_3> bboxes = splitBigBbox(bboxGlobal, nbVoxelsAlongAxis, voxelSide);
    for (const auto &bbox: bboxes) {
        cout.setstate(ios_base::failbit);
        cerr.setstate(ios_base::failbit);
        VoxelArrangement arr(bbox, voxelSide);
        arr.assignLabel(allTrees, classesWithColor.size(), true);
        cout.clear();
        cerr.clear();
        ASSERT_EQ(arr.labels().size(), nbVoxelsAlongAxis);
        for(const auto& arr_slice: arr.labels())
        {
            ASSERT_EQ(arr_slice.size(), nbVoxelsAlongAxis);
            for(const auto& arr_line: arr_slice)
                ASSERT_EQ(arr_line.size(), nbVoxelsAlongAxis) << nbVoxelsAlongAxis;
        }
    }
}


TEST(VoxelArrangement, intersectSegment)
{
    CGAL::Bbox_3 bbox(0., 0., 0., 1., 1., 1.);
    double voxelSide = 0.5;
    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);

    // Features
    vector<Point> points;
    vector<Point> pointOfViews;
    vector<int> pointLabels;
    points.emplace_back(-0.25, 0.25, 0.25);
    pointOfViews.emplace_back(0.75, 0.25, 0.25);
    pointLabels.push_back(1);

    auto voxArr = VoxelArrangement(bbox, voxelSide);

    cout.clear();
    cerr.clear();
    vector<VoxelArrangement::triplet> intersectedVoxels = voxArr.intersectSegment(pointOfViews[0], points[0]);

}
