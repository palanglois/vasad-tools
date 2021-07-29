#include <gtest/gtest.h>

#include "EvalMetrics.h"

using namespace std;

TEST(EvalMetrics, computePcClosestLabel)
{
    // Load shape
    const string testObjPath = (string) TEST_DIR + "simplecube.obj";
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");
    auto allTrees = loadTreesFromObj(testObjPath, classesWithColor);

    // Make point cloud
    vector<Point> pointCloud;
    pointCloud.emplace_back(0.75, 0.25, 0.5);

    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    pair<vector<int>, vector<double>> labelsAndDist = computePcClosestLabel(pointCloud, allTrees);
    cout.clear();
    cerr.clear();

    ASSERT_EQ(labelsAndDist.second[0], pow(0.25, 2));

}