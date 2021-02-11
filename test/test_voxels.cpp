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
    cout.clear();
    cerr.clear();

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
    const int voidIdx = 11;
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
}