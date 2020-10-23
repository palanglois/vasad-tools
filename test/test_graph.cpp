#include <gtest/gtest.h>
#include "graph_fixture.h"

using namespace std;


PlaneArrangementFixture::PlaneArrangementFixture()
{

}

PlaneArrangementFixture::~PlaneArrangementFixture()
{

}

void PlaneArrangementFixture::SetUp()
{
    myPlaneArrangement = new Arrangement();

    // Bounding box
    bbox = CGAL::Bbox_3(0., 0., 0., 1., 1., 1.);
    myPlaneArrangement->set_bbox(bbox);

    // Plane A
    Kernel2::Vector_3 normalA(0., 0., 1.);
    Kernel2::Point_3 inlierA(0., 0., 0.5);
    Kernel2::Plane_3 planeA(inlierA, normalA);
    myPlaneArrangement->insert(planeA);

    // Plane B
    Kernel2::Vector_3 normalB(1., 0., 0.);
    Kernel2::Point_3 inlierB(0.5, 0., 0.);
    Kernel2::Plane_3 planeB(inlierB, normalB);
    myPlaneArrangement->insert(planeB);

    // Mappings
    for(auto cellIt = myPlaneArrangement->cells_begin(); cellIt != myPlaneArrangement->cells_end(); cellIt++)
    {
        if(myPlaneArrangement->is_cell_bounded(*cellIt))
        {
            bool left = cellIt->point().x() < 0.5;
            bool bottom = cellIt->point().z() < 0.5;
            if(left && bottom)
                label2cell[0] = myPlaneArrangement->cell_handle(*cellIt);
            else if(!left && bottom)
                label2cell[1] = myPlaneArrangement->cell_handle(*cellIt);
            else if(!left && !bottom)
                label2cell[2] = myPlaneArrangement->cell_handle(*cellIt);
            else if(left && !bottom)
                label2cell[3] = myPlaneArrangement->cell_handle(*cellIt);
        }
    }
    for(auto mapIt: label2cell)
        cell2label[mapIt.second] = mapIt.first;

}

void PlaneArrangementFixture::TearDown()
{
    delete myPlaneArrangement;
}

TEST_F(PlaneArrangementFixture, NodeFusion)
{

    // First test
    vector<bool> labels = {false, false, false, true};
    pair<Nodes, Edges> nodesEdges = computeGraphStatistics(labels, cell2label, *myPlaneArrangement);
    ASSERT_EQ(nodesEdges.first.size(), 2);
    ASSERT_EQ(nodesEdges.second.size(), 1);
    vector<int> &bigNode = nodesEdges.first[0].size() == 3 ? nodesEdges.first[0] : nodesEdges.first[1];
    vector<int> &smallNode = nodesEdges.first[0].size() == 1 ? nodesEdges.first[0] : nodesEdges.first[1];
    ASSERT_EQ(bigNode.size(), 3);
    ASSERT_EQ(smallNode.size(), 1);
    // Big node should have cells {0, 1, 2] and small node should have cell {3}
    ASSERT_TRUE(std::find(bigNode.begin(), bigNode.end(), label2cell[0]) != bigNode.end());
    ASSERT_TRUE(std::find(bigNode.begin(), bigNode.end(), label2cell[1]) != bigNode.end());
    ASSERT_TRUE(std::find(bigNode.begin(), bigNode.end(), label2cell[2]) != bigNode.end());
    ASSERT_EQ(smallNode[0], label2cell[3]);

    // Second test
    vector<bool> labels2 = {false, true, false, true};
    pair<Nodes, Edges> nodesEdges2 = computeGraphStatistics(labels2, cell2label, *myPlaneArrangement);
    ASSERT_EQ(nodesEdges2.first.size(), 4);
    ASSERT_EQ(nodesEdges2.second.size(), 4);
}

TEST_F(PlaneArrangementFixture, NodeLabeling)
{
    // Test-mesh
    vector<Point> points;
    points.emplace_back(0., 0., 0.);
    points.emplace_back(0.5, 0., 0.);
    points.emplace_back(0.5, 0., 0.6);
    points.emplace_back(0., 0., 0.6);
    points.emplace_back(0., 1., 0.);
    points.emplace_back(0.5, 1., 0.);
    points.emplace_back(0.5, 1., 0.6);
    points.emplace_back(0., 1., 0.6);

    vector<Triangle> triangles;
    triangles.emplace_back(points[0], points[1], points[2]);
    triangles.emplace_back(points[0], points[2], points[3]);
    triangles.emplace_back(points[0], points[1], points[4]);
    triangles.emplace_back(points[1], points[4], points[5]);
    triangles.emplace_back(points[0], points[4], points[3]);
    triangles.emplace_back(points[4], points[3], points[7]);
    triangles.emplace_back(points[2], points[3], points[7]);
    triangles.emplace_back(points[2], points[7], points[6]);
    triangles.emplace_back(points[4], points[5], points[6]);
    triangles.emplace_back(points[4], points[6], points[7]);
    triangles.emplace_back(points[1], points[5], points[2]);
    triangles.emplace_back(points[5], points[2], points[6]);

    // AABB Tree for test mesh
    vector<facesLabelName> labeledTrees = {make_tuple(triangles, 0, "")};

    vector<int> nodeLabels = assignLabel(*myPlaneArrangement, cell2label, bbox, labeledTrees, 10000, true, false);
    ASSERT_EQ(nodeLabels[0], 0);
    ASSERT_EQ(nodeLabels[1], -1);
    ASSERT_EQ(nodeLabels[2], -1);
    ASSERT_EQ(nodeLabels[3], -1);
}

TEST_F(PlaneArrangementFixture, LabelingWithObjLoad)
{
    const string testObjPath = (string) TEST_DIR + "test.obj";
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses("../semantic_classes.json");
    auto allTrees =  loadTreesFromObj(testObjPath, classesWithColor);
    vector<int> gtLabels = assignLabel(*myPlaneArrangement, cell2label, bbox, allTrees, 10000, true, false);
    ASSERT_EQ(gtLabels[0], 5);
    ASSERT_EQ(gtLabels[1], -1);
    ASSERT_EQ(gtLabels[2], -1);
    ASSERT_EQ(gtLabels[3], 3);
}
