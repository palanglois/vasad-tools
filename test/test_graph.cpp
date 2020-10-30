#include <gtest/gtest.h>

#include "graph_fixture.h"
#include "../lib/RegionGrowing.h"

using namespace std;
using Json = nlohmann::json;


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
    for(auto facetIt = myPlaneArrangement->facets_begin(); facetIt != myPlaneArrangement->facets_end(); facetIt++)
    {
        if(!myPlaneArrangement->is_facet_bounded(*facetIt)) continue;
        ASSERT_EQ(facetIt->number_of_superfaces(), 2);
        int cell1 = facetIt->superface(0);
        int cell2 = facetIt->superface(1);
        if (cell1 == label2cell[0] && cell2 == label2cell[1] ||
            cell2 == label2cell[0] && cell1 == label2cell[1])
            label2facet[0] = myPlaneArrangement->facet_handle(*facetIt);
        else if (cell1 == label2cell[1] && cell2 == label2cell[2] ||
                 cell2 == label2cell[2] && cell1 == label2cell[1])
            label2facet[1] = myPlaneArrangement->facet_handle(*facetIt);
        else if (cell1 == label2cell[2] && cell2 == label2cell[3] ||
                 cell2 == label2cell[3] && cell1 == label2cell[2])
            label2facet[2] = myPlaneArrangement->facet_handle(*facetIt);
        else if (cell1 == label2cell[0] && cell2 == label2cell[3] ||
                 cell2 == label2cell[3] && cell1 == label2cell[0])
            label2facet[3] = myPlaneArrangement->facet_handle(*facetIt);
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

    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    vector<int> nodeLabels = assignLabel(*myPlaneArrangement, cell2label, bbox, labeledTrees, 1, true, false);
    cout.clear();
    cerr.clear();
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
    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    vector<int> gtLabels = assignLabel(*myPlaneArrangement, cell2label, bbox, allTrees, classesWithColor.size(), true, false);
    cout.clear();
    cerr.clear();
    ASSERT_EQ(gtLabels[0], 5);
    ASSERT_EQ(gtLabels[1], -1);
    ASSERT_EQ(gtLabels[2], -1);
    ASSERT_EQ(gtLabels[3], 3);
}

TEST_F(PlaneArrangementFixture, pointSampling)
{
    const int factor = 20;
    pair<vector<Point>, map<Point, int>> samples = sampleFacets(*myPlaneArrangement, factor);
    ASSERT_EQ(samples.first.size(), 640);

    // Simulated points
    const int nbClasses = 3;
    vector<Point> inPoints;
    vector<int> inLabels;

    // Point A
    inPoints.emplace_back(0.5, 0.5, 0.25);
    inLabels.push_back(0);

    // Point B
    inPoints.emplace_back(0.5, 0.5, 0.26);
    inLabels.push_back(0);

    // Point C
    inPoints.emplace_back(0.5, 0.51, 0.24);
    inLabels.push_back(1);

    EdgeFeatures features = computeFeaturesFromLabeledPoints(*myPlaneArrangement, inPoints, inLabels, nbClasses, 20);
    ASSERT_EQ(features.size(), 1);

    int cell1 = myPlaneArrangement->facet(label2facet[0]).superface(0);
    int cell2 = myPlaneArrangement->facet(label2facet[0]).superface(1);
    pair<int, int> edgeZeroV1(cell1, cell2);
    pair<int, int> edgeZeroV2(cell2, cell1);
    ASSERT_TRUE((features.find(edgeZeroV1) != features.end()) ||
                (features.find(edgeZeroV2) != features.end()));
    vector<double> targetDistrib = {2./3., 1./3., 0.};
    if(features.find(edgeZeroV1) != features.end())
        for(int i=0; i < 3; i++)
            ASSERT_EQ(features.at(edgeZeroV1)[i], targetDistrib[i]);
}

TEST(GraphStatistics, ComputePlanesInBoundingBox)
{
    CGAL::Bbox_3 bbox(0., 0., 0., 1., 1., 1.);

    vector<Point> points = {
            Point(0.5, 0., 0.),
            Point(0.5, 0., 1.),
            Point(0.5, 1., 0.),
            Point(0.5, 0., -1.5),
            Point(0.5, 0., -0.5),
            Point(0.5, 1., -1.5),
            Point(0.5, -0.5, 1.5),
            Point(0.5, -0.5, -0.5),
            Point(0.5, 1.5, -0.5),
            Point(0.5, 0.5, 0.5),
            Point(2., 0.5, 0.5),
            Point(2., 0.5, 1.),
            Point(0.5, 0.5, 0.5),
            Point(0.5, 0.5, 0.75),
            Point(0.75, 0.5, 0.5)
    };

    // Plane 1
    vector<vector<int>> faces1(1, {0, 1, 2});
    Kernel2::Point_3 inlier1(0.5, 0., 0.);
    Kernel2::Vector_3 normal1(1., 0., 0.);
    double cumulatedArea1 = 0.5;
    Plane plane1 = {inlier1, normal1, faces1, cumulatedArea1};

    // Plane 2
    vector<vector<int>> faces2(1, {3, 4, 5});
    Kernel2::Point_3 inlier2(0.5, 0., -1.5);
    Kernel2::Vector_3 normal2(1., 0., 0.);
    double cumulatedArea2 = 0.5;
    Plane plane2 = {inlier2, normal2, faces2, cumulatedArea2};

    // Plane 3
    vector<vector<int>> faces3(1, {3, 4, 5});
    Kernel2::Point_3 inlier3(0.5, 0., -1.5);
    Kernel2::Vector_3 normal3(1., 0., 0.);
    double cumulatedArea3 = 0.98;
    Plane plane3 = {inlier3, normal3, faces3, cumulatedArea3};

    // Plane 4
    vector<vector<int>> faces4(1, {6, 7, 8});
    Kernel2::Point_3 inlier4(0.5, -0.5, 1.5);
    Kernel2::Vector_3 normal4(1., 0., 0.);
    double cumulatedArea4 = 0.5;
    Plane plane4 = {inlier4, normal4, faces4, cumulatedArea4};

    // Plane 5
    vector<vector<int>> faces5(1, {9, 10, 11});
    Kernel2::Point_3 inlier5(0.5, 0.5, 0.5);
    Kernel2::Vector_3 normal5(0., 1., 0.);
    double cumulatedArea5 = 0.5;
    Plane plane5 = {inlier5, normal5, faces5, cumulatedArea5};

    // Plane 6
    vector<vector<int>> faces6(1, {12, 13, 14});
    Kernel2::Point_3 inlier6(0.5, -0.5, 1.5);
    Kernel2::Vector_3 normal6(1., 0., 0.);
    double cumulatedArea6 = 0.5;
    Plane plane6 = {inlier6, normal6, faces6, cumulatedArea6};

    vector<Plane> allPlanes = {plane1, plane2, plane3, plane4, plane5, plane6};

    vector<int> inlierPlaneIdx = computePlanesInBoundingBox(allPlanes, points, bbox, 0.95);

    ASSERT_EQ(inlierPlaneIdx.size(), (size_t) 4);
    ASSERT_EQ(inlierPlaneIdx[0], 0);
    ASSERT_EQ(inlierPlaneIdx[1], 3);
    ASSERT_EQ(inlierPlaneIdx[2], 4);
    ASSERT_EQ(inlierPlaneIdx[3], 5);
}

TEST(GraphStatistics, SplitingModel)
{
    const string testObjPath = (string) TEST_DIR + "cube.obj";
    const string outPath = (string) TEST_DIR + "coloredGtPlanes2.json";
    double epsilonPoint = 0.01;
    double epsilonNormal = 0.005;
    double sigmaPoint = 0.04;
    double sigmaNormal = 0.001;
    double step = 1.;
    int nbSamplesPerCell = 40;
    int maxNodes = 14;
    int maxNbPlanes = 250;
    double proba = 0.05;
    bool geom = true;
    RegionGrowing rg(testObjPath, epsilonPoint, epsilonNormal, sigmaPoint, sigmaNormal, false);
    int nbPrimitives = rg.run();
    ASSERT_EQ(nbPrimitives, 9);
    rg.saveAsJson((string) TEST_DIR + "coloredGtPlanes2.json", true);
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses("../semantic_classes.json");
    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    PlaneArrangement myArrangement(outPath);
    auto allTrees = loadTreesFromObj(testObjPath, classesWithColor);
    vector<Json> allSplits = splitArrangementInBatch(myArrangement, allTrees, classesWithColor.size(),
            step, maxNodes, maxNbPlanes, nbSamplesPerCell, proba, geom, 0.95, false);
    cout.clear();
    cerr.clear();
    ASSERT_GE(allSplits.size(), 1);
    for(const auto &split: allSplits) {
        ASSERT_LE(split.at("NodeFeatures").size(), maxNodes);
        ASSERT_GE(split.at("NodeFeatures").size(), 2);
    }
}
