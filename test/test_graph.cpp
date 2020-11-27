#include <gtest/gtest.h>

#include "graph_fixture.h"
#include "../lib/RegionGrowing.h"

using namespace std;
using Json = nlohmann::json;


PlaneArrangementFixture::PlaneArrangementFixture() :
        planeArrangement(PlaneArrangement(vector<Plane>(), map<int, int>(), CGAL::Bbox_3()))
{

}

PlaneArrangementFixture::~PlaneArrangementFixture()
{

}

void PlaneArrangementFixture::SetUp()
{
    Arrangement myPlaneArrangement;

    // Bounding box
    bbox = CGAL::Bbox_3(0., 0., 0., 1., 1., 1.);
    myPlaneArrangement.set_bbox(bbox);

    // Plane A
    Kernel2::Vector_3 normalA(0., 0., 1.);
    Kernel2::Point_3 inlierA(0., 0., 0.5);
    Kernel2::Plane_3 planeACgal(inlierA, normalA);
    Plane planeA = {inlierA, normalA, vector<vector<int>>(), 0.5};
    myPlaneArrangement.insert(planeACgal);

    // Plane B
    Kernel2::Vector_3 normalB(1., 0., 0.);
    Kernel2::Point_3 inlierB(0.5, 0., 0.);
    Kernel2::Plane_3 planeBCgal(inlierB, normalB);
    Plane planeB = {inlierB, normalB, vector<vector<int>>(), 0.5};
    myPlaneArrangement.insert(planeBCgal);

    // Mappings
    for(auto cellIt = myPlaneArrangement.cells_begin(); cellIt != myPlaneArrangement.cells_end(); cellIt++)
    {
        if(myPlaneArrangement.is_cell_bounded(*cellIt))
        {
            bool left = cellIt->point().x() < 0.5;
            bool bottom = cellIt->point().z() < 0.5;
            if(left && bottom)
                label2cell[0] = myPlaneArrangement.cell_handle(*cellIt);
            else if(!left && bottom)
                label2cell[1] = myPlaneArrangement.cell_handle(*cellIt);
            else if(!left && !bottom)
                label2cell[2] = myPlaneArrangement.cell_handle(*cellIt);
            else if(left && !bottom)
                label2cell[3] = myPlaneArrangement.cell_handle(*cellIt);
        }
    }
    for(auto facetIt = myPlaneArrangement.facets_begin(); facetIt != myPlaneArrangement.facets_end(); facetIt++)
    {
        if(!myPlaneArrangement.is_facet_bounded(*facetIt)) continue;
        ASSERT_EQ(facetIt->number_of_superfaces(), 2);
        int cell1 = facetIt->superface(0);
        int cell2 = facetIt->superface(1);
        if (cell1 == label2cell[0] && cell2 == label2cell[1] ||
            cell2 == label2cell[0] && cell1 == label2cell[1])
            label2facet[0] = myPlaneArrangement.facet_handle(*facetIt);
        else if (cell1 == label2cell[1] && cell2 == label2cell[2] ||
                 cell2 == label2cell[1] && cell1 == label2cell[2])
            label2facet[1] = myPlaneArrangement.facet_handle(*facetIt);
        else if ((cell1 == label2cell[2] && cell2 == label2cell[3]) ||
                (cell2 == label2cell[2] && cell1 == label2cell[3]))
            label2facet[2] = myPlaneArrangement.facet_handle(*facetIt);
        else if (cell1 == label2cell[0] && cell2 == label2cell[3] ||
                 cell2 == label2cell[0] && cell1 == label2cell[3])
            label2facet[3] = myPlaneArrangement.facet_handle(*facetIt);
    }
    for(auto mapIt: label2cell)
        cell2label[mapIt.second] = mapIt.first;
    planeArrangement = PlaneArrangement({planeA, planeB}, cell2label, bbox);

}

void PlaneArrangementFixture::TearDown()
{

}

TEST_F(PlaneArrangementFixture, NodeFusion)
{
    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    Arrangement& myPlaneArrangement = planeArrangement.arrangement();
    cout.clear();
    cerr.clear();

    // First test
    vector<bool> labels = {false, false, false, true};
    pair<Nodes, Edges> nodesEdges = computeGraphStatistics(labels, cell2label, myPlaneArrangement);
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
    pair<Nodes, Edges> nodesEdges2 = computeGraphStatistics(labels2, cell2label, myPlaneArrangement);
    ASSERT_EQ(nodesEdges2.first.size(), 4);
    ASSERT_EQ(nodesEdges2.second.size(), 4);
}

TEST_F(PlaneArrangementFixture, NodeLabeling)
{
    // Test-mesh
    vector<Point> points;
    points.emplace_back(0., 0., 0.);
    points.emplace_back(0.5, 0., 0.);
    points.emplace_back(0.5, 0., 0.55);
    points.emplace_back(0., 0., 0.55);
    points.emplace_back(0., 1., 0.);
    points.emplace_back(0.5, 1., 0.);
    points.emplace_back(0.5, 1., 0.55);
    points.emplace_back(0., 1., 0.55);

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
    vector<int> nodeLabels = assignLabel(planeArrangement, labeledTrees, 1, 100, true);
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
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");
    auto allTrees =  loadTreesFromObj(testObjPath, classesWithColor);
    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    vector<int> gtLabels = assignLabel(planeArrangement, allTrees, classesWithColor.size(), true, false);
    cout.clear();
    cerr.clear();
    ASSERT_EQ(gtLabels[0], 5);
    ASSERT_EQ(gtLabels[1], -1);
    ASSERT_EQ(gtLabels[2], -1);
    ASSERT_EQ(gtLabels[3], 3);
}

TEST_F(PlaneArrangementFixture, visibilityRays)
{
    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    Arrangement& myPlaneArrangement = planeArrangement.arrangement();
    cout.clear();
    cerr.clear();

    vector<Point> inPoints;
    vector<Point> pointOfViews;
    inPoints.emplace_back(0.25, 0.5, 0.25);
    pointOfViews.emplace_back(1.75, 0.5, 0.25);
    inPoints.emplace_back(0.25, 0.5, 0.25);
    pointOfViews.emplace_back(0.75, 0.4, 0.35);
    EdgeFeatures edgeFeatures;
    int nbClasses = 1;
    vector<double> defaultFeature(nbClasses + 2, 0.);
    for(auto facetIt = myPlaneArrangement.facets_begin(); facetIt != myPlaneArrangement.facets_end(); facetIt++)
    {
        if(!myPlaneArrangement.is_facet_bounded(*facetIt)) continue;
        if(!myPlaneArrangement.is_cell_bounded(facetIt->superface(0))) continue;
        if(!myPlaneArrangement.is_cell_bounded(facetIt->superface(1))) continue;
        int cell0 = planeArrangement.cell2label().at(facetIt->superface(0));
        int cell1 = planeArrangement.cell2label().at(facetIt->superface(1));
        edgeFeatures[make_pair(cell0, cell1)] = defaultFeature;
    }

    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    computeVisibility(planeArrangement, inPoints, pointOfViews, edgeFeatures, nbClasses);
    cout.clear();
    cerr.clear();
    auto goodKey = edgeFeatures.find(make_pair(0, 1)) != edgeFeatures.end() ? make_pair(0, 1) : make_pair(1, 0);

    ASSERT_EQ(edgeFeatures[goodKey][nbClasses], 2);

}

TEST_F(PlaneArrangementFixture, pointSampling)
{
    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    Arrangement& myPlaneArrangement = planeArrangement.arrangement();
    cout.clear();
    cerr.clear();
    map<int, double> facetAreas;
    pair<vector<Point>, map<Point, int>> samples = sampleFacets(myPlaneArrangement, facetAreas);
    ASSERT_EQ(samples.first.size(), 8);

    // Simulated points
    const int nbClasses = 3;
    vector<Point> inPoints;
    vector<Point> pointOfViews;
    vector<int> inLabels;

    // Point A
    inPoints.emplace_back(0.25, 0.5, 0.501);
    pointOfViews.emplace_back(0.25, 0.5, 0.1);
    inLabels.push_back(0);

    // Point B
    inPoints.emplace_back(0.26, 0.5, 0.501);
    pointOfViews.emplace_back(0.25, 0.5, 0.1);
    inLabels.push_back(0);

    // Point C
    inPoints.emplace_back(0.24, 0.51, 0.5);
    pointOfViews.emplace_back(0.25, 0.5, 0.1);
    inLabels.push_back(1);

    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    EdgeFeatures features = computeFeaturesFromLabeledPoints(planeArrangement, inPoints, inLabels, nbClasses, 40, pointOfViews);
    cout.clear();
    cerr.clear();
    ASSERT_EQ(features.size(), 4);

    int cell1 = cell2label.at(myPlaneArrangement.facet(label2facet[3]).superface(0));
    int cell2 = cell2label.at(myPlaneArrangement.facet(label2facet[3]).superface(1));
    pair<int, int> edgeZeroV1(cell1, cell2);
    pair<int, int> edgeZeroV2(cell2, cell1);
    ASSERT_TRUE((features.find(edgeZeroV1) != features.end()) ||
                (features.find(edgeZeroV2) != features.end()));
    double nnDistance = (sqrt((inPoints[1] - inPoints[0]).squared_length()) +
                        sqrt((inPoints[1] - inPoints[0]).squared_length()) +
                        sqrt((inPoints[2] - inPoints[0]).squared_length()))/3.;
    double expectedNbOfPoints = 0.5 / (3.141592 * pow(nnDistance, 2));
    vector<double> targetDistrib = {2./expectedNbOfPoints, 1./expectedNbOfPoints, 0., min(1., 2. / expectedNbOfPoints), 1.};
    if(features.find(edgeZeroV1) != features.end())
    {
        stringstream outStream;
        outStream << "We got the following feature on edge (" << cell1 << ", " << cell2 << ") : " ;
        for(double i : features.at(edgeZeroV1))
            outStream << i << " ";
        outStream << endl;
        for(int i=0; i < targetDistrib.size(); i++)
            ASSERT_EQ(features.at(edgeZeroV1)[i], targetDistrib[i]) << outStream.rdbuf();
    }
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
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");
    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    PlaneArrangement myArrangement(outPath);
    auto allTrees = loadTreesFromObj(testObjPath, classesWithColor);
    pair<vector<Point>, vector<int>> pointsWithLabel;
    vector<Json> allSplits = splitArrangementInBatch(myArrangement, allTrees, classesWithColor.size(),
            step, maxNodes, pointsWithLabel, vector<Point>(0), maxNbPlanes, nbSamplesPerCell, proba, geom, 0.95, false);
    cout.clear();
    cerr.clear();
    ASSERT_GE(allSplits.size(), 1);
    for(const auto &split: allSplits) {
        ASSERT_LE(split.at("NodeFeatures").size(), maxNodes);
        ASSERT_GE(split.at("NodeFeatures").size(), 2);
    }
}

TEST(GraphStatistics, SampleBoundingBox)
{
    Eigen::MatrixXd points(8, 3);
    points.row(0) = PointRg(0.1, 0., 0.);
    points.row(1) = PointRg(0., 0., 1.);
    points.row(2) = PointRg(0., 2., 0.);
    points.row(3) = PointRg(0., 2., 1.);
    points.row(4) = PointRg(10., 0., 0.);
    points.row(5) = PointRg(10.1, 0., 1.);
    points.row(6) = PointRg(10., 2., 0.);
    points.row(7) = PointRg(10., 2., 1.);

    const double PI = 3.141592;
    random_device rd;
    mt19937 e2(rd());
    uniform_real_distribution<> roll(0, PI);
    uniform_real_distribution<> yaw(0, PI);
    uniform_real_distribution<> pitch(0, PI);
    Eigen::AngleAxisd rollAngle(roll(e2), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw(e2), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch(e2), Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

    Matrix rotationMatrix = q.matrix();
    Eigen::MatrixXd rotPoints = points * rotationMatrix.transpose();

    pair<Matrix, PointRg> transform = computeTransform(rotPoints);
    for(int i=0; i < rotationMatrix.rows(); i++)
        for(int j=0; j < rotationMatrix.cols(); j++)
            ASSERT_NEAR(abs(rotationMatrix(i, j)), abs(transform.first(j, i)), 0.005);

    vector<Kernel2::Point_3> cgalPoints;
    for(int i=0; i < points.rows(); i++)
        cgalPoints.emplace_back(points(i, 0), points(i, 1), points(i, 2));

    vector<pair<Point, int>> samples;
    sampleBetweenPoints(cgalPoints, samples);
    for(const auto sample: samples)
    {
        ASSERT_LE(-0.1, sample.first.x());
        ASSERT_LE(-0.1, sample.first.y());
        ASSERT_LE(-0.1, sample.first.z());
        ASSERT_GE(10.2, sample.first.x());
        ASSERT_GE(2.1, sample.first.y());
        ASSERT_GE(1.1, sample.first.z());
    }
}

TEST_F(PlaneArrangementFixture, sampleInConvexCell)
{
    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    Arrangement& myPlaneArrangement = planeArrangement.arrangement();
    cout.clear();
    cerr.clear();

    // Sample in it
    const int nbSamples = 100;
    vector<pair<Point, int>> samples = planeArrangement.getSamples(nbSamples);
    Simple_to_Epeck s2e;

    // Check that samples are well in the cell
    for(const auto& sample: samples) {
        int cellIdx = find_containing_cell(myPlaneArrangement, s2e(sample.first));
        ASSERT_EQ(cellIdx, sample.second) << "Point: " << sample.first << endl;
    }

//    // DEBUG
//    map<int, vector<int>> colorMap;
//    int iter = 0;
//    double step = 255. / double(planeArrangement.cell2label().size() + 1);
//    for(const auto& idx: planeArrangement.cell2label()) {
//        colorMap[idx.first] = {int(step*iter), int(step*iter), int(step*iter)};
//        iter++;
//    }
//    ofstream outFile((string) TEST_DIR + "testRot.obj");
//    for(const auto& sample:samples) {
//        cout << "Cur cell: " << sample.second << endl;
//        const auto& color = colorMap[sample.second];
//        outFile << "v " << sample.first.x() << " " << sample.first.y() << " "<< sample.first.z()
//        << " " << color[0] <<" " << color[1] << " " << color[2] << endl;
//    }
//    outFile << "v 0 0 1 0 0 255" << endl;
//    outFile << "v 0 1 0 0 0 255" << endl;
//    outFile << "v 0 1 1 0 0 255" << endl;
//    outFile << "v 1 0 0 0 0 255" << endl;
//    outFile << "v 1 0 1 0 0 255" << endl;
//    outFile << "v 1 1 0 0 0 255" << endl;
//    outFile << "v 1 1 1 0 0 255" << endl;
//    outFile << "v 0 0 0 0 0 255" << endl;
//    outFile.close();
}

TEST_F(PlaneArrangementFixture, computeNodeFeatures)
{
    vector<int> labels = {1, -1, -1, -1};
    double proba = 0.;
    bool withGeom = true;
    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    NodeFeatures nodeFeats = computeNodeFeatures(planeArrangement, labels, proba, withGeom);
    cout.clear();
    cerr.clear();

    for (const auto &feature: nodeFeats) {
        ASSERT_EQ(feature.size(), 4);
        ASSERT_EQ(feature[1], 0.5);
        ASSERT_EQ(feature[2], 1.);
        ASSERT_EQ(feature[3], 0.5);

        //No emptiness should be specified
        ASSERT_EQ(feature[0], 1);
    }
    proba = 1.;
    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    nodeFeats = computeNodeFeatures(planeArrangement, labels, proba, withGeom);
    cout.clear();
    cerr.clear();

    for (const auto &feature: nodeFeats) {
        ASSERT_EQ(feature.size(), 4);
        ASSERT_EQ(feature[1], 0.5);
        ASSERT_EQ(feature[2], 1.);
        ASSERT_EQ(feature[3], 0.5);
    }

    //Emptiness should be specified on nodes 1, 2 and 3
    ASSERT_EQ(nodeFeats[1][0], 0);
    ASSERT_EQ(nodeFeats[2][0], 0);
    ASSERT_EQ(nodeFeats[3][0], 0);
}

TEST_F(PlaneArrangementFixture, computeFacetOrientation)

{
    cout.setstate(ios_base::failbit);
    cerr.setstate(ios_base::failbit);
    double facetOrientation0 = computeFacetOrientation(planeArrangement.arrangement(), label2facet.at(0));
    double facetOrientation1 = computeFacetOrientation(planeArrangement.arrangement(), label2facet.at(1));
    double facetOrientation2 = computeFacetOrientation(planeArrangement.arrangement(), label2facet.at(2));
    double facetOrientation3 = computeFacetOrientation(planeArrangement.arrangement(), label2facet.at(3));
    cout.clear();
    cerr.clear();
    ASSERT_EQ(facetOrientation0, 0.);
    ASSERT_EQ(facetOrientation1, 1.);
    ASSERT_EQ(facetOrientation2, 0.);
    ASSERT_EQ(facetOrientation3, 1.);
}

TEST(PointOfViews, cubePointOfView)
{
    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");

    // Load 3D model
    vector<Triangle> triangles;
    const string inputPath = (string) TEST_DIR + "weirdCube.obj";
    auto trianglesAndClasses = loadTrianglesFromObj(inputPath, classesWithColor);
    triangles = trianglesAndClasses.first;

    // BBox Triangles
    CGAL::Bbox_3 bbox(-1., -1., -1., 1., 1., 1.);
    vector<Triangle> bboxMesh = meshBbox(bbox);

    // Concatenate all triangles
    triangles.insert(triangles.end(), bboxMesh.begin(), bboxMesh.end());

    // Perform point refinement
    Point initPoint(0.75, -0.75, -0.1);
    refinePoint(initPoint, triangles, 100);

    ASSERT_NEAR(initPoint.x(), 0., 0.3) << "Point: " << initPoint;
}


TEST(PointOfViews, SamplePtViewOnObject)
{
    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");

    // Load 3D model
    vector<Triangle> triangles;
    const string inputPath = (string) TEST_DIR + "weirdCube.obj";
    auto trianglesAndClasses = loadTrianglesFromObj(inputPath, classesWithColor);
    triangles = trianglesAndClasses.first;
    vector<facesLabelName> shapesAndClasses = loadTreesFromObj(inputPath, classesWithColor);

    // BBox Triangles
    CGAL::Bbox_3 bbox;
    for(const auto& triangle: triangles)
        bbox += triangle.bbox();
    vector<Triangle> bboxMesh = meshBbox(bbox);

    // Concatenate all triangles
    triangles.insert(triangles.end(), bboxMesh.begin(), bboxMesh.end());

    int nbShoot = 10;
    int nbCandidates = 100;
    vector<Point> ptViews = findPtViewInBbox(bbox, shapesAndClasses, triangles, nbShoot, nbCandidates);

    ASSERT_EQ(ptViews.size(), nbShoot);
}
