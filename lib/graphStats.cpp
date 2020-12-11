#include "graphStats.h"


using namespace std;
using Json = nlohmann::json;

template <typename T, typename A>
int arg_max(vector<T, A> const& vec) {
    return static_cast<int>(distance(vec.begin(), max_element(vec.begin(), vec.end())));
}

pair<Nodes, Edges> computeGraphStatistics(const vector<bool> &labels, const map<int, int> &cell2label,
        const Arrangement &arr, bool verbose)
{
    map<int, int> label2cell;
    for(auto &mapIt: cell2label)
        label2cell[mapIt.second] = mapIt.first;

    // Computing statistics
    int clusterId = 1;
    vector<bool> visitedCell(labels.size(), false);
    stack<Arrangement::Face_handle> nextCells;

    //We start with a bounded cell
    auto firstCell = arr.cells_begin();
    while(!arr.is_cell_bounded(*firstCell)) firstCell++;
    nextCells.push(arr.cell_handle(*firstCell));

    vector<vector<int>> nodes;
    vector<set<Arrangement::Face_handle>> cluster2neighbourClusterCells;
    while(!nextCells.empty())
    {
        auto nextCell = nextCells.top();
        nextCells.pop();
        int nextCellIdx = cell2label.at(nextCell);

        if(visitedCell[nextCellIdx] || !arr.is_cell_bounded(nextCell))
            continue;
        visitedCell[nextCellIdx] = true;

        int curLabel = labels[nextCellIdx];

        // Region growing
        set<Arrangement::Face_handle> sameLabel; // All cells of the current region
        set<Arrangement::Face_handle> currentCells; // Surrounding cells (at iteration) with same label
        set<Arrangement::Face_handle> neighbourClusterCells; // Surrounding cells with different label

        sameLabel.insert(nextCell);
        currentCells.insert(nextCell);
        while(!currentCells.empty()) {
            set<Arrangement::Face_handle> neighbourCells;
            for (const auto &cell_handle: currentCells) {
                auto &cell = arr.cell(cell_handle);
                for(auto facetIt = cell.subfaces_begin(); facetIt != cell.subfaces_end(); facetIt++) {
                    Arrangement::Face_handle cell0 = arr.facet(*facetIt).superface(0);
                    Arrangement::Face_handle cell1 = arr.facet(*facetIt).superface(1);
                    // We find the cell that is not the current cell
                    auto candidateCell = (cell0 == cell_handle) ? cell1 : cell0;
                    if(!arr.is_cell_bounded(candidateCell))
                        continue;
                    int candidateCellIdx = cell2label.at(arr.cell_handle(arr.cell(candidateCell)));
                    if(visitedCell[candidateCellIdx])
                        continue;
                    // We store it according to its label
                    int candidateLabel = labels[candidateCellIdx];
                    //cout << "Cell " << nextCellIdx << " and " << candidateCellIdx << " are neighbors." << endl;
                    if(candidateLabel == curLabel) {
                        sameLabel.insert(candidateCell);
                        neighbourCells.insert(candidateCell);
                        visitedCell[candidateCellIdx] = true;
                    }
                    else {
                        nextCells.push(candidateCell);
                        neighbourClusterCells.insert(candidateCell);
                    }
                }
            }
            currentCells = neighbourCells;
        }
        cluster2neighbourClusterCells.push_back(neighbourClusterCells);
        nodes.emplace_back(sameLabel.begin(), sameLabel.end());
        if(verbose)
            cout << "Cluster number " << clusterId << " computed with label " << curLabel << "!" << endl;
        for(auto &cell: sameLabel) {
            int cellIdx = cell2label.at(arr.cell_handle(arr.cell(cell)));
            if(verbose)
                cout << cellIdx << " " ;
            assert(labels[cellIdx] == curLabel);
        }
        if(verbose)
            cout << endl;
        clusterId++;
    }
    if(verbose)
        cout << "Computed " << nodes.size() << " nodes." << endl;

    //Edges
    int clusterIdx = 0;
    vector<pair<int, int>> edges;
    for(auto i = 0;i < cluster2neighbourClusterCells.size(); i++)
    {
        auto &cluster = cluster2neighbourClusterCells[i];
        set<int> curNeighbours;
        for(auto &neighbourCell: cluster) {
            int corresIdx = 0;
            while (find(nodes[corresIdx].begin(), nodes[corresIdx].end(),
                        arr.cell_handle(arr.cell(neighbourCell))) == nodes[corresIdx].end())
                corresIdx++;
            curNeighbours.insert(corresIdx);
            if(find(edges.begin(), edges.end(), pair<int, int>(i, corresIdx)) == edges.end() &&
               find(edges.begin(), edges.end(), pair<int, int>(corresIdx, i)) == edges.end())
                edges.emplace_back(i, corresIdx);
        }
        //cout << "Cluster number " << clusterIdx << " has neighbors: ";
        //for(auto &neighbourCluster: curNeighbours)
        //    cout << neighbourCluster << " ";
        //cout << endl;
        clusterIdx++;
    }
    if(verbose)
        cout << "Computed " << edges.size() << " edges." << endl;

    return make_pair(nodes, edges);
}

//vector<int> assignLabel(const Arrangement &arr, const map<int, int> &cell2label, CGAL::Bbox_3 bbox,
//                        vector<facesLabelName> &labeledShapes, int nbClasses,
//                        int nbSamplesPerCell, bool verbose)
vector<int> assignLabel(PlaneArrangement& planeArr,
                        vector<facesLabelName> &labeledShapes, int nbClasses,
                        int nbSamplesPerCell, bool verbose)
{
    const Arrangement& arr = planeArr.arrangement();
    const map<int, int> &cell2label = planeArr.cell2label();
    CGAL::Bbox_3 bbox = planeArr.bbox();

    Simple_to_Epeck s2e;
    Epeck_to_Simple e2s;
    //Build bbox for each shape
    vector<CGAL::Bbox_3> bboxes;
    for(const auto& labeledShape: labeledShapes)
        bboxes.push_back(CGAL::bbox_3(get<0>(labeledShape).begin(), get<0>(labeledShape).end()));
    // Number of classes to consider
//    int nbClasses = 0;
//    for(auto &labeledTree: labeledShapes)
//        if(get<1>(labeledTree) + 2 > nbClasses)
//            nbClasses = get<1>(labeledTree) + 2;
    int voidClass = nbClasses;

    vector<int> labels(cell2label.size(), -1);

    // Draw points in the arrangement
    default_random_engine generator;
//    uniform_real_distribution<double> xDist(bbox.xmin(), bbox.xmax());
//    uniform_real_distribution<double> yDist(bbox.ymin(), bbox.ymax());
//    uniform_real_distribution<double> zDist(bbox.zmin(), bbox.zmax());
    vector<pair<Point, int>> queryPoints = planeArr.getSamples();

    //DEBUG
//    const auto& testRef = labeledShapes[labeledShapes.size() - 1].first->m_primitives[0].datum();
//    cout << testRef << endl;
//    cout << endl;
    //END DEBUG

    vector<vector<int>> votes(cell2label.size(), vector<int>(nbClasses + 1, 0));


    //DEBUG
//    cout << testRef << endl;
//    cout << endl;
    //END DEBUG
    //queryPoints.reserve(nbSamples);
    //for(int i=0; i < nbSamples; i++)
        //queryPoints.emplace_back(Point(xDist(generator), yDist(generator), zDist(generator)), -1);
//    for(auto cellIt = arr.cells_begin(); cellIt != arr.cells_end(); cellIt++)
//    {
//        if(!arr.is_cell_bounded(*cellIt)) continue;
//        vector<Kernel2::Point_3> points;
//        for(auto facetIt = cellIt->subfaces_begin(); facetIt != cellIt->subfaces_end(); facetIt++)
//        {
//            auto facet = arr.facet(*facetIt);
//            for(auto edgeIt = facet.subfaces_begin(); edgeIt != facet.subfaces_end(); edgeIt++) {
//                auto edge = arr.edge(*edgeIt);
//                for(auto pointIt = edge.subfaces_begin(); pointIt != edge.subfaces_end(); pointIt++)
//                    points.push_back(arr.point(*pointIt));
//            }
//        }

        // The almighty great sampling
//        sampleInConvexCell(arr, arr.cell_handle(*cellIt), queryPoints, 40);
//        // New sampling
//        sampleBetweenPoints(points, queryPoints, 40, arr.cell_handle(*cellIt));
        /* Former bounding box drawing
        // Note: We explicitely compute the bounding box, the function CGAL::bbox_3 gives too big
        // bounding boxes!
        Point pt = e2s(points[0]);
        double xmin = pt.x();
        double xmax = pt.x();
        double ymin = pt.y();
        double ymax = pt.y();
        double zmin = pt.z();
        double zmax = pt.z();
        for(const auto& point: points)
        {
            Point pt2 = e2s(point);
            xmin = min(xmin, pt2.x());
            xmax = max(xmax, pt2.x());
            ymin = min(ymin, pt2.y());
            ymax = max(ymax, pt2.y());
            zmin = min(zmin, pt2.z());
            zmax = max(zmax, pt2.z());
        }
        auto curBbox = CGAL::Bbox_3(xmin, ymin, zmin, xmax, ymax, zmax);

        // Drawing points in the bounding box
        uniform_real_distribution<double> curXDist(curBbox.xmin(), curBbox.xmax());
        uniform_real_distribution<double> curYDist(curBbox.ymin(), curBbox.ymax());
        uniform_real_distribution<double> curZDist(curBbox.zmin(), curBbox.zmax());
        for(int i=0; i< nbSamplesPerCell; i++)
            queryPoints.emplace_back(Point(curXDist(generator), curYDist(generator), curZDist(generator)), arr.cell_handle(*cellIt));
         */

//            // DEBUG
//            if(int(arr.cell_handle(*cellIt)) == 125161) {
//                for (int i = queryPoints.size() - 40; i < queryPoints.size(); i++)
//                    cout << "Here " << queryPoints[i].first << endl;
//                cout << "Bbox " << curBbox << endl;
//                for(const auto& point: points)
//                    cout << "Point " << point << " bbox " << point.bbox() << endl;
//            }
//    }

    // For every point, test it for every shape
    Tree tree;
    vector<int> pointsLabel(queryPoints.size(), voidClass);
    auto tqLabeledShapes = tq::trange(labeledShapes.size()); // works for rvalues too!
    tqLabeledShapes.set_prefix("Labeling each point: ");
    for (int j : tqLabeledShapes)
    {
        if(bboxes[j].xmax() < bbox.xmin()) continue;
        if(bboxes[j].xmin() > bbox.xmax()) continue;
        if(bboxes[j].ymax() < bbox.ymin()) continue;
        if(bboxes[j].ymin() > bbox.ymax()) continue;
        if(bboxes[j].zmax() < bbox.zmin()) continue;
        if(bboxes[j].zmin() > bbox.zmax()) continue;
        auto &labeledTree = labeledShapes[j];
        tree.rebuild(get<0>(labeledTree).begin(), get<0>(labeledTree).end());
        bool warningSent = false;
#pragma omp parallel for schedule(static)
	for(int i=0; i < queryPoints.size(); i++)
        {

            auto &point = queryPoints[i].first;
            //if(point.x() < 0.5 && point.z() < 0.6) // DEBUG
            //    cout << "debug" << endl;
//            int label = voidClass;
            if(point.x() < bboxes[j].xmin()) continue;
            if(point.x() > bboxes[j].xmax()) continue;
            if(point.y() < bboxes[j].ymin()) continue;
            if(point.y() > bboxes[j].ymax()) continue;
            if(point.z() < bboxes[j].zmin()) continue;
            if(point.z() > bboxes[j].zmax()) continue;
//            cout << point << endl;
//            if(j==1 && point.x() < 0.5 && point.z() > 0.6)
//                cout << "debug" << endl;
//            cout << "DEBUG: tree nb " << j << " out of " << labeledShapes.size() << endl;
//            cout << labeledTree.first->bbox() << endl;
//            if(j == labeledShapes.size() - 1) {
//                for (auto prim: labeledShapes[labeledShapes.size() - 1].first->m_primitives)
//                    cout << prim.datum() << endl;
//                cout << endl;
//            }
            //Make a random query ray and intersect it
            //Ray query(point, Vector(1., 0., 0.));
            vector<Ray> queries = {Ray(point, Vector(1., 0., 0.)),
                                   Ray(point, Vector(0., 1., 0.)),
                                   Ray(point, Vector(0., 0., 1.))};
            vector<list<Ray_intersection>> intersections(3, list<Ray_intersection>(0));
            for(int k=0; k < queries.size(); k++)
                tree.all_intersections(queries[k], back_inserter(intersections[k]));
            unsigned int odd = 0;
            for(const auto& inter: intersections)
                odd += int(inter.size() % 2 == 1);
            if(odd == 2)
            {
                if(!warningSent) {
#pragma omp critical
{
                    warningSent = true;
                    cout << endl << "\033[1;34mPotential issue with shape " << get<2>(labeledTree) << "\033[0m" << endl;
}
                }
            }

            // Test the nb of intersections for parity
            if(odd >= 2)
            {
                // In the current shape
#pragma omp critical
                pointsLabel[i] = get<1>(labeledTree);
                //cout << "Here " << labeledTree.second << endl;
            }
        }
    }
    auto tqQueryPoints = tq::trange(queryPoints.size());
    tqQueryPoints.set_prefix("Computing cell votes: ");
    for(int i: tqQueryPoints) {
        // Find the current cell
        Arrangement::Face_handle cellHandle;
        if(queryPoints[i].second == -1)
            cellHandle = find_containing_cell(arr, s2e(queryPoints[i].first));
        else
            cellHandle = find_containing_cell(arr, s2e(queryPoints[i].first), queryPoints[i].second);
        if(!arr.is_cell_bounded(cellHandle)) continue;
        assert(cell2label.at(cellHandle) < votes.size());
        assert(cell2label.at(cellHandle) >= 0);
        assert(i < pointsLabel.size());
        assert(i >= 0);
        assert(pointsLabel[i] < votes[cell2label.at(cellHandle)].size());
        assert(pointsLabel[i] >= 0);
        votes[cell2label.at(cellHandle)][pointsLabel[i]]++;
    }
    for(int i = 0; i < votes.size(); i++)
        if(*max_element(votes[i].begin(), votes[i].end()) == 0)
            // If the cell is too thin, there can be no vote. In this case, we'll assume the cell is empty
            labels[i] = -1;
        else
            labels[i] = arg_max(votes[i]) == voidClass ? -1 : arg_max(votes[i]);

//    if(verbose) {
//        for (int i = 0; i < votes.size(); i++) {
//            bool display = false;
//            for(int j=0; j < votes[i].size() - 1; j++)
//                if(votes[i][j] != 0)
//                    display = true;
//            if(display) {
//                cout << "Cell vote for cell " << i << ": ";
//                for (auto nb: votes[i])
//                    cout << nb << " ";
//                cout << endl;
//            }
//        }
//    }

//    //DEBUG
//    cout << "DEBUG" << endl;
//    Point query(37.7275666666667, -30.2528, 8.59148666666667);
//    Point query(34.5212666666667, -30.2528, 8.17148666666667);
//    Point query(9.60298666666667, 6.73633666666667, 5.93695333333333);
//    Point query(47.3597333333333, -9.72466666666667, 12.0712666666667);
//    Arrangement::Face_handle debugCell = find_containing_cell(arr, s2e(query));
//
//    if(arr.is_cell_bounded(debugCell)) {
//        cout << "Cell id: " << debugCell << endl;
//        cout << "votes: " << endl;
//        for (auto nb: votes[cell2label.at(debugCell)])
//            cout << nb << " ";
//        cout << endl;
//        cout << "label given: " << labels[cell2label.at(debugCell)] << endl;
//        cout << "points of current cell: " << endl;
//    }
//    auto cell = arr.cell(debugCell);
//    for(auto facetIt = cell.subfaces_begin(); facetIt != cell.subfaces_end(); facetIt++)
//    {
//        auto facet = arr.facet(*facetIt);
//        for(auto edgeIt = facet.subfaces_begin(); edgeIt != facet.subfaces_end(); edgeIt++) {
//            auto edge = arr.edge(*edgeIt);
//            for(auto pointIt = edge.subfaces_begin(); pointIt != edge.subfaces_end(); pointIt++)
//                cout << arr.point(*pointIt) << endl;
//        }
//    }
//    vector<Triangle> triangles;
//    TriangleClassMap colorMap;
//    cout << "nb of faces: " << cell.number_of_subfaces() << endl;
//    for(auto facetIt = cell.subfaces_begin(); facetIt != cell.subfaces_end(); facetIt++) {
//        auto facet = arr.facet(*facetIt);
//        auto edgeOne = facet.subfaces_begin();
//        auto edgeTwo = facet.subfaces_begin()++;
//        while (edgeTwo != facet.subfaces_end()) {
//            if(arr.edge(*edgeTwo).number_of_subfaces() != 2) break;
//            auto pt1 = arr.edge(*edgeOne).subface(0);
//            auto pt2 = arr.edge(*edgeTwo).subface(0);
//            auto pt3 = arr.edge(*edgeTwo).subface(1);
//            triangles.emplace_back(e2s(arr.point(pt1)), e2s(arr.point(pt2)), e2s(arr.point(pt3)));
//            colorMap[triangles[triangles.size() - 1]] = {100, 100, 100};
//            edgeTwo++;
//        }
//    }
//    saveTrianglesAsObj(triangles, "test.obj", colorMap);

    return labels;
}

NodeFeatures computeNodeFeatures(PlaneArrangement& planeArr, vector<int> &labels, const double proba,
                                 const bool withGeom, bool verbose)
{
    const map<int, int> &cell2label = planeArr.cell2label();
    const Arrangement &arr = planeArr.arrangement();

    // Graph nodes
    default_random_engine generator;
    uniform_real_distribution<double> unitRandom(0., 1.);
    NodeFeatures nodeFeatures(cell2label.size(), vector<double>(1, 0));
    for (auto cellIt = arr.cells_begin(); cellIt != arr.cells_end(); cellIt++) {
        if (!arr.is_cell_bounded(*cellIt)) continue;
        auto cellHandle = arr.cell_handle(*cellIt);
        int labelIdx = cell2label.at(cellHandle);
        double feature = 1.; // We suppose the current cell is full
        if (labels[labelIdx] == -1) {
            // If it was actually empty, we only say it in proba% of the cases
            if (unitRandom(generator) < proba)
                feature = 0.;
        }

        nodeFeatures[labelIdx] = {feature};

        // Node geometry
        if (withGeom) {
            vector<double> curBbox = {DBL_MAX, DBL_MAX, DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX};
            for (auto facetIt = cellIt->subfaces_begin(); facetIt != cellIt->subfaces_end(); facetIt++) {
                auto facet = arr.facet(*facetIt);
                for (auto edgeIt = facet.subfaces_begin(); edgeIt != facet.subfaces_end(); edgeIt++) {
                    auto edge = arr.edge(*edgeIt);
                    for (auto pointIt = edge.subfaces_begin(); pointIt != edge.subfaces_end(); pointIt++) {
                        curBbox[0] = min(curBbox[0], CGAL::to_double(arr.point(*pointIt).x()));
                        curBbox[1] = min(curBbox[1], CGAL::to_double(arr.point(*pointIt).y()));
                        curBbox[2] = min(curBbox[2], CGAL::to_double(arr.point(*pointIt).z()));
                        curBbox[3] = max(curBbox[3], CGAL::to_double(arr.point(*pointIt).x()));
                        curBbox[4] = max(curBbox[4], CGAL::to_double(arr.point(*pointIt).y()));
                        curBbox[5] = max(curBbox[5], CGAL::to_double(arr.point(*pointIt).z()));
                    }
                }
            }
            nodeFeatures[labelIdx].push_back(curBbox[3] - curBbox[0]);
            nodeFeatures[labelIdx].push_back(curBbox[4] - curBbox[1]);
            nodeFeatures[labelIdx].push_back(curBbox[5] - curBbox[2]);
        }
    }
    return nodeFeatures;
}

EdgeFeatures computeTrivialEdgeFeatures(PlaneArrangement& planeArr, vector<int> &labels, const int nbClasses,
                                        bool verbose)
{
    const map<int, int> &cell2label = planeArr.cell2label();
    const Arrangement &arr = planeArr.arrangement();

    // Graph edges
    EdgeFeatures edgeFeatures;
    for(auto facetIt = arr.facets_begin(); facetIt != arr.facets_end(); facetIt++)
    {
        if(facetIt->number_of_superfaces() != 2)
            cerr << "Facet doesn't have exactly 2 adjacent cells!" << endl;
        if(arr.is_cell_bounded(arr.cell(facetIt->superface(0))) &&
           arr.is_cell_bounded(arr.cell(facetIt->superface(1))))
        {
            // We have a valid graph edge
            int cellIdx1 = facetIt->superface(0);
            int cellIdx2 = facetIt->superface(1);
            int cellFeatureIdx1 = cell2label.at(cellIdx1);
            int cellFeatureIdx2 = cell2label.at(cellIdx2);
            int cellLabel1 = labels[cellFeatureIdx1];
            int cellLabel2 = labels[cellFeatureIdx2];
            vector<double> edgeFeature(nbClasses + 1, 0.);
            if(cellLabel1 == -1 && cellLabel2 != -1)
            {
                // We're at a void/full transition
                edgeFeature[cellLabel2] = 1.;
                edgeFeatures[make_pair(cellFeatureIdx1, cellFeatureIdx2)] = edgeFeature;
            }
            else if(cellLabel1 != -1 && cellLabel2 == -1)
            {
                // Same here but different direction
                edgeFeature[cellLabel1] = 1.;
                edgeFeatures[make_pair(cellFeatureIdx1, cellFeatureIdx2)] = edgeFeature;
            }
            else
            {
                // We're not at a transition
                edgeFeature[nbClasses] = 1.;
                edgeFeatures[make_pair(cellFeatureIdx1, cellFeatureIdx2)] = edgeFeature;
            }
        }
    }
    return edgeFeatures;
}

pair<vector<Point>, map<Point, int>> sampleFacets(const Arrangement &arr, map<int, double> &facetAreas)
{
    auto e2s = Epeck_to_Simple();
    double minArea = DBL_MAX;
    double totalArea = 0;
    vector<double> triangleAreas;
    vector<vector<Point>> triangles;
    vector<int> triangleToHandle;
    for(auto facetIt = arr.facets_begin(); facetIt != arr.facets_end(); facetIt++) {

        // Compute area of current facet
        double facetArea = 0.;
        if (!arr.is_facet_bounded(*facetIt)) continue;
        if(!arr.is_cell_bounded(facetIt->superface(0))) continue;
        if(!arr.is_cell_bounded(facetIt->superface(1))) continue;
        std::vector<Arrangement::Face_handle> vertices;
        arr.facet_to_polygon(*facetIt, std::back_inserter(vertices));

        if (vertices.size() >= 3) {

            std::vector<Arrangement::Face_handle>::const_iterator vhi = vertices.begin();
            Arrangement::Point first = arr.point(*vhi);
            ++vhi;
            Arrangement::Point second = arr.point(*vhi);
            ++vhi;
            while (vhi != vertices.end()) {
                Arrangement::Point third = arr.point(*vhi);
                double triangleArea = sqrt(CGAL::to_double(CGAL::squared_area(first, second, third)));
                triangleAreas.push_back(triangleArea);
                totalArea += triangleArea;
                facetArea += triangleArea;
                triangles.push_back({e2s(first), e2s(second), e2s(third)});
                triangleToHandle.push_back(arr.facet_handle(*facetIt));
                second = third;
                ++vhi;
            }
        }
        facetAreas[arr.facet_handle(*facetIt)] = facetArea;
        // Update
        minArea = min(minArea, facetArea);
    }

    // Normalize the area vector
    for(double &currentArea: triangleAreas)
        currentArea /= totalArea;
    int nbPointsToSample = 1; // TODO remove in the future
    // Actual sampling
    vector<Point> sampledPoints;
    map<Point, int> pointToHandle;
#pragma omp parallel for
    for(int i=0; i < triangles.size(); i++)
    {
        // Number of points to sample on this triangle
        int currentNbPtToSample = max(1, int(triangleAreas[i] * nbPointsToSample));
        for (int j = 0; j < currentNbPtToSample; j++) {
            // Draw a random point in this triangle
            double r1 = ((double) rand() / (RAND_MAX));
            double r2 = ((double) rand() / (RAND_MAX));
            const Point &A = triangles[i][0];
            const Point &B = triangles[i][1];
            const Point &C = triangles[i][2];
            Point P = CGAL::ORIGIN + (1 - sqrt(r1)) * (A - CGAL::ORIGIN)
                      + (sqrt(r1) * (1 - r2)) * (B - CGAL::ORIGIN)
                      + (sqrt(r1) * r2) * (C - CGAL::ORIGIN);
#pragma omp critical
            {
                pointToHandle[P] = triangleToHandle[i];
                sampledPoints.push_back(P);
            }
        }
    }
    return make_pair(sampledPoints, pointToHandle);
}

inline double computeFacetArea(const Arrangement &arr, int facetHandle)
{
    // Compute area of current facet
    double facetArea = 0.;
    if (!arr.is_facet_bounded(facetHandle)) return -1.;
    const auto& facet = arr.facet(facetHandle);
    vector<Arrangement::Face_handle> vertices;
    arr.facet_to_polygon(facet, back_inserter(vertices));

    if (vertices.size() >= 3) {

        vector<Arrangement::Face_handle>::const_iterator vhi = vertices.begin();
        Arrangement::Point first = arr.point(*vhi);
        ++vhi;
        Arrangement::Point second = arr.point(*vhi);
        ++vhi;
        while (vhi != vertices.end()) {
            Arrangement::Point third = arr.point(*vhi);
            double triangleArea = sqrt(CGAL::to_double(CGAL::squared_area(first, second, third)));
            facetArea += triangleArea;
            second = third;
            ++vhi;
        }
    }
    return facetArea;
}

vector<double> computeVisibility(PlaneArrangement &planeArr, const vector<Point> &points,
                                 const vector<Point> &pointOfViews, EdgeFeatures &edgeFeatures, int nbClasses,
                                 const vector<Arrangement::Face_handle> &exactPovCells)
{
    vector<double> nodeVisibility(planeArr.cell2label().size(), 0.);
    Simple_to_Epeck s2e;
    auto tqPoints = tq::trange(points.size());
    tqPoints.set_prefix("Computing visibility: ");
    for (int i : tqPoints)
    {
        // Intersect point_of_view <-> detected point segments with the plane arrangement
        Arrangement::Face_handle begin_cell;
        Arrangement::Face_handle end_cell;
        vector<pair<Arrangement::Face_handle, int>> intersectedFacets;
        vector<pair<Arrangement::Face_handle, double>> intersectedCellsAndDists;
        if(exactPovCells.empty())
            segment_search_advanced(planeArr.arrangement(), s2e(pointOfViews[i]), s2e(points[i]),
                           begin_cell, back_inserter(intersectedFacets),
                           back_inserter(intersectedCellsAndDists), end_cell);
        else {
            segment_search_advanced(planeArr.arrangement(), s2e(pointOfViews[i]), s2e(points[i]),
                           begin_cell, back_inserter(intersectedFacets),
                           back_inserter(intersectedCellsAndDists), end_cell, exactPovCells[i], true);
//            // BEGIN DEBUG
//            if(CGAL::do_overlap(points[i].bbox(), planeArr.bbox()) && CGAL::do_overlap(pointOfViews[i].bbox(), planeArr.bbox())) {
//                cout << planeArr.bbox() << endl;
//                cout << exactPovCells[i] << " " << begin_cell << endl;
//                cout << planeArr.arrangement().cell(exactPovCells[i]).point() << endl;
//                cout << planeArr.arrangement().cell(begin_cell).point() << endl;
//                cout << endl;
//            }
//            bool fail=true;
//            for(const auto& pair: intersectedFacets) {
//                // For every intersected facet, we increment the visibility bin
//                if (!planeArr.arrangement().is_facet_bounded(pair.first)) continue;
//                auto &facet = planeArr.arrangement().facet(pair.first);
//                if (!planeArr.arrangement().is_cell_bounded(facet.superface(0))) continue;
//                if (!planeArr.arrangement().is_cell_bounded(facet.superface(1))) continue;
//                fail=false;
//                break;
//            }
//            if(fail)
//                cout << "Fail" << endl;
//            else
//                cout << "Good" << endl;
//            // END DEBUG
        }
        // Node visibility
        for(const auto& cellAndDist: intersectedCellsAndDists){
            if(!planeArr.arrangement().is_cell_bounded(cellAndDist.first)) continue;
            nodeVisibility[planeArr.cell2label().at(cellAndDist.first)] += cellAndDist.second;
        }

        for(const auto& pair: intersectedFacets)
        {
            // For every intersected facet, we increment the visibility bin
            if(!planeArr.arrangement().is_facet_bounded(pair.first)) continue;
            auto& facet = planeArr.arrangement().facet(pair.first);
            if(!planeArr.arrangement().is_cell_bounded(facet.superface(0))) continue;
            if(!planeArr.arrangement().is_cell_bounded(facet.superface(1))) continue;
            int cell0 = planeArr.cell2label().at(facet.superface(0));
            int cell1 = planeArr.cell2label().at(facet.superface(1));
            std::pair<int, int> keyOne = make_pair(cell0, cell1);
            std::pair<int, int> keyTwo = make_pair(cell1, cell0);
            std::pair<int, int> goodKey = edgeFeatures.find(keyTwo) != edgeFeatures.end() ? keyTwo : keyOne;
            assert(edgeFeatures.find(goodKey) != edgeFeatures.end());
            edgeFeatures[goodKey][nbClasses]++;
        }
    }
    return nodeVisibility;
}

EdgeFeatures computeFeaturesFromLabeledPoints(PlaneArrangement &planeArr, const vector<Point> &points,
                                              const vector<int> &labels, const int nbClasses, int nbSamplesPerCell,
                                              vector<double> &nodeVisibility, const vector<Point> &pointOfViews, bool verbose)
{
    bool withVisibility = !pointOfViews.empty();
    Arrangement &arr = planeArr.arrangement();
    const map<int, int> &cell2label = planeArr.cell2label();
    const CGAL::Bbox_3& bbox = planeArr.bbox();
    auto s2e = Simple_to_Epeck();

    // Get samples
    const vector<pair<Point, int>> &samples = planeArr.getSamples(nbSamplesPerCell);
    if(verbose)
        cout << "Sampled " << samples.size() << " points." << endl;

    // Extract samples
    vector<Point> sampledPoints;
    map<Point, int> pointToCellHandle;
    for(const auto& sample: samples)
    {
        sampledPoints.push_back(sample.first);
        pointToCellHandle[sample.first] = sample.second;
    }

    // Make tree
    kdTree tree(sampledPoints.begin(), sampledPoints.end());

    // Initializing edge features
    EdgeFeatures features;
    map<pair<int, int>, double> facetAreasMapping;
    map<pair<int, int>, double> facetOrientationMapping;
    for(auto facetIt = arr.facets_begin(); facetIt != arr.facets_end(); facetIt++)
    {
        if(!arr.is_facet_bounded(*facetIt)) continue;
        int cell1 = facetIt->superface(0);
        if(!arr.is_cell_bounded(cell1)) continue;
        int cell2 = facetIt->superface(1);
        if(!arr.is_cell_bounded(cell2)) continue;
        pair<int, int> facetId(cell2label.at(cell1), cell2label.at(cell2));
        features[facetId] = vector<double>(nbClasses + 2, 0);
        if(!withVisibility)
            features[facetId][nbClasses] = 1.;
        double facetOrientation = computeFacetOrientation(arr, arr.facet_handle(*facetIt));
        features[facetId][nbClasses + 1] = facetOrientation;
        facetAreasMapping[facetId] = computeFacetArea(arr, arr.facet_handle(*facetIt));
        facetOrientationMapping[facetId] = facetOrientation;
    }

    // Find closest facet by nearest neighbour search
    auto tqPoints = tq::trange(points.size());
    vector<Point> validPoints;
    tqPoints.set_prefix("Making edge features: ");
    for (int i : tqPoints)
    {
        const auto& point = points[i];
        if(point.x() <= bbox.xmin()) continue;
        if(point.y() <= bbox.ymin()) continue;
        if(point.z() <= bbox.zmin()) continue;
        if(point.x() >= bbox.xmax()) continue;
        if(point.y() >= bbox.ymax()) continue;
        if(point.z() >= bbox.zmax()) continue;
        validPoints.push_back(point);
        Neighbor_search search(tree, point, 1);

        // Find facet handle
        int closestFacetHandle = -1;
        auto curPointEpeck = s2e(point);
        int cell1Raw = pointToCellHandle.at(search.begin()->first);
        int curPointCellIdx = find_containing_cell(arr, curPointEpeck, cell1Raw);
        double bestDistance = DBL_MAX;
        for (auto facetIt = arr.cell(curPointCellIdx).subfaces_begin();
             facetIt != arr.cell(curPointCellIdx).subfaces_end(); facetIt++) {
            const Arrangement::Plane& curPlane = arr.plane(arr.facet_plane(arr.facet(*facetIt)));
            double projDist = CGAL::to_double((curPlane.projection(curPointEpeck) - curPointEpeck).squared_length());
            if(projDist < bestDistance)
            {
                bestDistance = projDist;
                closestFacetHandle = *facetIt;
            }
        }
        cell1Raw = arr.facet(closestFacetHandle).superface(0);
        int cell2Raw = arr.facet(closestFacetHandle).superface(1);
        if(!arr.is_cell_bounded(cell1Raw) || !arr.is_cell_bounded(cell2Raw)) continue;

        // Update corresponding feature
        int cell1 = cell2label.at(cell1Raw);
        int cell2 = cell2label.at(cell2Raw);
        auto pair1 = make_pair(cell1, cell2);
        auto pair2 = make_pair(cell2, cell1);
        pair<int, int> goodPair;
        if(features.find(pair1) != features.end()) {
            features[pair1][labels[i]] += 1;
            features[pair1][nbClasses] = 0;
            goodPair = pair1;
        }
        else if(features.find(pair2) != features.end()) {
            features[pair2][labels[i]] += 1;
            features[pair2][nbClasses] = 0;
            goodPair = pair2;
        }
        else
            cerr << "Issue: edge " << cell1 << ", " << cell2 << " should have been initialized." << endl;
    }

    // Get the average distance between neighbour points in the point cloud
    double averageNNDistance = 0.;
    kdTree densityTree(validPoints.begin(), validPoints.end());
    for(const auto& point: validPoints) {
        Neighbor_search search(densityTree, point, 2);
        averageNNDistance += sqrt(next(search.begin(), 1)->second);
    }
    averageNNDistance /= validPoints.size();

    // Visibility
    if(withVisibility)
    {
        // Initial guess for the point of views cells
        vector<Point> beginPoints;
        vector<Point> endPoints;
        addSegmentIfInBbox(pointOfViews, points, back_inserter(beginPoints), back_inserter(endPoints), bbox);

        // We efficiently compute the cell where the point of view is located
        vector<Arrangement::Face_handle> guessedCells(0, Arrangement::Face_handle(0));
        map<Point, int> pov2cell;
        for(auto & beginPoint : beginPoints) {
            if(pov2cell.find(beginPoint) != pov2cell.end())
                guessedCells.emplace_back(pov2cell.at(beginPoint));
            else
            {
                Neighbor_search search(tree, beginPoint, 1);
                int curPovCellIdx = find_containing_cell(arr, s2e(beginPoint),
                                                         pointToCellHandle.at(search.begin()->first));
                pov2cell[beginPoint] = curPovCellIdx;
                guessedCells.emplace_back(curPovCellIdx);
            }
        }
        // Processing visibility segments
        nodeVisibility = computeVisibility(planeArr, endPoints, beginPoints, features, nbClasses, guessedCells);
    }

    // Normalize features
    vector<double> emptyVec( nbClasses + 2, 0.);
    if(!withVisibility)
        emptyVec[nbClasses] = 1.;
    for(auto& edgeFeat: features)
    {
        double expectedNbOfPoints = facetAreasMapping[edgeFeat.first] / (3.141592 * pow(averageNNDistance, 2));
        if (expectedNbOfPoints >= 1. && edgeFeat.second[nbClasses] != 1.) {
            for (int i = 0; i < nbClasses + 1; i++)
                edgeFeat.second[i] /= expectedNbOfPoints;
        } else {
            edgeFeat.second = emptyVec;
            edgeFeat.second[nbClasses + 1] = facetOrientationMapping[edgeFeat.first];
        }
	// Renormalization
    for (auto &elem: edgeFeat.second)
        elem = min(elem, 1.);
    }

    return features;
}

inline bool isInBbox(const Triangle& tri, const CGAL::Bbox_3 &bbox)
{
    if(tri.bbox().xmin() < bbox.xmin()) return false;
    if(tri.bbox().ymin() < bbox.ymin()) return false;
    if(tri.bbox().zmin() < bbox.zmin()) return false;
    if(tri.bbox().xmax() > bbox.xmax()) return false;
    if(tri.bbox().ymax() > bbox.ymax()) return false;
    return !(tri.bbox().zmax() > bbox.zmax());
}

vector<int> computePlanesInBoundingBox(const vector<Plane> &planes, const vector<Point> &points, CGAL::Bbox_3 bbox,
        double ratioReconstructed)
{
    vector<int> planeIdx;
    vector<Triangle> bboxTriangles = meshBbox(bbox);
    Tree tree(bboxTriangles.begin(), bboxTriangles.end());
    for(int i=0; i < planes.size(); i++)
    {
        if(planes[i].cumulatedPercentage > ratioReconstructed) continue;
        bool doesIntersect = false;
        for(int j=0; j < planes[i].faces.size() && !doesIntersect; j++) {
            Triangle query(points[planes[i].faces[j][0]], points[planes[i].faces[j][1]],
                    points[planes[i].faces[j][2]]);
            doesIntersect = isInBbox(query, bbox) || tree.do_intersect(query);
        }
        if(doesIntersect)
            planeIdx.push_back(i);
    }
    return planeIdx;
}

void subdivideBbox(stack<CGAL::Bbox_3> &bboxes, CGAL::Bbox_3 curBbox)
{
    //  subdivide pillar and add the 4 resulting pillars to the stack
    //  we subdivide along x axis which is arbitrary
    double xHalf = curBbox.xmin() + (curBbox.xmax() - curBbox.xmin()) / 2.;
    double yHalf = curBbox.ymin() + (curBbox.ymax() - curBbox.ymin()) / 2.;
    bboxes.emplace(curBbox.xmin(), curBbox.ymin(), curBbox.zmin(),
                   xHalf, yHalf, curBbox.zmax());
    bboxes.emplace(xHalf, curBbox.ymin(), curBbox.zmin(),
                   curBbox.xmax(), yHalf, curBbox.zmax());
    bboxes.emplace(curBbox.xmin(), yHalf, curBbox.zmin(),
                   xHalf, curBbox.ymax(), curBbox.zmax());
    bboxes.emplace(xHalf, yHalf, curBbox.zmin(),
                   curBbox.xmax(), curBbox.ymax(), curBbox.zmax());
}

//void subdivideBboxLongestAxis(stack<CGAL::Bbox_3> &bboxes, CGAL::Bbox_3 curBbox) {
void subdivideBboxLongestAxis(queue<CGAL::Bbox_3> &bboxes, CGAL::Bbox_3 curBbox) {
    // Subdivides curBbox in 2 along its longest axis
    int longestDim = -1;
    double dimX = curBbox.xmax() - curBbox.xmin();
    double dimY = curBbox.ymax() - curBbox.ymin();
    double dimZ = curBbox.zmax() - curBbox.zmin();
    if (dimX >= dimY && dimX >= dimZ) {
        bboxes.emplace(curBbox.xmin(), curBbox.ymin(), curBbox.zmin(),
                       curBbox.xmin() + dimX / 2., curBbox.ymax(), curBbox.zmax());
        bboxes.emplace(curBbox.xmin() + dimX / 2., curBbox.ymin(), curBbox.zmin(),
                       curBbox.xmax(), curBbox.ymax(), curBbox.zmax());
    } else if (dimY >= dimZ && dimY >= dimX) {
        bboxes.emplace(curBbox.xmin(), curBbox.ymin(), curBbox.zmin(),
                       curBbox.xmax(), curBbox.ymin() + dimY / 2., curBbox.zmax());
        bboxes.emplace(curBbox.xmin(), curBbox.ymin() + dimY / 2., curBbox.zmin(),
                       curBbox.xmax(), curBbox.ymax(), curBbox.zmax());
    } else {
        bboxes.emplace(curBbox.xmin(), curBbox.ymin(), curBbox.zmin(),
                       curBbox.xmax(), curBbox.ymax(), curBbox.zmin() + dimZ / 2.);
        bboxes.emplace(curBbox.xmin(), curBbox.ymin(), curBbox.zmin() + dimZ / 2.,
                       curBbox.xmax(), curBbox.ymax(), curBbox.zmax());
    }
}

vector<Json>
splitArrangementInBatch(const PlaneArrangement &planeArr, vector<facesLabelName> &labeledShapes, int nbClasses,
        double step, int maxNodes, const pair<vector<Point>, vector<int>> &labeledPointCloud,
        const vector<Point> &pointOfViews, int maxNbPlanes, int nbSamplesPerCell, double proba, bool geom,
        double ratioReconstructed, bool verbose) {

    vector<Json> computedArrangements;

//    // Computing steps along x axis
//    vector<double> xSteps;
//    for (int i = 0; i < int(floor((planeArr.bbox().xmax() - planeArr.bbox().xmin()) / step)); i++)
//        xSteps.push_back(planeArr.bbox().xmin() + double(i) * step);
//    xSteps.push_back(planeArr.bbox().xmax());
//
//    // Computing steps along y axis
//    vector<double> ySteps;
//    for(int j = 0; j < int(floor((planeArr.bbox().ymax() - planeArr.bbox().ymin()) / step)); j++)
//        ySteps.push_back(planeArr.bbox().ymin() + double(j) * step);
//    ySteps.push_back(planeArr.bbox().ymax());

    // Computing initial bboxes
    queue<CGAL::Bbox_3> bboxes;
    bboxes.push(planeArr.bbox());
    // Pillar Initialization. May be removed in the future
    /*for(int i = 0; i < xSteps.size() - 1; i++)
        for(int j = 0; j < ySteps.size() - 1; j++)
            bboxes.emplace(xSteps[i], ySteps[j], planeArr.bbox().zmin(),
                           xSteps[i + 1], ySteps[j + 1], planeArr.bbox().zmax());*/
    while(!bboxes.empty()) {
        if(verbose)
            cout << endl << "Bbox queue size: \033[1;31m"  << bboxes.size() << "\033[0m" << endl;
        CGAL::Bbox_3 curBbox = bboxes.front();
        bboxes.pop();
        // Compute planes in current pillar
        vector<int> validPlaneIdx = computePlanesInBoundingBox(planeArr.planes(), planeArr.points(), curBbox,
                ratioReconstructed);
        if(validPlaneIdx.size() > maxNbPlanes)
        {
            subdivideBboxLongestAxis(bboxes, curBbox);
            continue;
        }
        // Compute plane arrangement
        if(validPlaneIdx.empty())
            continue;
        if(verbose)
            cout << "Found " << validPlaneIdx.size() << " valid planes in current bbox which is: " << curBbox << endl;
        auto fullArrangement = PlaneArrangement(planeArr.planes(), validPlaneIdx, curBbox);
        auto& onlyArrangement = fullArrangement.arrangement();
        // Compute number of cells
        int nbCells = 0;
        for(auto cellIt = onlyArrangement.cells_begin(); cellIt != onlyArrangement.cells_end(); cellIt++)
            if(onlyArrangement.is_cell_bounded(*cellIt))
                nbCells++;
        // We need at least 2 nodes to have a valid chunk
        if(nbCells < 2) continue;
        if(nbCells <= maxNodes) {
            // labelling, feature computing
            vector<int> gtLabels = assignLabel(fullArrangement,
                                    labeledShapes, nbClasses,  nbSamplesPerCell, verbose);
            NodeFeatures nodeFeats = computeNodeFeatures(fullArrangement, gtLabels, proba, geom, verbose);

            // Compiling into json
            Json data;
            Json cell2labelJ;
            for(auto idx: fullArrangement.cell2label())
                cell2labelJ[to_string(idx.first)] = idx.second;
            data["map"] = cell2labelJ;
            data["NodeFeatures"] = nodeFeats;
            if(labeledPointCloud.first.empty())
                data["EdgeFeatures"] = computeTrivialEdgeFeatures(fullArrangement, gtLabels, nbClasses, verbose);
            else {
                vector<double> nodeVisibility;
                data["EdgeFeatures"] = computeFeaturesFromLabeledPoints(fullArrangement, labeledPointCloud.first,
                                                                        labeledPointCloud.second, nbClasses,
                                                                        nbSamplesPerCell, nodeVisibility,
                                                                        pointOfViews, verbose);
                data["NodeVisibility"] = nodeVisibility;
            }
            data["gtLabels"] = gtLabels;
            data["NodePoints"] = getCellsPoints(fullArrangement.cell2label(), onlyArrangement);
            data["NodeBbox"] = getCellsBbox(fullArrangement.cell2label(), onlyArrangement);
            data["NodeVolumes"] = fullArrangement.computeAllNodesVolumes();
            vector<Plane> currentPlanes;
            for(int validIdx: validPlaneIdx)
                currentPlanes.push_back(planeArr.planes()[validIdx]);
            data["planes"] = currentPlanes;
            data["bbox"] = curBbox;
            data["nbPlanes"] = validPlaneIdx.size();

            // Adding to the valid arrangements
            computedArrangements.push_back(data);
        }
        else
            subdivideBboxLongestAxis(bboxes, curBbox);
    }

    return computedArrangements;
}

pair<Matrix, PointRg> computeTransform(const Eigen::MatrixXd &rotPoints)
{

    // Compute the covariance matrix of the input points
    PointRg center = rotPoints.colwise().mean();
    Eigen::MatrixXd centered = rotPoints.rowwise() - center.transpose();
    Eigen::MatrixXd cov = centered.adjoint() * centered;

    // Find its eigen values/vectors
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);

    // Build the rotation matrix out of the 2 main axis
    Matrix rot;
    rot.row(0) = eig.eigenvectors().real().col(2).normalized();
    Eigen::Vector3d candidate = eig.eigenvectors().real().col(1).normalized();
    rot.row(1) = (candidate.transpose() - candidate.transpose().dot(rot.row(0))*rot.row(0)).normalized();
    rot.row(2) = rot.row(0).cross(rot.row(1));

    return make_pair(rot, center);
}

void sampleBetweenPoints(const vector<Kernel2::Point_3>& points, vector<pair<Point, int>> &query, int nbSamples, int faceHandle)
{
    //CGal to Eigen
    Eigen::MatrixXd eigenPoints(points.size(), 3);
    for(int i=0; i < points.size(); i++)
        for(int j=0; j < 3; j++)
            eigenPoints(i, j) = CGAL::to_double(points[i][j]);

    // Compute the transformation to better fit the bounding box
    pair<Matrix, PointRg> transform = computeTransform(eigenPoints);
    Eigen::MatrixXd transformedPoints = (transform.first*(eigenPoints.transpose().colwise() - transform.second));
    PointRg minPt = transformedPoints.rowwise().minCoeff();
    PointRg maxPt = transformedPoints.rowwise().maxCoeff();

    // Sampling points in the bounding box
    default_random_engine generator(time(nullptr));
    uniform_real_distribution<double> curXDist(minPt(0), maxPt(0));
    uniform_real_distribution<double> curYDist(minPt(1), maxPt(1));
    uniform_real_distribution<double> curZDist(minPt(2), maxPt(2));
    Eigen::MatrixXd rawSamples(nbSamples, 3);
    for(int i = 0; i < nbSamples; i++)
    {
        rawSamples(i, 0) = curXDist(generator);
        rawSamples(i, 1) = curYDist(generator);
        rawSamples(i, 2) = curZDist(generator);
    }

    // Apply inverted transformation
    Eigen::MatrixXd finalPoints = (transform.first.transpose() * rawSamples.transpose()).colwise() + transform.second;

    // Add the samples to the query vector
    for(int i=0; i< nbSamples; i++)
        query.emplace_back(Point(finalPoints(0, i), finalPoints(1, i),
                                            finalPoints(2, i)), faceHandle);
}

void refinePoint(Point &point, std::vector<Triangle> &mesh, int nbShoot)
{
//    ofstream fileOut((string) TEST_DIR + "ptView.obj");
    Tree tree(mesh.begin(), mesh.end());
    default_random_engine generator(random_device{}());
    normal_distribution<double> normalDist(0., 1.);
    for(int i=0; i < nbShoot; i++)
    {
        int level = int(double(i) / double(nbShoot) * 255.);
//        fileOut << "v " << point.x() << " " << point.y() << " " << point.z() << " 0 " << level << " " << level << endl;
        Vector direction1(normalDist(generator), normalDist(generator), normalDist(generator));
        Vector direction2(0., -direction1.z(), direction1.y());
        Vector direction3 = CGAL::cross_product(direction1, direction2);
        vector<Vector> allDirections = {direction1, direction2, direction3};
        vector<Point> allMidPoints;
        for (const auto &direction: allDirections) {
            // 1st intersection
            Ray curRayOne(point, direction);
            Ray_intersection intersection = tree.first_intersection(curRayOne);
            if (!intersection) continue;
            if (!boost::get<Point>(&(intersection->first))) continue;
            const Point *interOne = boost::get<Point>(&(intersection->first));

            // Other intersection
            Ray curRayTwo(point, -direction);
            Ray_intersection intersectionTwo = tree.first_intersection(curRayTwo);
            if (!intersectionTwo) continue;
            if (!boost::get<Point>(&(intersectionTwo->first))) continue;
            const Point *interTwo = boost::get<Point>(&(intersectionTwo->first));
            allMidPoints.push_back(*interOne + 0.5 * (*interTwo - *interOne));
        }

        // Update
        point = CGAL::ORIGIN + 0.333 * ((allMidPoints[0] - CGAL::ORIGIN) + (allMidPoints[1] - CGAL::ORIGIN)
                + (allMidPoints[2] - CGAL::ORIGIN));
    }
}


vector<Point> findPtViewInBbox(const CGAL::Bbox_3 &bbox, vector<facesLabelName> &shapesAndClasses,
                               vector<Triangle> &mesh, int nbShoot, int nbCandidates, bool verbose)
{
    vector<Point> ptViews;
    default_random_engine generator(random_device{}());
    uniform_real_distribution<double> xDist(bbox.xmin(), bbox.xmax());
    uniform_real_distribution<double> yDist(bbox.ymin(), bbox.ymax());
    uniform_real_distribution<double> zDist(bbox.zmin(), bbox.zmax());

    vector<CGAL::Bbox_3> bboxes;
    for(const auto& shape: shapesAndClasses) {
        CGAL::Bbox_3 curBbox;
        for (const auto &triangle: get<0>(shape))
            curBbox += triangle.bbox();
        bboxes.push_back(curBbox);
    }

    vector<Point> candidates;
    Tree tree(mesh.begin(), mesh.end());
    bool hasUnseenSpots = true;
    while(ptViews.size() < nbShoot || hasUnseenSpots) {
        if(verbose)
            cout << "Got " << ptViews.size() << " point of views out of " << nbShoot << endl;
        // Gather candidates
        while(candidates.size() != nbCandidates) {
            Point candidate(xDist(generator), yDist(generator), zDist(generator));
            bool outside = true;
            for (int j = 0; j < shapesAndClasses.size(); j++) {
                if (isInShape(candidate, bboxes[j], get<0>(shapesAndClasses[j]))) {
                    outside = false;
                    break;
                }
            }
            if (!outside) continue;
            candidates.push_back(candidate);
        }

        // Attribute scores to the candidates
        vector<int> allScores(candidates.size(), 0);
#pragma omp parallel for
        for(int i=0; i < candidates.size(); i++)
        {
            const auto &candidate = candidates[i];
            int curScore = 0;
            for(const auto ptView: ptViews) {
                Segment query(candidate, ptView);
                curScore += (int) tree.do_intersect(query);
            }
#pragma omp critical
            allScores[i] = curScore;
        }

        // We keep the best candidate
        int bestIdx = arg_max(allScores);
        ptViews.push_back(candidates[bestIdx]);

        // Among the remaining candidates, we keep the good ones
        vector<Point> nextCandidates;
        for(int i = 0; i < allScores.size(); i++)
        {
            if(i != bestIdx && allScores[i] == ptViews.size() - 1)
                nextCandidates.push_back(candidates[i]);
        }

        // Check if there are still unseen spots
        hasUnseenSpots = !nextCandidates.empty();

        if(verbose) {
            cout << "Best theoretical score: " << ptViews.size() - 1 << endl;
            cout << "Actual best score: " << allScores[bestIdx] << endl;
            cout << "Next cand size: " << nextCandidates.size() << " out of " << candidates.size() << endl;
        }
        candidates = nextCandidates;
    }
    return ptViews;

}


vector<Point> findPtViewInBboxWithRefine(const CGAL::Bbox_3 &bbox, vector<facesLabelName> &shapesAndClasses,
                               vector<Triangle> &mesh, int nbShoot, int nbCandidates, bool verbose)
{
    vector<Point> ptViews;
    default_random_engine generator(random_device{}());
    uniform_real_distribution<double> xDist(bbox.xmin(), bbox.xmax());
    uniform_real_distribution<double> yDist(bbox.ymin(), bbox.ymax());
    uniform_real_distribution<double> zDist(bbox.zmin(), bbox.zmax());

    vector<CGAL::Bbox_3> bboxes;
    for(const auto& shape: shapesAndClasses) {
        CGAL::Bbox_3 curBbox;
        for (const auto &triangle: get<0>(shape))
            curBbox += triangle.bbox();
        bboxes.push_back(curBbox);
    }

//    vector<Point> candidates;
    multiset<pair<Point, double>, SetComparison> candidates;
    Tree tree(mesh.begin(), mesh.end());
    tree.accelerate_distance_queries();
    bool hasUnseenSpots = true;
    while(ptViews.size() < nbShoot || hasUnseenSpots) {
        if(verbose)
            cout << "Got " << ptViews.size() << " point of views out of " << nbShoot << endl;
        // Gather candidates
        while(candidates.size() != nbCandidates) {
            Point candidate(xDist(generator), yDist(generator), zDist(generator));
            bool outside = true;
            double bestDist = DBL_MAX;
            for (int j = 0; j < shapesAndClasses.size(); j++) {
                if (isInShape(candidate, bboxes[j], get<0>(shapesAndClasses[j]))) {
                    outside = false;
                    break;
                }
                double curDist = sqrt(CGAL::to_double((tree.closest_point(candidate) - candidate).squared_length()));
                if(curDist < bestDist) bestDist = curDist;
            }
            if (!outside) continue;
//            candidates.push_back(candidate);
            candidates.insert(make_pair(candidate, bestDist));
        }

        // Attribute scores to the candidates
        vector<int> allScores(candidates.size(), 0);
        vector<pair<Point, double>> candidatesArray;
        cout << "Candidates: " << endl;
        for(const auto& cand: candidates) {
            cout << cand.first << " " << cand.second << endl;
            candidatesArray.push_back(cand);
        }

#pragma omp parallel for
        for(int i=0; i < candidates.size(); i++)
        {
            const auto &candidate = candidatesArray[i].first;
            int curScore = 0;
            for(const auto ptView: ptViews) {
                Segment query(candidate, ptView);
                curScore += (int) tree.do_intersect(query);
            }
#pragma omp critical
            allScores[i] = curScore;
        }

        // We keep the best candidate
        int bestIdx = arg_max(allScores);
        ptViews.push_back(candidatesArray[bestIdx].first);

        // Among the remaining candidates, we keep the good ones
        multiset<pair<Point, double>, SetComparison> nextCandidates;
        for(int i = 0; i < allScores.size(); i++)
        {
            if(i != bestIdx && allScores[i] == ptViews.size() - 1) {
//                Segment query(candidates[i], candidates[bestIdx]);
//                if(tree.do_intersect(query))
                    nextCandidates.insert(candidatesArray[i]);
            }
        }

        // Check if there are still unseen spots
        hasUnseenSpots = !nextCandidates.empty();

        if(verbose) {
            cout << "Best theoretical score: " << ptViews.size() - 1 << endl;
            cout << "Actual best score: " << allScores[bestIdx] << endl;
            cout << "Next cand size: " << nextCandidates.size() << " out of " << candidates.size() << endl;
        }
        candidates = nextCandidates;
    }
    return ptViews;

}

pair<vector<int>, vector<vector<int>>> mergeNodesFromVisibility(PlaneArrangement& planeArr,
                                                                const vector<double> &nodeVisibility,
                                                                double visThreshold)
{
    // Initialize mappings
    vector<int> node2merged(planeArr.cell2label().size(), 0);
    vector<vector<int>> merged2node;

    // Local algorithm variables
    vector<bool> visitedNodes(planeArr.cell2label().size(), false);
    int mergedNodeId = 0;
    int curNode = 0;

    // Initial node
    while(curNode < planeArr.cell2label().size())
    {
        // Finding an empty cell
        while ((nodeVisibility[curNode] < visThreshold || visitedNodes[curNode]) &&
        curNode < planeArr.cell2label().size()) {
            if(visitedNodes[curNode])
                curNode++;
            else {
                node2merged[curNode] = mergedNodeId;
                merged2node.push_back({curNode});
                mergedNodeId++;
                visitedNodes[curNode++] = true;
            }
        }

        // Check that we actually found something
        if(curNode >= planeArr.cell2label().size()) break;

        // Region growing
        queue<int> potentialInliers;
        potentialInliers.push(curNode);
        vector<int> currentInliers;
        while(!potentialInliers.empty())
        {
            // Get the front of the queue
            int curPotentialInlier = potentialInliers.front();
            potentialInliers.pop();

            // If the candidate does not match the criteria, we do not grow from it.
            if(nodeVisibility[curPotentialInlier] < visThreshold) continue;

            // If the candidate matches the criteria, we add it to the current merged class
            currentInliers.push_back(curPotentialInlier);
            node2merged[curPotentialInlier] = mergedNodeId;
            visitedNodes[curPotentialInlier] = true;

            // We expand the potential inliers to the non-visited neighbours of curPotentialInlier
            auto curPotentialCell = planeArr.arrangement().cell(planeArr.label2cell().at(curPotentialInlier));
            for(auto facetIt = curPotentialCell.subfaces_begin(); facetIt != curPotentialCell.subfaces_end(); facetIt++)
            {
                int cell0 = planeArr.arrangement().facet(*facetIt).superface(0);
                int cell1 = planeArr.arrangement().facet(*facetIt).superface(1);
                int goodCell = cell0 == planeArr.label2cell().at(curPotentialInlier) ? cell1 : cell0;
                if(!planeArr.arrangement().is_cell_bounded(goodCell)) continue;
                int goodCellIdx = planeArr.cell2label().at(goodCell);
                if(!visitedNodes[goodCellIdx])
                    potentialInliers.push(goodCellIdx);
            }
        }
        merged2node.push_back(currentInliers);
        mergedNodeId++;
    }
    return make_pair(node2merged, merged2node);
}