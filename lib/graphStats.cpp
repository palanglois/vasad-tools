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

vector<int> assignLabel(const Arrangement &arr, const map<int, int> &cell2label, CGAL::Bbox_3 bbox,
                        vector<facesLabelName> &labeledShapes, int nbClasses,
                        int nbSamplesPerCell, bool verbose)
{
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
    uniform_real_distribution<double> xDist(bbox.xmin(), bbox.xmax());
    uniform_real_distribution<double> yDist(bbox.ymin(), bbox.ymax());
    uniform_real_distribution<double> zDist(bbox.zmin(), bbox.zmax());
    vector<pair<Point, int>> queryPoints;

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
    for(auto cellIt = arr.cells_begin(); cellIt != arr.cells_end(); cellIt++)
    {
        if(!arr.is_cell_bounded(*cellIt)) continue;
        vector<Kernel2::Point_3> points;
        for(auto facetIt = cellIt->subfaces_begin(); facetIt != cellIt->subfaces_end(); facetIt++)
        {
            auto facet = arr.facet(*facetIt);
            for(auto edgeIt = facet.subfaces_begin(); edgeIt != facet.subfaces_end(); edgeIt++) {
                auto edge = arr.edge(*edgeIt);
                for(auto pointIt = edge.subfaces_begin(); pointIt != edge.subfaces_end(); pointIt++)
                    points.push_back(arr.point(*pointIt));
            }
        }

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

//            // DEBUG
//            if(int(arr.cell_handle(*cellIt)) == 125161) {
//                for (int i = queryPoints.size() - 40; i < queryPoints.size(); i++)
//                    cout << "Here " << queryPoints[i].first << endl;
//                cout << "Bbox " << curBbox << endl;
//                for(const auto& point: points)
//                    cout << "Point " << point << " bbox " << point.bbox() << endl;
//            }
    }

    // For every point, test it for every shape
    Tree tree;
    vector<int> pointsLabel(queryPoints.size(), voidClass);
    auto tqLabeledShapes = tq::trange(labeledShapes.size()); // works for rvalues too!
    tqLabeledShapes.set_prefix("Labeling each point: ");
    for (int j : tqLabeledShapes)
    {
        auto &labeledTree = labeledShapes[j];
        tree.rebuild(get<0>(labeledTree).begin(), get<0>(labeledTree).end());
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
                cout << "Potential issue with shape " << get<2>(labeledTree) << endl;

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
        if(verbose && (i % 100000 == 0))
            cout << "Processed " << i << " points out of " << queryPoints.size() << endl;
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
////    Point query(37.7275666666667, -30.2528, 8.59148666666667);
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
//    TriangleColorMap colorMap;
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

pair<NodeFeatures, EdgeFeatures>
computeGraph(const vector<int> &labels, const map<int, int> &cell2label, const Arrangement &arr,
        const int nbClasses, const double proba, const bool withGeom, bool verbose) {

    // Graph nodes
    default_random_engine generator;
    uniform_real_distribution<double> unitRandom(0., 1.);
    NodeFeatures nodeFeatures(cell2label.size(), vector<double>(1, 0));
    for(auto cellIt = arr.cells_begin(); cellIt != arr.cells_end(); cellIt++)
    {
        if(!arr.is_cell_bounded(*cellIt)) continue;
        auto cellHandle = arr.cell_handle(*cellIt);
        int labelIdx = cell2label.at(cellHandle);
	double feature = 1.; // We suppose the current cell is full
	if(labels[labelIdx] == -1)
	{
	    // If it was actually empty, we only say it in proba% of the cases
	    if(unitRandom(generator) < proba)
	        feature = 0.;
	}
	    
        nodeFeatures[labelIdx] = {feature};

	// Node geometry
	if(withGeom)
	{
            vector<double> curBbox = {DBL_MAX, DBL_MAX, DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX};
            for(auto facetIt = cellIt->subfaces_begin(); facetIt != cellIt->subfaces_end(); facetIt++)
            {
                auto facet = arr.facet(*facetIt);
                for(auto edgeIt = facet.subfaces_begin(); edgeIt != facet.subfaces_end(); edgeIt++) {
                    auto edge = arr.edge(*edgeIt);
                    for(auto pointIt = edge.subfaces_begin(); pointIt != edge.subfaces_end(); pointIt++)
                    {
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
            vector<int> edgeFeature(nbClasses + 1, 0);
            if(cellLabel1 == -1 && cellLabel2 != -1)
            {
                // We're at a void/full transition
                edgeFeature[cellLabel2] = 1;
                edgeFeatures[make_pair(cellFeatureIdx1, cellFeatureIdx2)] = edgeFeature;
            }
            else if(cellLabel1 != -1 && cellLabel2 == -1)
            {
                // Same here but different direction
                edgeFeature[cellLabel1] = 1;
                edgeFeatures[make_pair(cellFeatureIdx1, cellFeatureIdx2)] = edgeFeature;
            }
            else
            {
                // We're not at a transition
                edgeFeature[nbClasses] = 1;
                edgeFeatures[make_pair(cellFeatureIdx1, cellFeatureIdx2)] = edgeFeature;
            }
        }
    }

    return make_pair(nodeFeatures, edgeFeatures);
}

vector<vector<double>> getCellsPoints(const map<int, int> &cell2label, const Arrangement &arr) {
    vector<vector<double>> cellsPoints(cell2label.size(), {0., 0., 0.});
    Epeck_to_Simple e2s;

    for(auto cellIt = arr.cells_begin(); cellIt != arr.cells_end(); cellIt++) {
        if(!arr.is_cell_bounded(*cellIt)) continue;

        auto pt = e2s(cellIt->point());
        cellsPoints[cell2label.at(arr.cell_handle(*cellIt))] = {pt.x(), pt.y(), pt.z()};
    }

    return cellsPoints;
}

vector<vector<double>> getCellsBbox(const map<int, int> &cell2label, const Arrangement &arr) {
    vector<vector<double>> cellsPoints(cell2label.size(), {0., 0., 0.});
    Epeck_to_Simple e2s;

    for(auto cellIt = arr.cells_begin(); cellIt != arr.cells_end(); cellIt++) {
        if(!arr.is_cell_bounded(*cellIt)) continue;
        cellsPoints[cell2label.at(arr.cell_handle(*cellIt))] = {DBL_MAX, DBL_MAX, DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX};
        for(auto facetIt = cellIt->subfaces_begin(); facetIt != cellIt->subfaces_end(); facetIt++)
        {
            auto facet = arr.facet(*facetIt);
            for(auto edgeIt = facet.subfaces_begin(); edgeIt != facet.subfaces_end(); edgeIt++) {
                auto edge = arr.edge(*edgeIt);
                for(auto pointIt = edge.subfaces_begin(); pointIt != edge.subfaces_end(); pointIt++)
		{
		    cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][0] = min(cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][0], CGAL::to_double(arr.point(*pointIt).x()));
		    cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][1] = min(cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][1], CGAL::to_double(arr.point(*pointIt).y()));
		    cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][2] = min(cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][2], CGAL::to_double(arr.point(*pointIt).z()));
		    cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][3] = max(cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][3], CGAL::to_double(arr.point(*pointIt).x()));
		    cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][4] = max(cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][4], CGAL::to_double(arr.point(*pointIt).y()));
		    cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][5] = max(cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][5], CGAL::to_double(arr.point(*pointIt).z()));
		}
            }
        }
    }

    return cellsPoints;
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
    vector<Point> bboxPoints = {
            Point(bbox.xmin(), bbox.ymin(), bbox.zmin()),
            Point(bbox.xmin(), bbox.ymin(), bbox.zmax()),
            Point(bbox.xmin(), bbox.ymax(), bbox.zmin()),
            Point(bbox.xmin(), bbox.ymax(), bbox.zmax()),
            Point(bbox.xmax(), bbox.ymin(), bbox.zmin()),
            Point(bbox.xmax(), bbox.ymin(), bbox.zmax()),
            Point(bbox.xmax(), bbox.ymax(), bbox.zmin()),
            Point(bbox.xmax(), bbox.ymax(), bbox.zmax()),
    };
    vector<Triangle> bboxTriangles = {
            Triangle(bboxPoints[0], bboxPoints[1], bboxPoints[5]),
            Triangle(bboxPoints[0], bboxPoints[5], bboxPoints[4]),
            Triangle(bboxPoints[4], bboxPoints[5], bboxPoints[6]),
            Triangle(bboxPoints[6], bboxPoints[5], bboxPoints[7]),
            Triangle(bboxPoints[1], bboxPoints[3], bboxPoints[5]),
            Triangle(bboxPoints[5], bboxPoints[3], bboxPoints[7]),
            Triangle(bboxPoints[0], bboxPoints[4], bboxPoints[2]),
            Triangle(bboxPoints[2], bboxPoints[4], bboxPoints[6]),
            Triangle(bboxPoints[0], bboxPoints[2], bboxPoints[1]),
            Triangle(bboxPoints[1], bboxPoints[2], bboxPoints[3]),
            Triangle(bboxPoints[2], bboxPoints[7], bboxPoints[3]),
            Triangle(bboxPoints[2], bboxPoints[6], bboxPoints[7]),
    };
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

vector<Json>
splitArrangementInBatch(const PlaneArrangement &planeArr, vector<facesLabelName> &labeledShapes, int nbClasses,
        double step, int maxNodes, int maxNbPlanes, int nbSamplesPerCell, double proba, bool geom, double ratioReconstructed,
        bool verbose) {

    vector<Json> computedArrangements;

    // Computing steps along x axis
    vector<double> xSteps;
    for (int i = 0; i < int(floor((planeArr.bbox().xmax() - planeArr.bbox().xmin()) / step)); i++)
        xSteps.push_back(planeArr.bbox().xmin() + double(i) * step);
    xSteps.push_back(planeArr.bbox().xmax());

    // Computing steps along y axis
    vector<double> ySteps;
    for(int j = 0; j < int(floor((planeArr.bbox().ymax() - planeArr.bbox().ymin()) / step)); j++)
        ySteps.push_back(planeArr.bbox().ymin() + double(j) * step);
    ySteps.push_back(planeArr.bbox().ymax());

    // Computing initial bboxes
    stack<CGAL::Bbox_3> bboxes;
    for(int i = 0; i < xSteps.size() - 1; i++)
        for(int j = 0; j < ySteps.size() - 1; j++)
            bboxes.emplace(xSteps[i], ySteps[j], planeArr.bbox().zmin(),
                           xSteps[i + 1], ySteps[j + 1], planeArr.bbox().zmax());
    while(!bboxes.empty()) {
        if(verbose)
            cout << "Bbox stack size: " << bboxes.size() << endl;
        CGAL::Bbox_3 curBbox = bboxes.top();
        bboxes.pop();
        // Compute planes in current pillar
        vector<int> validPlaneIdx = computePlanesInBoundingBox(planeArr.planes(), planeArr.points(), curBbox,
                ratioReconstructed);
        if(validPlaneIdx.size() > maxNbPlanes)
        {
            subdivideBbox(bboxes, curBbox);
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
            vector<int> gtLabels = assignLabel(onlyArrangement, fullArrangement.cell2label(), curBbox,
                                    labeledShapes, nbClasses,  nbSamplesPerCell, verbose);
            pair<NodeFeatures, EdgeFeatures> nodesEdges = computeGraph(gtLabels, fullArrangement.cell2label(),
                    onlyArrangement, nbClasses, proba, geom, true);

            // Compiling into json
            Json data;
            Json cell2labelJ;
            for(auto idx: fullArrangement.cell2label())
                cell2labelJ[to_string(idx.first)] = idx.second;
            data["map"] = cell2labelJ;
            data["NodeFeatures"] = nodesEdges.first;
            data["EdgeFeatures"] = nodesEdges.second;
            data["gtLabels"] = gtLabels;
            data["NodePoints"] = getCellsPoints(fullArrangement.cell2label(), onlyArrangement);
            data["NodeBbox"] = getCellsBbox(fullArrangement.cell2label(), onlyArrangement);
            vector<Plane> currentPlanes;
            for(int validIdx: validPlaneIdx)
                currentPlanes.push_back(planeArr.planes()[validIdx]);
            data["planes"] = currentPlanes;
            data["bbox"] = {curBbox.xmin(), curBbox.ymin(), curBbox.zmin(), curBbox.xmax(), curBbox.ymax(), curBbox.zmax()};
            data["nbPlanes"] = validPlaneIdx.size();

            // Adding to the valid arrangements
            computedArrangements.push_back(data);
        }
        else
            subdivideBbox(bboxes, curBbox);
    }

    return computedArrangements;
}
