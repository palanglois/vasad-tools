#include "graphStats.h"

using namespace std;

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
                        vector<std::pair<std::vector<Triangle>, int>> &labeledShapes, int nbSamples, bool fill, bool verbose)
{
    Simple_to_Epeck s2e;
    Epeck_to_Simple e2s;
    //Build bbox for each shape
    vector<CGAL::Bbox_3> bboxes;
    for(const auto& labeledShape: labeledShapes)
        bboxes.push_back(CGAL::bbox_3(labeledShape.first.begin(), labeledShape.first.end()));
    // Number of classes to consider
    int nbClasses = 0;
    for(auto &labeledTree: labeledShapes)
        if(labeledTree.second + 2 > nbClasses)
            nbClasses = labeledTree.second + 2;
    int voidClass = nbClasses - 1;

    vector<int> labels;

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

    vector<vector<int>> votes(cell2label.size(), vector<int>(nbClasses, 0));


    //DEBUG
//    cout << testRef << endl;
//    cout << endl;
    //END DEBUG
    //queryPoints.reserve(nbSamples);
    //for(int i=0; i < nbSamples; i++)
        //queryPoints.emplace_back(Point(xDist(generator), yDist(generator), zDist(generator)), -1);
    if(fill)
    {
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
            for(int i=0; i< 40; i++)
                queryPoints.emplace_back(Point(curXDist(generator), curYDist(generator), curZDist(generator)), arr.cell_handle(*cellIt));

//            // DEBUG
//            if(int(arr.cell_handle(*cellIt)) == 35480) {
//                for (int i = queryPoints.size() - 40; i < queryPoints.size(); i++)
//                    cout << "Here " << queryPoints[i].first << endl;
//                cout << "Bbox " << curBbox << endl;
//                for(const auto& point: points)
//                    cout << "Point " << point << " bbox " << point.bbox() << endl;
//            }
        }
    }

    // For every point, test it for every shape
    Tree tree;
    vector<int> pointsLabel(queryPoints.size(), voidClass);
    for(int j=0; j < labeledShapes.size(); j++)
    {
        if(verbose)
            cout << "Processed " << j << " shapes out of " << labeledShapes.size() << endl;
        auto &labeledTree = labeledShapes[j];
        tree.rebuild(labeledTree.first.begin(), labeledTree.first.end());
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
                cout << odd << endl;

            // Test the nb of intersections for parity
            if(odd >= 2)
            {
                // In the current shape
#pragma omp critical
                pointsLabel[i] = labeledTree.second;
                //cout << "Here " << labeledTree.second << endl;
            }
        }
    }
    for(int i=0; i < queryPoints.size(); i++) {
        // Find the current cell
        Arrangement::Face_handle cellHandle;
        if(queryPoints[i].second == -1)
            cellHandle = find_containing_cell(arr, s2e(queryPoints[i].first));
        else
            cellHandle = find_containing_cell(arr, s2e(queryPoints[i].first), queryPoints[i].second);
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
    for(const auto & vote : votes)
        labels.push_back(arg_max(vote) == voidClass ? -1 : arg_max(vote));

    if(verbose) {
        for (int i = 0; i < votes.size(); i++) {
            bool display = false;
            for(int j=0; j < votes[i].size() - 1; j++)
                if(votes[i][j] != 0)
                    display = true;
            if(display) {
                cout << "Cell vote for cell " << i << ": ";
                for (auto nb: votes[i])
                    cout << nb << " ";
                cout << endl;
            }
        }
    }

//    //DEBUG
//    cout << "DEBUG" << endl;
////    Point query(37.7275666666667, -30.2528, 8.59148666666667);
//    Point query(34.5212666666667, -30.2528, 8.17148666666667);
//    Arrangement::Face_handle debugCell = find_containing_cell(arr, s2e(query));
//    cout << "Cell id: " << debugCell << endl;
//    cout << "votes: " << endl;
//
//    if(arr.is_cell_bounded(debugCell)) {
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
        const int nbClasses, bool verbose) {

    // Graph nodes
    NodeFeatures nodeFeatures(cell2label.size(), vector<int>(1, 0));
    for(auto cellIt = arr.cells_begin(); cellIt != arr.cells_end(); cellIt++)
    {
        if(!arr.is_cell_bounded(*cellIt)) continue;
        auto cellHandle = arr.cell_handle(*cellIt);
        int labelIdx = cell2label.at(cellHandle);
        nodeFeatures[labelIdx] = {int(labels[labelIdx] != -1)};
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
