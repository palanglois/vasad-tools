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
            auto curBbox = CGAL::bbox_3(points.begin(), points.end());
            uniform_real_distribution<double> curXDist(curBbox.xmin(), curBbox.xmax());
            uniform_real_distribution<double> curYDist(curBbox.ymin(), curBbox.ymax());
            uniform_real_distribution<double> curZDist(curBbox.zmin(), curBbox.zmax());
            for(int i=0; i< 40; i++)
                queryPoints.emplace_back(Point(curXDist(generator), curYDist(generator), curZDist(generator)), arr.cell_handle(*cellIt));
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
            Ray query(point, Vector(1., 0., 0.));
            list<Ray_intersection> intersections;
            tree.all_intersections(query, back_inserter(intersections));

            // Test the nb of intersections for parity
            if(intersections.size() % 2 == 1)
            {
                // In the current shape
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
        votes[cell2label.at(cellHandle)][pointsLabel[i]]++;
        if(verbose && (i % 10000 == 0))
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

    return labels;
}
