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

vector<int> assignLabel(const Arrangement &arr,const map<int, int> &cell2label, CGAL::Bbox_3 bbox,
                        const vector<std::pair<Tree*, int>> &labeledTrees, int nbSamples, bool verbose)
{
    // Number of classes to consider
    int nbClasses = 0;
    for(auto &labeledTree: labeledTrees)
        if(labeledTree.second + 2 > nbClasses)
            nbClasses = labeledTree.second + 2;
    int voidClass = nbClasses - 1;

    vector<int> labels;

    // Draw points in the arrangement
    default_random_engine generator;
    uniform_real_distribution<double> xDist(bbox.xmin(), bbox.xmax());
    uniform_real_distribution<double> yDist(bbox.ymin(), bbox.ymax());
    uniform_real_distribution<double> zDist(bbox.zmin(), bbox.zmax());
    vector<Point> queryPoints;
    vector<vector<int>> votes(cell2label.size(), vector<int>(nbClasses, 0));
    queryPoints.reserve(nbSamples);
    for(int i=0; i < nbSamples; i++)
        queryPoints.emplace_back(xDist(generator), yDist(generator),zDist(generator));

    // For every point, test it for every shape
    Simple_to_Epeck s2e;
    for(int i=0; i < queryPoints.size(); i++)
    {
        auto &point = queryPoints[i];
        int label = voidClass;
        for(auto &labeledTree: labeledTrees)
        {
            //Make a random query ray and intersect it
            Ray query(point, Vector(1., 0., 0.));
            list<Ray_intersection> intersections;
            labeledTree.first->all_intersections(query, back_inserter(intersections));

            // Test the nb of intersections for parity
            if(intersections.size() % 2 == 1)
            {
                // In the current shape
                label = labeledTree.second;
            }
        }
        // Find the current cell
        Arrangement::Face_handle cellHandle = find_containing_cell(arr, s2e(point));
        votes[cell2label.at(cellHandle)][label]++;

        if(verbose)
            if((100*i) % (10*queryPoints.size()) == 0)
                cout << "Processed " << i << " points out of " << queryPoints.size() << endl;

    }
    for(const auto & vote : votes)
        labels.push_back(arg_max(vote));

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
