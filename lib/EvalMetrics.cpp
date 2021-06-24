#include "EvalMetrics.h"

using namespace std;
using Json = nlohmann::json;

EvalMetrics::EvalMetrics(vector<facesLabelName> &predShapes, vector<facesLabelName> &gtShapes,
                         vector<classKeywordsColor> &_classes,
                         int nbSamplesVolumic, int nbSamplesSurfacic) : classes(_classes) {

    // Pred points (surfacic)
    cout << endl << "Surfacic point sampling..." << endl;
    vector<Triangle> allPredShapes;
    for(const auto& shape: predShapes)
        allPredShapes.insert(allPredShapes.end(), get<0>(shape).begin(), get<0>(shape).end());
    predPoints = samplePointsOnMesh(allPredShapes, nbSamplesSurfacic);

    // Gt points (surfacic)
    vector<Triangle> allGtShapes;
    for(const auto& shape: gtShapes)
        allGtShapes.insert(allGtShapes.end(), get<0>(shape).begin(), get<0>(shape).end());
    gtPoints = samplePointsOnMesh(allGtShapes, nbSamplesSurfacic);
    cout << "Surfacic point sampling done!" << endl;

    // NN Distances
    nnDistancesGt = findPcDistance(gtPoints, predPoints);
    nnDistancesPred = findPcDistance(predPoints, gtPoints);

    vector<CGAL::Bbox_3> predBboxes = computeShapeBboxes(predShapes);
    vector<CGAL::Bbox_3> gtBboxes = computeShapeBboxes(gtShapes);

    // Computing global bbox
    for(const auto& bbox: predBboxes)
        globalBbox += bbox;
    for(const auto& bbox: gtBboxes)
        globalBbox += bbox;

    // Sample points (volumic)
    default_random_engine generator;
    uniform_real_distribution<double> xDist(globalBbox.xmin(), globalBbox.xmax());
    uniform_real_distribution<double> yDist(globalBbox.ymin(), globalBbox.ymax());
    uniform_real_distribution<double> zDist(globalBbox.zmin(), globalBbox.zmax());
    for(int i=0; i < nbSamplesVolumic; i++)
        points.emplace_back(xDist(generator), yDist(generator), zDist(generator));

    // Pred labels
    predLabels = assignLabelToPointsWithBboxes<Point>(points, predShapes, classes.size(), globalBbox, predBboxes);

    // Gt Labels
    gtLabels = assignLabelToPointsWithBboxes(points, gtShapes, classes.size(), globalBbox, gtBboxes);;

}

vector<vector<int>> EvalMetrics::getConfusionMatrix() const {
    vector<vector<int>> matrix(classes.size() + 1, vector<int>(classes.size() + 1, 0));
    for(int i=0; i < points.size(); i++)
        matrix[gtLabels[i]][predLabels[i]]++;
    return matrix;
}

double EvalMetrics::getIoUGeometric() const {
    int countTotal = 0;
    int countMutual = 0;
    for(int i=0; i < points.size(); i++)
    {
        if(predLabels[i] != classes.size() || gtLabels[i] != classes.size()) {
            countTotal++;
            countMutual += (predLabels[i] != classes.size() && gtLabels[i] != classes.size());
        }
    }
    return double(countMutual) / double(countTotal);
}



double EvalMetrics::getIoU() const {
    int countTotal = 0;
    int countMutual = 0;
    for(int i=0; i < points.size(); i++)
    {
        if(predLabels[i] != classes.size() || gtLabels[i] != classes.size()) {
            countTotal++;
            countMutual += predLabels[i] == gtLabels[i];
        }
    }
    return double(countMutual) / double(countTotal);
}

double EvalMetrics::meanSurfacicDistance() const {
    double accum = 0.;
    for(const auto& measurement: nnDistancesPred)
        accum += measurement;
    for(const auto& measurement: nnDistancesGt)
        accum += measurement;
    return accum / double(nnDistancesPred.size() + nnDistancesGt.size());
}

double EvalMetrics::maxSurfacicDistance() const {
    return max(*nnDistancesPred.rbegin(), *nnDistancesGt.rbegin());
}

double EvalMetrics::precision(double threshold) const {
    auto itOne = nnDistancesGt.begin();
    advance(itOne, int(threshold * (nnDistancesGt.size() - 1)));
    return *itOne;
}

double EvalMetrics::recall(double threshold) const {
    auto itTwo = nnDistancesPred.begin();
    advance(itTwo, int(threshold * (nnDistancesPred.size() - 1)));
    return *itTwo;
}

void EvalMetrics::saveAsJson(const string &path, double threshold) const {

    Json outputData = {{"iou",                    getIoU()},
                       {"iou_geometric",          getIoUGeometric()},
                       {"confusion_matrix",       getConfusionMatrix()},
                       {"mean_surfacic_distance", meanSurfacicDistance()},
                       {"max_surfacic_distance",  maxSurfacicDistance()},
                       {"threshold",              threshold},
                       {"precision",              precision(threshold)},
                       {"recall",                 recall(threshold)}};

    // Output the json file
    ofstream outFile(path);
    outFile << outputData;
}

vector<Point> samplePointsOnMesh(const vector<Triangle>& mesh, int nbSamples)
{
    // Build the cumulated areas histogram
    vector<double> areas(mesh.size(), 0.);
#pragma omp parallel for
    for(int i=0; i < mesh.size(); i++)
        areas[i]= sqrt(mesh[i].squared_area());
    default_random_engine generator(time(nullptr));
    discrete_distribution<> d(areas.begin(), areas.end());
    // Actual sampling
    vector<Point> sampledPoints(nbSamples);
#pragma omp parallel for
    for(int i=0; i < nbSamples; i++)
    {
        // Select a random triangle according to the areas distribution
        size_t found_index = d(generator);

        // Draw a random point in this triangle
        double r1 = ((double) rand() / (RAND_MAX));
        double r2 = ((double) rand() / (RAND_MAX));
        Point A = mesh[found_index][0];
        Point B = mesh[found_index][1];
        Point C = mesh[found_index][2];
        Point P = CGAL::ORIGIN +
                  (1 - sqrt(r1)) * (A - CGAL::ORIGIN) +
                  (sqrt(r1) * (1 - r2)) * (B - CGAL::ORIGIN) +
                  (sqrt(r1) * r2) * (C - CGAL::ORIGIN);
#pragma omp critical
        sampledPoints[i] = P;
    }
    return sampledPoints;
}

multiset<double> findPcDistance(const vector<Point>& refPointCloud, const vector<Point>& queryPointCloud)
{
    // Build the kd-tree for the reference Point Cloud
    kdTree tree(refPointCloud.begin(), refPointCloud.end());

    // do a knn search
    int itr = 0;
    multiset<double> allDistances;
#pragma omp parallel for
    for(int i=0; i < queryPointCloud.size(); i++)
    {
        Neighbor_search search(tree, queryPointCloud[i], 1);
#pragma omp atomic
        itr++;
#pragma omp critical
        {
            allDistances.insert(sqrt(search.begin()->second));
            if(itr % 10000 == 0)
                cout << "Nb itr: " << itr << "/" << queryPointCloud.size() << endl;
        }
    }
    return allDistances;
}
