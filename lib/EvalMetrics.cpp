#include "EvalMetrics.h"

using namespace std;

EvalMetrics::EvalMetrics(vector<facesLabelName> &predShapes, vector<facesLabelName> &gtShapes,
                         vector<classKeywordsColor> &_classes,
                         int nbSamples) : classes(_classes) {

    vector<CGAL::Bbox_3> predBboxes = computeShapeBboxes(predShapes);
    vector<CGAL::Bbox_3> gtBboxes = computeShapeBboxes(gtShapes);

    // Computing global bbox
    for(const auto& bbox: predBboxes)
        globalBbox += bbox;
    for(const auto& bbox: gtBboxes)
        globalBbox += bbox;

    // Sample points
    default_random_engine generator;
    uniform_real_distribution<double> xDist(globalBbox.xmin(), globalBbox.xmax());
    uniform_real_distribution<double> yDist(globalBbox.ymin(), globalBbox.ymax());
    uniform_real_distribution<double> zDist(globalBbox.zmin(), globalBbox.zmax());
    for(int i=0; i < nbSamples; i++)
        points.emplace_back(xDist(generator), yDist(generator), zDist(generator));

    // Pred labels
    predLabels = assignLabelToPointsWithBboxes<Point>(points, predShapes, classes.size(), globalBbox, predBboxes);

    // Gt Labels
    gtLabels = assignLabelToPointsWithBboxes(points, gtShapes, classes.size(), globalBbox, gtBboxes);
    cout << "debug" << endl;

}

vector<vector<int>> EvalMetrics::getConfusionMatrix() const {
    vector<vector<int>> matrix(classes.size() + 1, vector<int>(classes.size() + 1, 0));
    for(int i=0; i < points.size(); i++)
        matrix[gtLabels[i]][predLabels[i]]++;
    return matrix;
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