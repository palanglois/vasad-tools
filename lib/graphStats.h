#ifndef BIM_DATA_GRAPHSTATS_H
#define BIM_DATA_GRAPHSTATS_H

#include <random>
#include <stack>

#include "iogeometry.h"

typedef std::vector<std::vector<int>> Nodes;
typedef std::vector<std::pair<int, int>> Edges;

typedef std::vector<std::vector<double>> NodeFeatures;
typedef std::map<std::pair<int, int>, std::vector<int>> EdgeFeatures;

std::pair<Nodes, Edges> computeGraphStatistics(const std::vector<bool> &labels,
        const std::map<int, int> &cell2label, const Arrangement &arr, bool verbose=false);

std::pair<NodeFeatures, EdgeFeatures> computeGraph(const std::vector<int> &labels,
        const std::map<int, int> &cell2label, const Arrangement &arr, const int nbClasses, const double proba=1., const bool withGeom=false, bool verbose=false);

std::vector<std::vector<double>> getCellsPoints(const std::map<int, int> &cell2label, const Arrangement &arr);
std::vector<std::vector<double>> getCellsBbox(const std::map<int, int> &cell2label, const Arrangement &arr);

std::vector<int> assignLabel(const Arrangement &arr,const std::map<int, int> &cell2label, CGAL::Bbox_3 bbox,
        std::vector<facesLabelName> &labeledTrees, int nbClasses, int nbSamplesPerCell=40, bool verbose=false);

std::vector<int> computePlanesInBoundingBox(const std::vector<Plane> &planes, const std::vector<Point> &points, CGAL::Bbox_3 bbox);

std::vector<nlohmann::json> splitArrangementInBatch(const PlaneArrangement &planeArr,
        std::vector<facesLabelName> &labeledShapes, int nbClasses, double step, int maxNodes,
        int maxNbPlanes=250, int nbSamplesPerCell=40, double proba=1, bool geom=false, bool verbose=false);

#endif //BIM_DATA_GRAPHSTATS_H
