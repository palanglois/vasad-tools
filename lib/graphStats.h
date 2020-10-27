#ifndef BIM_DATA_GRAPHSTATS_H
#define BIM_DATA_GRAPHSTATS_H

#include <random>

#include "iogeometry.h"

typedef std::vector<std::vector<int>> Nodes;
typedef std::vector<std::pair<int, int>> Edges;

typedef std::vector<std::vector<double>> NodeFeatures;
typedef std::map<std::pair<int, int>, std::vector<int>> EdgeFeatures;

std::pair<Nodes, Edges> computeGraphStatistics(const std::vector<bool> &labels,
        const std::map<int, int> &cell2label, const Arrangement &arr, bool verbose=false);

std::pair<NodeFeatures, EdgeFeatures> computeGraph(const std::vector<int> &labels,
        const std::map<int, int> &cell2label, const Arrangement &arr, const int nbClasses, const int proba=1., const bool withGeom=false, bool verbose=false);

std::vector<std::vector<double>> getCellsPoints(const std::map<int, int> &cell2label, const Arrangement &arr);
std::vector<std::vector<double>> getCellsBbox(const std::map<int, int> &cell2label, const Arrangement &arr);

std::vector<int> assignLabel(const Arrangement &arr,const std::map<int, int> &cell2label, CGAL::Bbox_3 bbox,
        std::vector<facesLabelName> &labeledTrees, int nbSamples=10000, bool fill=true, bool verbose=false);

#endif //BIM_DATA_GRAPHSTATS_H
