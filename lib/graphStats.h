#ifndef BIM_DATA_GRAPHSTATS_H
#define BIM_DATA_GRAPHSTATS_H

#include <random>

#include "iogeometry.h"

typedef std::vector<std::vector<int>> Nodes;
typedef std::vector<std::pair<int, int>> Edges;

std::pair<Nodes, Edges> computeGraphStatistics(const std::vector<bool> &labels,
        const std::map<int, int> &cell2label, const Arrangement &arr, bool verbose=false);

std::vector<int> assignLabel(const Arrangement &arr,const std::map<int, int> &cell2label, CGAL::Bbox_3 bbox,
        const std::vector<std::pair<Tree*, int>> &labeledTrees, int nbSamples=10000, bool verbose=false);

#endif //BIM_DATA_GRAPHSTATS_H
