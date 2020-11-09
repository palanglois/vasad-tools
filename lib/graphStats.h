#ifndef BIM_DATA_GRAPHSTATS_H
#define BIM_DATA_GRAPHSTATS_H

#include <stack>

#include <Eigen/Eigenvalues>

#include "iogeometry.h"
#include "RegionGrowing.h"

std::pair<Nodes, Edges> computeGraphStatistics(const std::vector<bool> &labels,
        const std::map<int, int> &cell2label, const Arrangement &arr, bool verbose=false);

std::pair<NodeFeatures, EdgeFeatures> computeGraph(const std::vector<int> &labels,
                                                   const std::map<int, int> &cell2label, const Arrangement &arr,
                                                   const int nbClasses, const double proba=1.,
                                                   const bool withGeom=false, bool verbose=false);

inline double computeFacetArea(const Arrangement &arr, int facetHandle);
std::pair<std::vector<Point>, std::map<Point, int>> sampleFacets(const Arrangement &arr, std::map<int, double> &facetAreas);

EdgeFeatures computeFeaturesFromLabeledPoints(PlaneArrangement &planeArr, const std::vector<Point> &points,
                                              const std::vector<int> &labels, const int nbClasses,
                                              int nbSamplesPerCell, bool verbose=false);

std::vector<int> assignLabel(PlaneArrangement& planeArr,
        std::vector<facesLabelName> &labeledTrees, int nbClasses, int nbSamplesPerCell=40, bool verbose=false);

std::vector<int> computePlanesInBoundingBox(const std::vector<Plane> &planes, const std::vector<Point> &points, CGAL::Bbox_3 bbox, double ratioReconstructed=0.98);

std::vector<nlohmann::json> splitArrangementInBatch(const PlaneArrangement &planeArr,
        std::vector<facesLabelName> &labeledShapes, int nbClasses, double step, int maxNodes,
        const std::pair<std::vector<Point>, std::vector<int>> &labeledPointCloud,
        int maxNbPlanes=250, int nbSamplesPerCell=40, double proba=1, bool geom=false, double ratioReconstructed=0.98, bool verbose=false);

std::pair<Matrix, PointRg> computeTransform(const Eigen::MatrixXd &rotPoints);
void sampleBetweenPoints(const std::vector<Kernel2::Point_3>& points, std::vector<std::pair<Point, int>> &query,
                                                       int nbSamples=40, int faceHandle=-1);

#endif //BIM_DATA_GRAPHSTATS_H
