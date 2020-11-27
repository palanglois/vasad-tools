#ifndef BIM_DATA_GRAPHSTATS_H
#define BIM_DATA_GRAPHSTATS_H

#include <stack>

#include <Eigen/Eigenvalues>

#include "graphStatsInline.h"
#include "RegionGrowing.h"

std::pair<Nodes, Edges> computeGraphStatistics(const std::vector<bool> &labels,
        const std::map<int, int> &cell2label, const Arrangement &arr, bool verbose=false);

NodeFeatures computeNodeFeatures(PlaneArrangement& planeArr, std::vector<int> &labels, const double proba,
                                 const bool withGeom, bool verbose=false);

EdgeFeatures computeTrivialEdgeFeatures(PlaneArrangement& planeArr, std::vector<int> &labels, const int nbClasses,
                                        bool verbose=false);

inline double computeFacetArea(const Arrangement &arr, int facetHandle);

std::pair<std::vector<Point>, std::map<Point, int>> sampleFacets(const Arrangement &arr,
                                                                 std::map<int, double> &facetAreas);

void computeVisibility(PlaneArrangement &planeArr, const std::vector<Point> &points,
                       const std::vector<Point> &pointOfViews, EdgeFeatures &edgeFeatures, int nbClasses,
                       const std::vector<Arrangement::Face_handle> &exactPovCells = std::vector<Arrangement::Face_handle>(0));

EdgeFeatures computeFeaturesFromLabeledPoints(PlaneArrangement &planeArr, const std::vector<Point> &points,
                                              const std::vector<int> &labels, const int nbClasses,
                                              int nbSamplesPerCell,
                                              const std::vector<Point> &pointOfViews = std::vector<Point>(0),
                                              bool verbose = false);

std::vector<int> assignLabel(PlaneArrangement& planeArr,
        std::vector<facesLabelName> &labeledTrees, int nbClasses, int nbSamplesPerCell=40, bool verbose=false);

std::vector<int> computePlanesInBoundingBox(const std::vector<Plane> &planes, const std::vector<Point> &points,
                                            CGAL::Bbox_3 bbox, double ratioReconstructed=0.98);

std::vector<nlohmann::json> splitArrangementInBatch(const PlaneArrangement &planeArr,
        std::vector<facesLabelName> &labeledShapes, int nbClasses, double step, int maxNodes,
        const std::pair<std::vector<Point>, std::vector<int>> &labeledPointCloud,
        const std::vector<Point> &pointOfViews=std::vector<Point>(0),
        int maxNbPlanes=250, int nbSamplesPerCell=40, double proba=1, bool geom=false,
        double ratioReconstructed=0.98, bool verbose=false);

std::pair<Matrix, PointRg> computeTransform(const Eigen::MatrixXd &rotPoints);
void sampleBetweenPoints(const std::vector<Kernel2::Point_3>& points, std::vector<std::pair<Point, int>> &query,
                                                       int nbSamples=40, int faceHandle=-1);

void refinePoint(Point &point, std::vector<Triangle> &mesh, int nbShoot=100);
std::vector<Point> findPtViewInBbox(const CGAL::Bbox_3 &bbox, std::vector<facesLabelName> &shapesAndClasses,
                                    std::vector<Triangle> &mesh, int nbShoot, int nbCandidates=100, bool verbose=false);

#endif //BIM_DATA_GRAPHSTATS_H
