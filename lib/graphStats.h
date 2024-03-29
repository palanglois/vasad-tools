#ifndef BIM_DATA_GRAPHSTATS_H
#define BIM_DATA_GRAPHSTATS_H

#include <stack>

#include <Eigen/Eigenvalues>

#include "graphStatsInline.h"
#include "RegionGrowing.h"

std::pair<Nodes, Edges> computeGraphStatistics(const std::vector<bool> &labels,
        const std::map<int, int> &cell2label, const Arrangement &arr, bool verbose=false);

NodeFeatures computeNodeFeatures(PlaneArrangement& planeArr, const std::vector<double> &nodeVisibility,
                                 const double visThreshold=-1,
                                 const std::vector<double> &nodeVolume=std::vector<double>(0),
                                         const bool withGeom=true, bool verbose=false);

EdgeFeatures computeTrivialEdgeFeatures(PlaneArrangement& planeArr, std::vector<int> &labels, const int nbClasses,
                                        bool verbose=false);

inline double computeFacetArea(const Arrangement &arr, int facetHandle);

std::pair<std::vector<Point>, std::map<Point, int>> sampleFacets(const Arrangement &arr,
                                                                 std::map<int, double> &facetAreas);

std::vector<double> computeVisibility(PlaneArrangement &planeArr, const std::vector<Point> &points,
                                      const std::vector<Point> &pointOfViews, EdgeFeatures &edgeFeatures, int nbClasses,
                                      const std::vector<Arrangement::Face_handle> &exactPovCells = std::vector<Arrangement::Face_handle>(0));

EdgeFeatures computeFeaturesFromLabeledPoints(PlaneArrangement &planeArr, const std::vector<Point> &points,
                                              const std::vector<int> &labels, const int nbClasses,
                                              int nbSamplesPerCell,
                                              std::vector<double> &nodeVisibility,
                                              const std::vector<Point> &pointOfViews = std::vector<Point>(0),
                                              bool verbose = false);

std::vector<CGAL::Bbox_3> computeShapeBboxes(std::vector<facesLabelName> &labeledShapes);

template <typename T>
std::vector<int> assignLabelToPointsWithBboxes(const std::vector<T> &queryPoints,
                                               std::vector<facesLabelName> &labeledShapes, int nbClasses,
                                               const CGAL::Bbox_3 &bbox,
                                               const std::vector<CGAL::Bbox_3> &bboxes);

template <typename T>
std::vector<int> assignLabelToPoints(const std::vector<T>& queryPoints,
                                     std::vector<facesLabelName> &labeledShapes, int nbClasses,
                                     const CGAL::Bbox_3 &bbox);

std::vector<int> assignLabel(PlaneArrangement& planeArr,
        std::vector<facesLabelName> &labeledTrees, int nbClasses, int nbSamplesPerCell=40, bool verbose=false);

std::vector<int> computePlanesInBoundingBox(const std::vector<Plane> &planes, const std::vector<Point> &points,
                                            CGAL::Bbox_3 bbox, double ratioReconstructed=0.98);

int splitArrangementInBatch(const PlaneArrangement &planeArr,
        std::vector<facesLabelName> &labeledShapes, const std::string& path, int nbClasses, double step, int maxNodes,
        const std::pair<std::vector<Point>, std::vector<int>> &labeledPointCloud,
        const std::vector<Point> &pointOfViews=std::vector<Point>(0),
        int maxNbPlanes=250, int nbSamplesPerCell=40, double visThreshold=1, bool geom=false,
        double ratioReconstructed=0.98, bool merging=false, bool verbose=false);

std::pair<Matrix, PointRg> computeTransform(const Eigen::MatrixXd &rotPoints);
void sampleBetweenPoints(const std::vector<Kernel2::Point_3>& points, std::vector<std::pair<Point, int>> &query,
                                                       int nbSamples=40, int faceHandle=-1);

void refinePoint(Point &point, std::vector<Triangle> &mesh, int nbShoot=100);
std::vector<Point> findPtViewInBbox(const CGAL::Bbox_3 &bbox, std::vector<facesLabelName> &shapesAndClasses,
                                    std::vector<Triangle> &mesh, int nbShoot, int nbCandidates = 100,
                                    bool verbose = false);

std::pair<std::vector<int>, std::vector<std::vector<int>>>
mergeNodesFromVisibility(PlaneArrangement &planeArr, const std::vector<double> &nodeVisibility,
                         const std::vector<double> &nodeVolumes, double visThreshold);

EdgeFeatures mergeEdgeFeatures(const EdgeFeatures &edgeFeatures, const std::vector<int> &node2Merged);

NodeFeatures mergeNodeFeatures(const NodeFeatures &nodeFeatures, const std::vector<int> &cell2Merged,
                               const std::vector<std::vector<int>>& merged2Cell, PlaneArrangement &planeArr,
                               bool withVolume=false);

std::vector<int> mergeGtLabels(const std::vector<int> &gtLabels, const std::vector<std::vector<int>> &merged2Node);

std::vector<double> mergeNodeVolumes(const std::vector<double> &nodeVolumes,
                                     const std::vector<std::vector<int>> &merged2Node);

#endif //BIM_DATA_GRAPHSTATS_H
