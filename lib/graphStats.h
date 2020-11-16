#ifndef BIM_DATA_GRAPHSTATS_H
#define BIM_DATA_GRAPHSTATS_H

#include <stack>

#include <Eigen/Eigenvalues>

#include "iogeometry.h"
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

EdgeFeatures computeFeaturesFromLabeledPoints(PlaneArrangement &planeArr, const std::vector<Point> &points,
                                              const std::vector<int> &labels, const int nbClasses,
                                              int nbSamplesPerCell, bool verbose=false);

std::vector<int> assignLabel(PlaneArrangement& planeArr,
        std::vector<facesLabelName> &labeledTrees, int nbClasses, int nbSamplesPerCell=40, bool verbose=false);

std::vector<int> computePlanesInBoundingBox(const std::vector<Plane> &planes, const std::vector<Point> &points,
                                            CGAL::Bbox_3 bbox, double ratioReconstructed=0.98);

std::vector<nlohmann::json> splitArrangementInBatch(const PlaneArrangement &planeArr,
        std::vector<facesLabelName> &labeledShapes, int nbClasses, double step, int maxNodes,
        const std::pair<std::vector<Point>, std::vector<int>> &labeledPointCloud,
        int maxNbPlanes=250, int nbSamplesPerCell=40, double proba=1, bool geom=false, double ratioReconstructed=0.98, bool verbose=false);

std::pair<Matrix, PointRg> computeTransform(const Eigen::MatrixXd &rotPoints);
void sampleBetweenPoints(const std::vector<Kernel2::Point_3>& points, std::vector<std::pair<Point, int>> &query,
                                                       int nbSamples=40, int faceHandle=-1);



inline double computeFacetOrientation(const Arrangement &arr, int facetHandle)
{
    const auto& facetPlane = arr.plane(arr.facet_plane(facetHandle));
    const auto& planeNormal = facetPlane.orthogonal_direction().vector();
    return abs(CGAL::to_double(planeNormal.z())) / sqrt(CGAL::to_double(planeNormal.squared_length()));
}

void refinePoint(Point &point, std::vector<Triangle> &mesh, int nbShoot=100);
std::vector<Point> findPtViewInBbox(const CGAL::Bbox_3 &bbox, std::vector<facesLabelName> &shapesAndClasses,
                                    std::vector<Triangle> &mesh, int nbShoot, int nbCandidates=100, bool verbose=false);

inline std::vector<Triangle> meshBbox(const CGAL::Bbox_3 &bbox)
{
    std::vector<Point> bboxPoints = {
            Point(bbox.xmin(), bbox.ymin(), bbox.zmin()),
            Point(bbox.xmin(), bbox.ymin(), bbox.zmax()),
            Point(bbox.xmin(), bbox.ymax(), bbox.zmin()),
            Point(bbox.xmin(), bbox.ymax(), bbox.zmax()),
            Point(bbox.xmax(), bbox.ymin(), bbox.zmin()),
            Point(bbox.xmax(), bbox.ymin(), bbox.zmax()),
            Point(bbox.xmax(), bbox.ymax(), bbox.zmin()),
            Point(bbox.xmax(), bbox.ymax(), bbox.zmax()),
    };
    std::vector<Triangle> bboxTriangles = {
            Triangle(bboxPoints[0], bboxPoints[1], bboxPoints[5]),
            Triangle(bboxPoints[0], bboxPoints[5], bboxPoints[4]),
            Triangle(bboxPoints[4], bboxPoints[5], bboxPoints[6]),
            Triangle(bboxPoints[6], bboxPoints[5], bboxPoints[7]),
            Triangle(bboxPoints[1], bboxPoints[3], bboxPoints[5]),
            Triangle(bboxPoints[5], bboxPoints[3], bboxPoints[7]),
            Triangle(bboxPoints[0], bboxPoints[4], bboxPoints[2]),
            Triangle(bboxPoints[2], bboxPoints[4], bboxPoints[6]),
            Triangle(bboxPoints[0], bboxPoints[2], bboxPoints[1]),
            Triangle(bboxPoints[1], bboxPoints[2], bboxPoints[3]),
            Triangle(bboxPoints[2], bboxPoints[7], bboxPoints[3]),
            Triangle(bboxPoints[2], bboxPoints[6], bboxPoints[7]),
    };
    return bboxTriangles;
}

#endif //BIM_DATA_GRAPHSTATS_H
