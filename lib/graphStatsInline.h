#ifndef BIM_DATA_GRAPHSTATSINLINE_H
#define BIM_DATA_GRAPHSTATSINLINE_H

#include "iogeometry.h"


template <typename T, typename A>
int arg_max(std::vector<T, A> const& vec) {
    return static_cast<int>(std::distance(vec.begin(), std::max_element(vec.begin(), vec.end())));
}

struct SetComparison {
  bool operator() (const std::pair<Point, double>& a, const std::pair<Point, double>& b) const {
    return (a.second > b.second);
  };
};

inline double computeFacetOrientation(const Arrangement &arr, int facetHandle) {
    const auto &facetPlane = arr.plane(arr.facet_plane(facetHandle));
    const auto &planeNormal = facetPlane.orthogonal_direction().vector();
    return abs(CGAL::to_double(planeNormal.z())) / sqrt(CGAL::to_double(planeNormal.squared_length()));
}

inline std::vector<Triangle> meshBbox(const CGAL::Bbox_3 &bbox) {
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

inline bool isInShape(const Point &candidate, const CGAL::Bbox_3 &bbox, std::vector <Triangle> &mesh) {
    if (candidate.x() < bbox.xmin()) return false;
    if (candidate.x() > bbox.xmax()) return false;
    if (candidate.y() < bbox.ymin()) return false;
    if (candidate.y() > bbox.ymax()) return false;
    if (candidate.z() < bbox.zmin()) return false;
    if (candidate.z() > bbox.zmax()) return false;
    Tree tree(mesh.begin(), mesh.end());
    //Make random queries ray and intersect it against the current shape
    std::vector <Ray> queries = {Ray(candidate, Vector(1., 0., 0.)),
                                 Ray(candidate, Vector(0., 1., 0.)),
                                 Ray(candidate, Vector(0., 0., 1.))};
    std::vector <std::list<Ray_intersection>> intersections(3, std::list<Ray_intersection>(0));
    for (int k = 0; k < queries.size(); k++)
        tree.all_intersections(queries[k], back_inserter(intersections[k]));
    // We use the parity of the number of intersections to know whether candidate is inside/outside the shape
    unsigned int even = 0;
    for (const auto &inter: intersections)
        even += int(inter.size() % 2 == 0);
    if (even != queries.size()) {
        // The point is inside a shape
        return true;
    }
    return false;
}

template<class OutputIterator>
inline std::vector<int> addSegmentIfInBbox(const std::vector<Point> &pointOfViews, const std::vector<Point> &points,
                               OutputIterator beginPointIt, OutputIterator endPointIt,
                               const CGAL::Bbox_3 &bbox)
{
    std::vector<int> selectedIndex;
    std::vector<Triangle> bboxMesh = meshBbox(bbox);
    Tree bboxTree(bboxMesh.begin(), bboxMesh.end());
    for(int i=0; i < pointOfViews.size(); i++) {
        // We keep only the visibility segments that cross our bounding box
        Segment visSegment(pointOfViews[i], points[i]);
        if (!CGAL::do_overlap(visSegment.bbox(), bbox))
            continue;
        if (!CGAL::do_overlap(pointOfViews[i].bbox(), bbox) && !CGAL::do_overlap(points[i].bbox(), bbox)
            && !bboxTree.do_intersect(visSegment))
            continue;
        *beginPointIt++ = pointOfViews[i];
        *endPointIt++ = pointOfViews[i] + 1. * (points[i] - pointOfViews[i]);
        selectedIndex.push_back(i);
    }
    return selectedIndex;
}

inline std::string padTo(std::string str, const size_t num, const char paddingChar = '0')
{
    if(num > str.size())
        str.insert(0, num - str.size(), paddingChar);
    return str;
}



inline void subdivideBboxLongestAxis(std::queue<CGAL::Bbox_3> &bboxes, CGAL::Bbox_3 curBbox) {
    // Subdivides curBbox in 2 along its longest axis
    int longestDim = -1;
    double dimX = curBbox.xmax() - curBbox.xmin();
    double dimY = curBbox.ymax() - curBbox.ymin();
    double dimZ = curBbox.zmax() - curBbox.zmin();
    if (dimX >= dimY && dimX >= dimZ) {
        bboxes.emplace(curBbox.xmin(), curBbox.ymin(), curBbox.zmin(),
                       curBbox.xmin() + dimX / 2., curBbox.ymax(), curBbox.zmax());
        bboxes.emplace(curBbox.xmin() + dimX / 2., curBbox.ymin(), curBbox.zmin(),
                       curBbox.xmax(), curBbox.ymax(), curBbox.zmax());
    } else if (dimY >= dimZ && dimY >= dimX) {
        bboxes.emplace(curBbox.xmin(), curBbox.ymin(), curBbox.zmin(),
                       curBbox.xmax(), curBbox.ymin() + dimY / 2., curBbox.zmax());
        bboxes.emplace(curBbox.xmin(), curBbox.ymin() + dimY / 2., curBbox.zmin(),
                       curBbox.xmax(), curBbox.ymax(), curBbox.zmax());
    } else {
        bboxes.emplace(curBbox.xmin(), curBbox.ymin(), curBbox.zmin(),
                       curBbox.xmax(), curBbox.ymax(), curBbox.zmin() + dimZ / 2.);
        bboxes.emplace(curBbox.xmin(), curBbox.ymin(), curBbox.zmin() + dimZ / 2.,
                       curBbox.xmax(), curBbox.ymax(), curBbox.zmax());
    }
}

inline bool isInBbox(const Triangle& tri, const CGAL::Bbox_3 &bbox)
{
    if(tri.bbox().xmin() < bbox.xmin()) return false;
    if(tri.bbox().ymin() < bbox.ymin()) return false;
    if(tri.bbox().zmin() < bbox.zmin()) return false;
    if(tri.bbox().xmax() > bbox.xmax()) return false;
    if(tri.bbox().ymax() > bbox.ymax()) return false;
    return !(tri.bbox().zmax() > bbox.zmax());
}

inline std::vector<int> computePlanesInBoundingBox(const std::vector<Plane> &planes, const std::vector<Point> &points,
                                              CGAL::Bbox_3 bbox, double ratioReconstructed)
{
    std::vector<int> planeIdx;
    std::vector<Triangle> bboxTriangles = meshBbox(bbox);
    Tree tree(bboxTriangles.begin(), bboxTriangles.end());
    for(int i=0; i < planes.size(); i++)
    {
        if(planes[i].cumulatedPercentage > ratioReconstructed) continue;
        bool doesIntersect = false;
        for(int j=0; j < planes[i].faces.size() && !doesIntersect; j++) {
            Triangle query(points[planes[i].faces[j][0]], points[planes[i].faces[j][1]],
                           points[planes[i].faces[j][2]]);
            doesIntersect = isInBbox(query, bbox) || tree.do_intersect(query);
        }
        if(doesIntersect)
            planeIdx.push_back(i);
    }
    return planeIdx;
}

#endif //BIM_DATA_GRAPHSTATSINLINE_H
