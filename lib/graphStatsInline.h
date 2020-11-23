#ifndef BIM_DATA_GRAPHSTATSINLINE_H
#define BIM_DATA_GRAPHSTATSINLINE_H

#include "iogeometry.h"


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

#endif //BIM_DATA_GRAPHSTATSINLINE_H
