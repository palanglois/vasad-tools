#ifndef BIM_DATA_VOXELSPARSE_H
#define BIM_DATA_VOXELSPARSE_H

#include "iogeometry.h"

class VoxelSparse {
public:
    typedef std::tuple<int, int, int> triplet;
    typedef std::vector<triplet> Coord;
    typedef std::vector<std::vector<double>> SparseVal;
    typedef std::map<triplet, int> CoordIdx;

    // Constructor
    VoxelSparse(const CGAL::Bbox_3 &inBbox, double inVoxelsSide);

    // Finding the cell in which a point is
    [[nodiscard]] triplet findVoxel(const Point &query) const;

    // Assign a normal information to a given point
    void updateNormalFeature(const Point &point, const Vector &normal);

    // Building sparse voxels from scan
    void computeFeaturesFromPointCloud(const std::vector<Point> &pointCloud,
                                       const std::vector<Vector> &normals);

    // Getters
    [[nodiscard]] const Coord& coord() const;
    [[nodiscard]] const SparseVal& values() const;
    [[nodiscard]] SparseVal normalizedValues() const;
    [[nodiscard]] const CoordIdx &coord2idx() const;

private:
    Coord _coord;
    SparseVal _values;
    std::vector<int> nbHits;
    CoordIdx _coord2idx;
    CGAL::Bbox_3 _bbox;
    double _voxelSide;
    int _width;
    int _height;
    int _depth;

};


#endif //BIM_DATA_VOXELSPARSE_H
