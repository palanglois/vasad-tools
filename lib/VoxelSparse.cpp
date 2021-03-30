#include "VoxelSparse.h"

using namespace std;

VoxelSparse::VoxelSparse(const CGAL::Bbox_3 &inBbox, double inVoxelsSide) :
_bbox(inBbox), _voxelSide(inVoxelsSide){

    // Init the sparse voxels
    _coord = Coord(0);
    _values = SparseVal(0);
    _pointCloud = vector<Point>(0);

    // Get the dimensions in nb of voxels
    vector<double> ranges = {_bbox.xmax() - _bbox.xmin(),
                             _bbox.ymax() - _bbox.ymin(),
                             _bbox.zmax() - _bbox.zmin()};
    _width = round(ranges[0] / _voxelSide);
    _height = round(ranges[1] / _voxelSide);
    _depth = round(ranges[2] / _voxelSide);
}

VoxelSparse::triplet VoxelSparse::findVoxel(const Point &query) const
{
    int idx_x = floor((CGAL::to_double(query.x()) - _bbox.xmin())/_voxelSide);
    int idx_y = floor((CGAL::to_double(query.y()) - _bbox.ymin())/_voxelSide);
    int idx_z = floor((CGAL::to_double(query.z()) - _bbox.zmin())/_voxelSide);
    return make_tuple(idx_x, idx_y, idx_z);
}

void VoxelSparse::updateNormalFeature(const Point &point, const Vector &normal)
{
    if(!CGAL::do_overlap(point.bbox(), _bbox)) return;
    _pointCloud.push_back(point);
    triplet coord = findVoxel(point);
    assert(get<0>(coord) >= 0);
    assert(get<0>(coord) < _width);
    assert(get<1>(coord) >= 0);
    assert(get<1>(coord) < _height);
    assert(get<2>(coord) >= 0);
    assert(get<2>(coord) < _depth);

    if(_coord2idx.find(coord) == _coord2idx.end()) {
        // The voxel is empty so far
        _coord2idx[coord] = _coord.size();
        _coord.push_back({get<0>(coord), get<1>(coord), get<2>(coord)});
        _values.push_back({normal.x(), normal.y(), normal.z()});
        nbHits.push_back(1);
    }
    else {
        // The voxel already exists in the representation
        int voxelIdx = _coord2idx.at(coord);
        nbHits[voxelIdx] += 1;
        vector<double>& curFeature =  _values[voxelIdx];
        curFeature[0] += normal.x();
        curFeature[1] += normal.y();
        curFeature[2] += normal.z();
    }
}

const VoxelSparse::Coord &VoxelSparse::coord() const {
    return _coord;
}

const VoxelSparse::SparseVal &VoxelSparse::values() const {
    return _values;
}

VoxelSparse::SparseVal VoxelSparse::normalizedValues() const {
    SparseVal outVal(_values.size(), vector<double>(3, 0.));
    for(int i = 0; i < _values.size(); i++)
    {
        double norm = sqrt(pow(_values[i][0], 2) + pow(_values[i][1], 2) + pow(_values[i][2], 2));
        if(norm != 0.) {
            outVal[i][0] = _values[i][0] / norm;
            outVal[i][1] = _values[i][1] / norm;
            outVal[i][2] = _values[i][2] / norm;
        }
    }
    return outVal;
}

const VoxelSparse::CoordIdx &VoxelSparse::coord2idx() const {
    return _coord2idx;
}

void VoxelSparse::computeFeaturesFromPointCloud(const vector<Point> &pointCloud, const vector<Vector> &normals) {
    for(int i=0; i < pointCloud.size(); i++)
        updateNormalFeature(pointCloud[i], normals[i]);
}

const std::vector<Point> &VoxelSparse::pointCloud() const {
    return _pointCloud;
}
