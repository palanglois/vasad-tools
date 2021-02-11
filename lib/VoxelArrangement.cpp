#include "VoxelArrangement.h"

using namespace std;

VoxelArrangement::VoxelArrangement(const std::string &name) : isArrangementComputed(false)
{

}

VoxelArrangement::VoxelArrangement(const CGAL::Bbox_3 &inBbox, double inVoxelSide) : _bbox(inBbox),
_voxelSide(inVoxelSide), isArrangementComputed(false)
{
    computePlanes();
    buildArrangement();
}

void VoxelArrangement::computePlanes()
{
    Simple_to_Epeck s2e;
    // Planes
    vector<double> ranges = {_bbox.xmax() - _bbox.xmin(),
                             _bbox.ymax() - _bbox.ymin(),
                             _bbox.zmax() - _bbox.zmin()};
    double totalArea = 0.;
    for(int i=0; i < ranges.size(); i++)
        totalArea += (ceil(ranges[i] / _voxelSide) + 1) * ranges[(i + 1) % 3] * ranges[(i + 2) % 3];
    vector<double> mins = {_bbox.xmin(), _bbox.ymin(), _bbox.zmin()};
    vector<double> maxs = {_bbox.xmax(), _bbox.ymax(), _bbox.zmax()};
    vector<Vector> normals = {{1., 0., 0.}, {0., 1., 0.}, {0., 0., 1.}};
    vector<Point> pointCloud;
    int faceAccum = 0;
    for(int i=0; i < ranges.size(); i++) {
        int nbPlanes = ceil(ranges[i] / _voxelSide);
        Vector& normal = normals[i];
        for (int j = 0; j < nbPlanes + 1; j++) {
            double curAxisCoordinate = min(mins[i] + _voxelSide * j, maxs[i]);
            vector<double> lowLeft = {0., 0., 0.};
            lowLeft[i] = curAxisCoordinate;
            lowLeft[(i + 1) % 3] = mins[(i + 1) % 3];
            lowLeft[(i + 2) % 3] = mins[(i + 2) % 3];
            pointCloud.emplace_back(lowLeft[0], lowLeft[1], lowLeft[2]);
            vector<double> lowRight = {0., 0., 0.};
            lowRight[i] = curAxisCoordinate;
            lowRight[(i + 1) % 3] = maxs[(i + 1) % 3];
            lowRight[(i + 2) % 3] = mins[(i + 2) % 3];
            pointCloud.emplace_back(lowRight[0], lowRight[1], lowRight[2]);
            vector<double> topLeft = {0., 0., 0.};
            topLeft[i] = curAxisCoordinate;
            topLeft[(i + 1) % 3] = mins[(i + 1) % 3];
            topLeft[(i + 2) % 3] = maxs[(i + 2) % 3];
            pointCloud.emplace_back(topLeft[0], topLeft[1], topLeft[2]);
            vector<double> topRight = {0., 0., 0.};
            topRight[i] = curAxisCoordinate;
            topRight[(i + 1) % 3] = maxs[(i + 1) % 3];
            topRight[(i + 2) % 3] = maxs[(i + 2) % 3];
            pointCloud.emplace_back(topRight[0], topRight[1], topRight[2]);
            double area = (ranges[(i + 1) % 3] * ranges[(i + 2) % 3]);

            // Triangulating the plane
            vector<vector<int>> faces = {{faceAccum, faceAccum + 1, faceAccum + 2},
                                         {faceAccum + 1, faceAccum + 2, faceAccum + 3}};
            faceAccum += 4;
            Plane p = {{lowLeft[0], lowLeft[1], lowLeft[2]}, s2e(normal), faces, area};
            _planes.push_back(p);
        }
    }
}

void VoxelArrangement::buildArrangement()
{
    if(isArrangementComputed) return;
    isArrangementComputed = true;
    _arr.set_bbox(_bbox);
    auto tqPlanes = tq::tqdm(_planes);
    tqPlanes.set_prefix("Inserting " + to_string(_planes.size()) + " planes: ");
    for (const auto &plane: tqPlanes)
        _arr.insert(Kernel2::Plane_3(plane.inlier, plane.normal));

    _width = -1;
    _height = -1;
    _depth = -1;
    for (auto cellIt = _arr.cells_begin(); cellIt != _arr.cells_end(); cellIt++) {
        if (!_arr.is_cell_bounded(*cellIt)) continue;
        const auto &centroid = cellIt->point();
        int idx_x = floor((CGAL::to_double(centroid.x()) - _bbox.xmin()) / _voxelSide);
        int idx_y = floor((CGAL::to_double(centroid.y()) - _bbox.ymin()) / _voxelSide);
        int idx_z = floor((CGAL::to_double(centroid.z()) - _bbox.zmin()) / _voxelSide);
        if(idx_x + 1 > _width) _width = idx_x + 1;
        if(idx_y + 1 > _height) _height = idx_y + 1;
        if(idx_z + 1 > _depth) _depth = idx_z + 1;

        _node2index[_arr.cell_handle(*cellIt)] = make_tuple(idx_x, idx_y, idx_z);
        _index2node[make_tuple(idx_x, idx_y, idx_z)] = _arr.cell_handle(*cellIt);

        vector<int> cellFacets;
        for(auto faceIt = cellIt->subfaces_begin(); faceIt != cellIt->subfaces_end(); faceIt++)
            cellFacets.push_back(*faceIt);
        _node2facets[_arr.cell_handle(*cellIt)] = cellFacets;
    }
}

int VoxelArrangement::closestFacet(const Arrangement::Point &query)
{
    int idx_x = floor((CGAL::to_double(query.x()) - _bbox.xmin())/_voxelSide);
    int idx_y = floor((CGAL::to_double(query.y()) - _bbox.ymin())/_voxelSide);
    int idx_z = floor((CGAL::to_double(query.z()) - _bbox.zmin())/_voxelSide);
    const vector<int> &cellFacets = _node2facets[_index2node[make_tuple(idx_x, idx_y, idx_z)]];

    auto bestDistance = DBL_MAX;
    int finalFacet = -1;
    for(const auto facetHandle: cellFacets)
    {
        double distance = CGAL::to_double((_arr.plane(_arr.facet_plane(facetHandle)).projection(query) - query).squared_length());
        if(distance < bestDistance)
        {
            bestDistance = distance;
            finalFacet = facetHandle;
        }
    }
    return finalFacet;
}

void VoxelArrangement::assignLabel(vector<facesLabelName> &labeledShapes, int nbClasses, bool verbose)
{
    Epeck_to_Simple e2s;
    // Make sure that the arrangement has been built
    buildArrangement();
    // Gather all the points
    vector<pair<Point, int>> points;
    for(auto cellIt = _arr.cells_begin(); cellIt != _arr.cells_end(); cellIt++)
        if(_arr.is_cell_bounded(*cellIt))
            points.emplace_back(e2s(cellIt->point()), _arr.cell_handle(*cellIt));
    // Label the points
    vector<int> labels = assignLabelToPoints(points, labeledShapes, nbClasses, _bbox);
    // Store them
    _labels = vector<vector<vector<int>>>(_width, vector<vector<int>>(_height, vector<int>(_depth, -1)));
    for (int i = 0; i < points.size(); i++) {
        tuple<int, int, int> idx = _node2index[points[i].second];
        _labels[get<0>(idx)][get<1>(idx)][get<2>(idx)] = labels[i];
    }
}

const std::vector<Plane> &VoxelArrangement::planes() const
{
    return _planes;
}

const Arrangement::Plane & VoxelArrangement::planeFromFacetHandle(int handle) const
{
    return _arr.plane(_arr.facet_plane(handle));
}

const VoxelArrangement::LabelTensor & VoxelArrangement::labels() const {
    return _labels;
}