#include "VoxelArrangement.h"

using namespace std;
using Json = nlohmann::json;
using namespace HighFive;

VoxelArrangement::VoxelArrangement(const std::string &name) : isArrangementComputed(false), _width(0), _height(0),
                                                              _depth(0) {
    if(name.substr(name.size() - 4) == "json")
        loadJson(name);
    else if(name.substr(name.size() - 2) == "h5")
        loadHdf(name);
    else
        cerr << "Only .json and .h5 files can be loaded!" << endl;

}

void VoxelArrangement::loadJson(const std::string& name) {
    cout << "Loading voxels from json!" << endl;
    ifstream inStream(name);
    Json data;
    inStream >> data;

    //Features
    _features = data["features"].get<FeatTensor>();

    // Labels
    _labels = data["labels"].get<LabelTensor>();

    // width, height, depth
    if(!_labels.empty())
    {
        _width = _labels.size();
        _height = _labels[0].size();
        _depth = _labels[0][0].size();
    }

    // Map
    _index2node = data["map"].get<map<triplet, int>>();

    // Inverse mapping
    for(const auto &indexAndNode: _index2node)
        _node2index[indexAndNode.second] = indexAndNode.first;

    // Planes
    _planes = data["planes"].get<vector<Plane>>();

    // Bbox
    _bbox = data["bbox"].get<CGAL::Bbox_3>();

    // Voxel side size
    _voxelSide = data["voxelSide"].get<double>();

}

void VoxelArrangement::loadHdf(const std::string &name) {
    H5Easy::File file(name, H5Easy::File::ReadOnly);

    // Features
    _features = H5Easy::load<FeatTensor>(file, "/features");

    // Labels
    _labels = H5Easy::load<LabelTensor>(file, "/labels");

    // width, height, depth
    if(!_labels.empty())
    {
        _width = _labels.size();
        _height = _labels[0].size();
        _depth = _labels[0][0].size();
    }

    // Map
    auto vectorizedMap = H5Easy::load<vector<vector<int>>>(file, "/map");
    for(auto & mapElem : vectorizedMap)
        _index2node[make_tuple(mapElem[0], mapElem[1], mapElem[2])] = mapElem[3];

    // Planes
    auto inliers = H5Easy::load<vector<vector<double>>>(file, "/planes/inliers");
    auto normals = H5Easy::load<vector<vector<double>>>(file, "/planes/normals");
    auto faces = H5Easy::load<vector<vector<vector<int>>>>(file, "/planes/faces");
    auto cumulatedPercentages = H5Easy::load<vector<double>>(file, "/planes/cumulatedPercentages");
    for(int i=0; i < inliers.size(); i++)
    {
        Kernel2::Point_3 inlier(inliers[i][0], inliers[i][1], inliers[i][2]);
        Kernel2::Vector_3 normal(normals[i][0], normals[i][1], normals[i][2]);
        vector<vector<int>> curFaces = faces[i];
        double cumulatedPercentage = cumulatedPercentages[i];
        _planes.push_back({inlier, normal, curFaces, cumulatedPercentage});
    }

    // Bbox
    auto bbox = H5Easy::load<vector<double>>(file, "/bbox");
    _bbox = CGAL::Bbox_3(bbox[0], bbox[1], bbox[2], bbox[3], bbox[4], bbox[5]);

    // Voxel side size
    _voxelSide = H5Easy::load<double>(file, "/voxelSide");

}

VoxelArrangement::VoxelArrangement(const CGAL::Bbox_3 &inBbox, double inVoxelSide) : _bbox(inBbox),
_voxelSide(inVoxelSide), isArrangementComputed(false), _width(0), _height(0), _depth(0)
{
    computePlanes();
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
    int faceAccum = 0;
    for(int i=0; i < ranges.size(); i++) {
        int nbPlanes = round(ranges[i] / _voxelSide);
        Vector& normal = normals[i];
        for (int j = 1; j < nbPlanes; j++) {
            double curAxisCoordinate = min(mins[i] + _voxelSide * j, maxs[i]);
            vector<double> lowLeft = {0., 0., 0.};
            lowLeft[i] = curAxisCoordinate;
            lowLeft[(i + 1) % 3] = mins[(i + 1) % 3];
            lowLeft[(i + 2) % 3] = mins[(i + 2) % 3];
            _pointCloud.emplace_back(lowLeft[0], lowLeft[1], lowLeft[2]);
            vector<double> lowRight = {0., 0., 0.};
            lowRight[i] = curAxisCoordinate;
            lowRight[(i + 1) % 3] = maxs[(i + 1) % 3];
            lowRight[(i + 2) % 3] = mins[(i + 2) % 3];
            _pointCloud.emplace_back(lowRight[0], lowRight[1], lowRight[2]);
            vector<double> topLeft = {0., 0., 0.};
            topLeft[i] = curAxisCoordinate;
            topLeft[(i + 1) % 3] = mins[(i + 1) % 3];
            topLeft[(i + 2) % 3] = maxs[(i + 2) % 3];
            _pointCloud.emplace_back(topLeft[0], topLeft[1], topLeft[2]);
            vector<double> topRight = {0., 0., 0.};
            topRight[i] = curAxisCoordinate;
            topRight[(i + 1) % 3] = maxs[(i + 1) % 3];
            topRight[(i + 2) % 3] = maxs[(i + 2) % 3];
            _pointCloud.emplace_back(topRight[0], topRight[1], topRight[2]);
            double area = (ranges[(i + 1) % 3] * ranges[(i + 2) % 3]);

            // Triangulating the plane
            vector<vector<int>> faces = {{faceAccum, faceAccum + 1, faceAccum + 2},
                                         {faceAccum + 1, faceAccum + 2, faceAccum + 3}};
            faceAccum += 4;
            Plane p = {{lowLeft[0], lowLeft[1], lowLeft[2]}, s2e(normal), faces, area};
            _planes.push_back(p);
        }
    }
    // Sort the planes from the biggest to the smallest
    sort(_planes.begin(), _planes.end(),
         [](const Plane & a, const Plane & b) -> bool
         {
             return a.cumulatedPercentage > b.cumulatedPercentage;
         });

    // Compute cumulated percentage of reconstructed surface
    double areaCumulator = 0.;
    for(auto &plane: _planes) {
        areaCumulator += plane.cumulatedPercentage;
        plane.cumulatedPercentage = areaCumulator / totalArea;
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
        int idx_x = max(0, (int) floor((CGAL::to_double(centroid.x()) - _bbox.xmin()) / _voxelSide));
        int idx_y = max(0, (int) floor((CGAL::to_double(centroid.y()) - _bbox.ymin()) / _voxelSide));
        int idx_z = max(0, (int) floor((CGAL::to_double(centroid.z()) - _bbox.zmin()) / _voxelSide));
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

void VoxelArrangement::normalizeFeatures() {
    for(int i=0; i < _width; i++)
        for(int j=0; j < _height; j++)
            for(int k=0; k < _depth; k++) {
                double nbElems = 0.;
                for (int l = 0; l < _features[i][j][k].size(); l++)
                    nbElems += _features[i][j][k][l];
                if(nbElems == 0.) continue;
                for (int l = 0; l < _features[i][j][k].size(); l++)
                    _features[i][j][k][l] /= nbElems;
            }
}

bool VoxelArrangement::isLabelEmpty() const {
    bool empty = true;
    for(int i=0; i < _width; i++) {
        for (int j = 0; j < _height; j++) {
            for (int k = 0; k < _depth; k++) {
                if (_labels[i][j][k] != -1) {
                    empty = false;
                    break;
                }
            }
            if (!empty) break;
        }
        if(!empty) break;
    }
    return empty;
}

int VoxelArrangement::numberOfCells()
{
    // Make sure that the arrangement has been built
    buildArrangement();
    // Compute number of cells
    int nbCells = 0;
    for(auto cellIt = _arr.cells_begin(); cellIt != _arr.cells_end(); cellIt++)
        if(_arr.is_cell_bounded(*cellIt))
            nbCells++;
    return nbCells;
}

VoxelArrangement::triplet VoxelArrangement::findVoxel(const Point &query) {
    int idx_x = floor((CGAL::to_double(query.x()) - _bbox.xmin())/_voxelSide);
    int idx_y = floor((CGAL::to_double(query.y()) - _bbox.ymin())/_voxelSide);
    int idx_z = floor((CGAL::to_double(query.z()) - _bbox.zmin())/_voxelSide);
    return make_tuple(idx_x, idx_y, idx_z);

}

int VoxelArrangement::closestFacet(const Point &query)
{
    Simple_to_Epeck s2e;
    const vector<int> &cellFacets = _node2facets[_index2node[findVoxel(query)]];

    auto bestDistance = DBL_MAX;
    int finalFacet = -1;
    for(const auto facetHandle: cellFacets)
    {
        double distance = CGAL::to_double((_arr.plane(_arr.facet_plane(facetHandle)).projection(s2e(query)) - s2e(query)).squared_length());
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
        _labels[get<0>(idx)][get<1>(idx)][get<2>(idx)] = labels[i] == nbClasses ? -1: labels[i];
    }
}

void VoxelArrangement::computeFeaturesRegular(const std::vector<Point> &points, const std::vector<Point> &pointOfViews,
                                              const std::vector<int> &labels, int nbClasses, bool verbose) {
    Simple_to_Epeck s2e;
    Epeck_to_Simple e2s;
    // Make sure that the arrangement has been built
    buildArrangement();
    // Initialize the features
    _features = vector<vector<vector<vector<double>>>>(_width,
                  vector<vector<vector<double>>>(_height,
                          vector<vector<double>>(_depth,
                                  vector<double>(nbClasses + 1, 0.))));

    // Histograms
    for(int i=0; i < points.size(); i++) {
        const Point &point = points[i];
        const Point &pov = pointOfViews[i];
        const int &label = labels[i];

        // Use point only if it is in the current bounding box
        if (!CGAL::do_overlap(_bbox, point.bbox())) continue;

        triplet cellIdx = findVoxel(point);
        _features[get<0>(cellIdx)][get<1>(cellIdx)][get<2>(cellIdx)][label]++;
    }


    // Select the visibility segments that lie in the bounding box
    vector<Point> beginPoints;
    vector<Point> endPoints;
    vector<int> validIdx = addSegmentIfInBbox(pointOfViews, points, back_inserter(beginPoints), back_inserter(endPoints), _bbox);

    // Visibility
    auto tqPoints = tq::trange(validIdx.size());
    tqPoints.set_prefix("Computing visibility: ");
    for (int i : tqPoints) {
        const Point &point = endPoints[i];
        const Point &pov = beginPoints[i];
        const int &label = labels[validIdx[i]];

        bool isPointInBbox = CGAL::do_overlap(point.bbox(), _bbox);
        int frontCellIdx = _arr.cell_handle(*_arr.cells_begin());
        if(CGAL::do_overlap(point.bbox(), _bbox))
            frontCellIdx = _index2node[findVoxel(point)];
        Kernel2::Point_3 originPoint = s2e(point);

        // Intersect the (point_of_view <-> target facet point) segment with the plane arrangement
        Arrangement::Face_handle begin_cell;
        Arrangement::Face_handle end_cell;
        vector<pair<Arrangement::Face_handle, int>> intersectedFacets;
        vector<pair<Arrangement::Face_handle, double>> intersectedCellsAndDists;
        segment_search_advanced(_arr, originPoint, s2e(pov), begin_cell, back_inserter(intersectedFacets),
                                back_inserter(intersectedCellsAndDists), end_cell, frontCellIdx, false);

        // Add visibility information
        for(const auto& cellAndDist: intersectedCellsAndDists) {
            if(cellAndDist.first == begin_cell) continue; // Discard the cell that contains the current point
            if (!_arr.is_cell_bounded(cellAndDist.first)) continue;
            triplet cellIdx = _node2index[cellAndDist.first];
            _features[get<0>(cellIdx)][get<1>(cellIdx)][get<2>(cellIdx)][nbClasses]++;
        }
    }
    normalizeFeatures();
}

void VoxelArrangement::computeFeatures(const vector<Point> &points, const vector<Point> &pointOfViews,
                                       const vector<int> &labels, int nbClasses, bool verbose) {
    Simple_to_Epeck s2e;
    Epeck_to_Simple e2s;
    // Make sure that the arrangement has been built
    buildArrangement();
    // Initialize the features
    _features = vector<vector<vector<vector<double>>>>(_width,
                  vector<vector<vector<double>>>(_height,
                          vector<vector<double>>(_depth,
                                  vector<double>(nbClasses + 1, 0.))));

    // Histograms
    for(int i=0; i < points.size(); i++)
    {
        const Point &point = points[i];
        const Point &pov = pointOfViews[i];
        const int &label = labels[i];

        // Use point only if it is in the current bounding box
        if(!CGAL::do_overlap(_bbox, point.bbox())) continue;

        // Retrieve the closest facet to the current point
        int facetHandle = closestFacet(point);

        // Retrieve the index of the opposite cell of the point of view w.r.t the facet
        Kernel2::Point_3 facetPoint = _arr.facet(facetHandle).point();
        int cellIdx0 = _arr.facet(facetHandle).superface(0);
        Kernel2::Point_3 cellPoint0 = _arr.cell(cellIdx0).point();
        int cellIdx1 = _arr.facet(facetHandle).superface(1);
        Vector visibilityVector(pov, point);
        Vector vectorCell0(e2s(facetPoint), e2s(cellPoint0));
        double scalarProduct = CGAL::scalar_product(visibilityVector, vectorCell0);
        int validCellIdx = (scalarProduct >= 0) ? cellIdx0 : cellIdx1;
        triplet coordinates = _node2index[validCellIdx];

        // Update the label distribution
        _features[get<0>(coordinates)][get<1>(coordinates)][get<2>(coordinates)][label]++;

    }

    // Select the visibility segments that lie in the bounding box
    vector<Point> beginPoints;
    vector<Point> endPoints;
    vector<int> validIdx = addSegmentIfInBbox(pointOfViews, points, back_inserter(beginPoints), back_inserter(endPoints), _bbox);

    // Visibility
    auto tqPoints = tq::trange(validIdx.size());
    tqPoints.set_prefix("Computing visibility: ");
    for (int i : tqPoints)
    {
        const Point &point = endPoints[i];
        const Point &pov = beginPoints[i];
        const int &label = labels[validIdx[i]];

        bool isPointInBbox = CGAL::do_overlap(point.bbox(), _bbox);
        int frontCellIdx = _arr.cell_handle(*_arr.cells_begin());
        Kernel2::Point_3 originPoint = s2e(point);
        if(isPointInBbox) {
            // Retrieve the closest facet to the current point
            int facetHandle = closestFacet(point);

            // If point is in the bounding box, we use the closest facet's center as the origin of the
            // visibility ray (to avoid boundary effects).
            Kernel2::Point_3 facetPoint = _arr.facet(facetHandle).point();

            // Retrieve the index of the opposite cell of the point of view w.r.t the facet
            int cellIdx0 = _arr.facet(facetHandle).superface(0);
            Kernel2::Point_3 cellPoint0 = _arr.cell(cellIdx0).point();
            int cellIdx1 = _arr.facet(facetHandle).superface(1);
            Vector visibilityVector(pov, point);
            Vector vectorCell0(e2s(facetPoint), e2s(cellPoint0));
            double scalarProduct = CGAL::scalar_product(visibilityVector, vectorCell0);

            // Retrieving the cell just before the current point and its center
            frontCellIdx = (scalarProduct >= 0) ? cellIdx1 : cellIdx0;
            originPoint = _arr.cell(frontCellIdx).point();
        }

        // Intersect the (point_of_view <-> target facet point) segment with the plane arrangement
        Arrangement::Face_handle begin_cell;
        Arrangement::Face_handle end_cell;
        vector<pair<Arrangement::Face_handle, int>> intersectedFacets;
        vector<pair<Arrangement::Face_handle, double>> intersectedCellsAndDists;
        segment_search_advanced(_arr, originPoint, s2e(pov), begin_cell, back_inserter(intersectedFacets),
                                back_inserter(intersectedCellsAndDists), end_cell, frontCellIdx, false);

        // Add visibility information
        for(const auto& cellAndDist: intersectedCellsAndDists){
            if(!_arr.is_cell_bounded(cellAndDist.first)) continue;
            triplet cellIdx = _node2index[cellAndDist.first];
            _features[get<0>(cellIdx)][get<1>(cellIdx)][get<2>(cellIdx)][nbClasses]++;
        }
    }
    normalizeFeatures();
}

void VoxelArrangement::saveAsJson(const string &path)
{
    //Features
    Json features = _features;

    // Labels
    Json labels = _labels;

    // Map
    Json map = _index2node;

    // Planes
    Json planes = _planes;

    // NbPlanes
    Json nbPlanes = _planes.size();

    // Bbox
    Json bbox = _bbox;

    // Voxel side size
    Json voxelSide = _voxelSide;

    // Compiling the output data
    Json outputData = {{"features", features}, {"labels", labels}, {"map", map}, {"planes", planes},
                       {"nbPlanes", nbPlanes}, {"bbox", bbox}, {"voxelSide", voxelSide}};

    // Output the json file
    ofstream outFile(path);
    outFile << outputData;
}

void VoxelArrangement::saveAsHdf(const std::string &path) {
    H5Easy::File file(path, H5Easy::File::Overwrite);
    auto options = H5Easy::DumpOptions(H5Easy::Compression(), H5Easy::DumpMode::Overwrite);

    //Features
    H5Easy::dump(file, "/features", _features, options);

    // Labels
    H5Easy::dump(file, "/labels", _labels, options);

    // Map
    vector<vector<int>> vectorizedMap(_index2node.size(), vector<int>(4, 0));
    int mapIt=0;
    for(const auto& mapElem: _index2node) {
        vectorizedMap[mapIt] = {get<0>(mapElem.first), get<1>(mapElem.first), get<2>(mapElem.first), mapElem.second};
        mapIt++;
    }
    H5Easy::dump(file, "/map", vectorizedMap, options);

    // Planes
    Epeck_to_Simple e2s;
    vector<vector<double>> inliers(_planes.size(), vector<double>(3, 0.));
    vector<vector<double>> normals(_planes.size(), vector<double>(3, 0.));
    vector<vector<vector<int>>> faces;
    vector<double> cumulatedPercentages;
    int planeIt=0;
    for(const auto& pl: _planes) {
        Point inlier = e2s(pl.inlier);
        Vector normal = e2s(pl.normal);
        inliers[planeIt] = {inlier.x(), inlier.y(), inlier.z()};
        normals[planeIt] = {normal.x(), normal.y(), normal.z()};
        faces.push_back(pl.faces);
        cumulatedPercentages.push_back(pl.cumulatedPercentage);
        planeIt++;
    }
    H5Easy::dump(file, "/planes/inliers", inliers, options);
    H5Easy::dump(file, "/planes/normals", normals, options);
    H5Easy::dump(file, "/planes/faces", faces, options);
    H5Easy::dump(file, "/planes/cumulatedPercentages", cumulatedPercentages, options);

    // NbPlanes
    H5Easy::dump(file, "/nbPlanes", _planes.size(), options);

    // Bbox
    vector<double> bbox = {_bbox.xmin(), _bbox.ymin(), _bbox.zmin(), _bbox.xmax(), _bbox.ymax(), _bbox.zmax()};
    H5Easy::dump(file, "/bbox", bbox, options);

    // Voxel side size
    H5Easy::dump(file, "/voxelSide", _voxelSide, options);
}

void VoxelArrangement::saveAsPly(const string &path, const vector<classKeywordsColor> &classesWithColor) {
    // Make sure that the arrangement has been built
    buildArrangement();
    // Making colormap
    std::map<int, Colormap::Color> map;
    for(int i = 0; i < classesWithColor.size(); i++)
    {
        const auto& classColor = get<2>(classesWithColor[i]);
        map[i] = Colormap::Color(static_cast<unsigned char>(get<0>(classColor)),
                                 static_cast<unsigned char>(get<1>(classColor)),
                                 static_cast<unsigned char>(get<2>(classColor)));
    }
    Colormap colormap(map);

    // Label each facet which needs to be drawn
    for(auto itf = _arr.facets_begin(); itf != _arr.facets_end(); itf++){
        Arrangement::Face& f = *itf;
        f._info = -1;
        itf->to_draw = false;
        if(! _arr.is_facet_bounded(f)){continue;}
        Arrangement::Face_handle ch0 = f.superface(0), ch1 = f.superface(1);
        if(!_arr.is_cell_bounded(ch0) || !_arr.is_cell_bounded(ch1)) continue;
        triplet idxCh0 = _node2index[ch0];
        triplet idxCh1 = _node2index[ch1];
        int label1 = _labels[get<0>(idxCh0)][get<1>(idxCh0)][get<2>(idxCh0)];
        int label2 = _labels[get<0>(idxCh1)][get<1>(idxCh1)][get<2>(idxCh1)];
        if(label1 != label2){
            f.to_draw = true;
        }
        if(label1 == -1 && label2 != -1)
            f._info = label2;
        if(label1 != -1 && label2 == -1)
            f._info = label1;
    }

    // Standard polyhedral complex output procedure
    typedef Polyhedral_complex_3::Mesh_3<> Mesh;
    typedef Polyhedral_complex_3::Mesh_extractor_3<Arrangement,Mesh> Extractor;
    Mesh meshGC;
    Extractor extractorGC(_arr);
    extractorGC.extract(meshGC,false);
    {
        std::ofstream stream(path.c_str());
        if (!stream.is_open())
            return ;
        Polyhedral_complex_3::print_mesh_with_facet_color_PLY(stream, meshGC, colormap);
        stream.close();
    }
}

void VoxelArrangement::saveFeaturesAsPly(const string &path, const vector<classKeywordsColor> &classesWithColor)
{
    // Make sure that the arrangement has been built
    buildArrangement();
    // Making colormap
    std::map<int, Colormap::Color> map;
    for(int i = 0; i < classesWithColor.size(); i++)
    {
        const auto& classColor = get<2>(classesWithColor[i]);
        map[i] = Colormap::Color(static_cast<unsigned char>(get<0>(classColor)),
                                 static_cast<unsigned char>(get<1>(classColor)),
                                 static_cast<unsigned char>(get<2>(classColor)));
    }
    Colormap colormap(map);

    int voidClass = classesWithColor.size();

    // Label each facet which needs to be drawn
    for(auto itf = _arr.facets_begin(); itf != _arr.facets_end(); itf++){
        Arrangement::Face& f = *itf;
        f._info = -1;
        itf->to_draw = false;
        if(! _arr.is_facet_bounded(f)){continue;}
        Arrangement::Face_handle ch0 = f.superface(0), ch1 = f.superface(1);
        if(!_arr.is_cell_bounded(ch0) || !_arr.is_cell_bounded(ch1)) continue;
        triplet idxCh0 = _node2index[ch0];
        triplet idxCh1 = _node2index[ch1];
        vector<double> feature1 = _features[get<0>(idxCh0)][get<1>(idxCh0)][get<2>(idxCh0)];
        vector<double> feature2 = _features[get<0>(idxCh1)][get<1>(idxCh1)][get<2>(idxCh1)];
        int label1 = arg_max(feature1) == voidClass ? -1 : arg_max(feature1);
        int label2 = arg_max(feature2) == voidClass ? -1 : arg_max(feature2);
        if(*max_element(feature1.begin(), feature1.end()) == 0) label1 = -1;
        if(*max_element(feature2.begin(), feature2.end()) == 0) label2 = -1;
        if(label1 != label2){
            f.to_draw = true;
        }
        if(label1 == -1 && label2 != -1)
            f._info = label2;
        if(label1 != -1 && label2 == -1)
            f._info = label1;
    }

    // Standard polyhedral complex output procedure
    typedef Polyhedral_complex_3::Mesh_3<> Mesh;
    typedef Polyhedral_complex_3::Mesh_extractor_3<Arrangement,Mesh> Extractor;
    Mesh meshGC;
    Extractor extractorGC(_arr);
    extractorGC.extract(meshGC,false);
    {
        std::ofstream stream(path.c_str());
        if (!stream.is_open())
            return ;
        Polyhedral_complex_3::print_mesh_with_facet_color_PLY(stream, meshGC, colormap);
        stream.close();
    }
}

void VoxelArrangement::saveArrangementAsPly(const string &path)
{
    // Make sure that the arrangement has been built
    buildArrangement();

    // Label each facet which needs to be drawn
    for(auto itf = _arr.facets_begin(); itf != _arr.facets_end(); itf++){
        Arrangement::Face& f = *itf;
        f._info = -1;
        itf->to_draw = false;
        if(! _arr.is_facet_bounded(f)){continue;}
        Arrangement::Face_handle ch0 = f.superface(0), ch1 = f.superface(1);
        if(!_arr.is_cell_bounded(ch0) || !_arr.is_cell_bounded(ch1)) continue;
        itf->to_draw = true;
    }

    // Standard polyhedral complex output procedure
    typedef Polyhedral_complex_3::Mesh_3<> Mesh;
    typedef Polyhedral_complex_3::Mesh_extractor_3<Arrangement,Mesh> Extractor;
    Mesh meshGC;
    Extractor extractorGC(_arr);
    extractorGC.extract(meshGC,false);
    {
        std::ofstream stream(path.c_str());
        if (!stream.is_open())
            return ;
        Polyhedral_complex_3::print_mesh_PLY(stream, meshGC);
        stream.close();
    }
}

const std::vector<Plane> &VoxelArrangement::planes() const
{
    return _planes;
}

const std::vector<Point> &VoxelArrangement::pointCloud() const
{
    return _pointCloud;
}

const Arrangement::Plane & VoxelArrangement::planeFromFacetHandle(int handle)
{
    // Make sure that the arrangement has been built
    buildArrangement();
    return _arr.plane(_arr.facet_plane(handle));
}

const VoxelArrangement::LabelTensor & VoxelArrangement::labels() const {
    return _labels;
}

const VoxelArrangement::FeatTensor & VoxelArrangement::features() const {
    return _features;
}

double VoxelArrangement::width() const {
    return _width;
}

double VoxelArrangement::height() const {
    return _height;
}

double VoxelArrangement::depth() const {
    return _depth;
}

CGAL::Bbox_3 VoxelArrangement::bbox() const {
    return _bbox;
}

vector<CGAL::Bbox_3> splitBigBbox(const CGAL::Bbox_3 &bigBbox, int nbVoxelsAlongAxis, double voxelSide)
{
    vector<CGAL::Bbox_3> bboxes;
    double bboxSide = nbVoxelsAlongAxis*voxelSide;
    int nbSplitX = ceil(double(bigBbox.xmax() - bigBbox.xmin()) / bboxSide);
    int nbSplitY = ceil(double(bigBbox.ymax() - bigBbox.ymin()) / bboxSide);
    int nbSplitZ = ceil(double(bigBbox.zmax() - bigBbox.zmin()) / bboxSide);
    double originX = (bigBbox.xmin() + bigBbox.xmax() - nbSplitX * bboxSide) / 2.;
    double originY = (bigBbox.ymin() + bigBbox.ymax() - nbSplitY * bboxSide) / 2.;
    double originZ = (bigBbox.zmin() + bigBbox.zmax() - nbSplitZ * bboxSide) / 2.;
    for(int i=0; i < nbSplitX; i++)
    {
        for(int j=0; j < nbSplitY; j++)
        {
            for(int k=0; k < nbSplitZ; k++)
            {
                bboxes.emplace_back(originX + i * bboxSide,
                                    originY + j * bboxSide,
                                    originZ + k * bboxSide,
                                    originX + (i + 1) * bboxSide,
                                    originY + (j + 1) * bboxSide,
                                    originZ + (k + 1) * bboxSide);
            }
        }
    }
    return bboxes;
}

int splitArrangementInVoxels(vector<facesLabelName> &labeledShapes,
                             const vector<Point> &pointOfViews,
                             const vector<Point> &pointCloud,
                             const vector<int> &pointCloudLabels,
                             double voxelSide,
                             int nbClasses, const string &path, int maxNodes, bool verbose)
{
    // Compute initial bounding box
    CGAL::Bbox_3 initialBbox;
    for(const auto& shape: labeledShapes)
        for(const auto& triangle: get<0>(shape))
            initialBbox += triangle.bbox();
    queue<CGAL::Bbox_3> bboxes;
    bboxes.push(initialBbox);

    // Gather all the planes
    VoxelArrangement globalVoxels(initialBbox, voxelSide);
    const vector<Plane> &planes = globalVoxels.planes();
    const vector<Point> &pointCloudPlanes = globalVoxels.pointCloud();

    // Max number of plane gets an initial value and will be adjusted during computation
    int maxNbPlanes = 125;

    // We iteratively subdivide the bboxes until they are at acceptable scale
    int chunkIterator(0);
    while(!bboxes.empty()) {
        if (verbose)
            cout << endl << "Bbox queue size: \033[1;31m" << bboxes.size() << "\033[0m" << endl;
        CGAL::Bbox_3 curBbox = bboxes.front();
        bboxes.pop();
        // Compute planes in current bbox
        // We build the arrangement corresponding to the current bounding box
        auto fullArrangement = VoxelArrangement(curBbox, voxelSide);
        int nbPlanesInCurrentBbox = fullArrangement.planes().size();

        // If we have too many planes, we subdivide the current bounding box
        if(nbPlanesInCurrentBbox > maxNbPlanes)
        {
            subdivideBboxLongestAxis(bboxes, curBbox);
            continue;
        }
        // If we don't have any plane, we just drop the current bounding box
        if(nbPlanesInCurrentBbox == 0)
            continue;
        if(verbose)
            cout << "Found " << nbPlanesInCurrentBbox << " valid planes in current bbox which is: " << curBbox << endl;

        // Compute the number of cells in the current arrangement
        int nbCells = fullArrangement.numberOfCells();

        // We need at least 2 nodes to have a valid chunk
        if(nbCells < 2) continue;

        // If we have too many cells, we update maxNbPlanes accordingly and we subdivide the current bounding box
        if(nbCells > maxNodes)
        {
            maxNbPlanes = nbPlanesInCurrentBbox - 1;
            subdivideBboxLongestAxis(bboxes, curBbox);
            continue;
        }

        // We compute the labels
        fullArrangement.assignLabel(labeledShapes, nbClasses, verbose);

        // We compute the features
        fullArrangement.computeFeaturesRegular(pointCloud, pointOfViews, pointCloudLabels, nbClasses, verbose);

        // We save the current chunk
        string outPath(path + padTo(to_string(chunkIterator), 4) + ".json");
        fullArrangement.saveAsJson(outPath);
        chunkIterator++;
    }
    return chunkIterator;
}

int splitArrangementInVoxelsRegular(vector<facesLabelName> &labeledShapes,
                                    const vector<Point> &pointOfViews,
                                    const vector<Point> &pointCloud,
                                    const vector<int> &pointCloudLabels,
                                    double voxelSide,
                                    int nbClasses, const string &path, int nbVoxelsAlongAxis, bool verbose)
{

    // Compute initial bounding box
    CGAL::Bbox_3 initialBbox;
    for(const auto& shape: labeledShapes)
        for(const auto& triangle: get<0>(shape))
            initialBbox += triangle.bbox();

    // Split it
    vector<CGAL::Bbox_3> bboxes = splitBigBbox(initialBbox, nbVoxelsAlongAxis, voxelSide);

    // Generate the chunks
    for(int i=0; i < bboxes.size(); i++)
    {
        const CGAL::Bbox_3 &curBbox = bboxes[i];
        if (verbose) {
            cout << endl << "Bbox \033[1;31m" << i << "\033[0m out of " << bboxes.size() << endl;
            cout << "Current bbox: " << curBbox << endl;
        }

        // We build the arrangement corresponding to the current bounding box
        auto fullArrangement = VoxelArrangement(curBbox, voxelSide);

        // We compute the labels
        fullArrangement.assignLabel(labeledShapes, nbClasses, verbose);

        // If we have an empty chunk, we discard it
        if(fullArrangement.isLabelEmpty()) continue;

        // We compute the features
        fullArrangement.computeFeaturesRegular(pointCloud, pointOfViews, pointCloudLabels, nbClasses, verbose);

        // We save the current chunk
        string outPath(path + padTo(to_string(i), 4) + ".h5");
        fullArrangement.saveAsHdf(outPath);
    }
    return bboxes.size();
}
