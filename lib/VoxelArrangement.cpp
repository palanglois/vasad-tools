#include "VoxelArrangement.h"

using namespace std;
using Json = nlohmann::json;
using namespace HighFive;

VoxelArrangement::VoxelArrangement(const std::string &name) : isArrangementComputed(false),
                                                              areBboxPlanesComputed(false), _width(0), _height(0),
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



    // Planes
    Epeck_to_Simple e2s;
    vector<Plane> epeckPlanes = data["planes"].get<vector<Plane>>();
    for(const auto& plane: epeckPlanes)
        _planes.push_back({e2s(plane.inlier), e2s(plane.normal), plane.faces, plane.cumulatedPercentage});

    // Bbox
    _bbox = data["bbox"].get<CGAL::Bbox_3>();

    // Voxel side size
    _voxelSide = data["voxelSide"].get<double>();

}

void VoxelArrangement::loadHdf(const std::string &name) {
    H5Easy::File file(name, H5Easy::File::ReadOnly);

    // Features
    if(file.exist("/features"))
        _features = H5Easy::load<FeatTensor>(file, "/features");

    // Labels
    if(file.exist("/labels"))
        _labels = H5Easy::load<LabelTensor>(file, "/labels");

    // Rich features
    if(file.exist("/richFeatures"))
        _richFeatures = H5Easy::load<FeatTensor>(file, "/richFeatures");

    // width, height, depth
    if(!_labels.empty())
    {
        _width = _labels.size();
        _height = _labels[0].size();
        _depth = _labels[0][0].size();
    }


    // Planes
    if(file.exist("/planes")) {
        auto inliers = H5Easy::load<vector<vector<double>>>(file, "/planes/inliers");
        auto normals = H5Easy::load<vector<vector<double>>>(file, "/planes/normals");
        auto faces = H5Easy::load<vector<vector<vector<int>>>>(file, "/planes/faces");
        auto cumulatedPercentages = H5Easy::load<vector<double>>(file, "/planes/cumulatedPercentages");
        for (int i = 0; i < inliers.size(); i++) {
            Point inlier(inliers[i][0], inliers[i][1], inliers[i][2]);
            Vector normal(normals[i][0], normals[i][1], normals[i][2]);
            vector<vector<int>> curFaces = faces[i];
            double cumulatedPercentage = cumulatedPercentages[i];
            _planes.push_back({inlier, normal, curFaces, cumulatedPercentage});
        }
    }

    // Bbox
    auto bbox = H5Easy::load<vector<double>>(file, "/bbox");
    _bbox = CGAL::Bbox_3(bbox[0], bbox[1], bbox[2], bbox[3], bbox[4], bbox[5]);

    // Voxel side size
    _voxelSide = H5Easy::load<double>(file, "/voxelSide");

}

VoxelArrangement::VoxelArrangement(const CGAL::Bbox_3 &inBbox, double inVoxelSide) : _bbox(inBbox),
_voxelSide(inVoxelSide), isArrangementComputed(false), areBboxPlanesComputed(false), _width(0), _height(0), _depth(0)
{
    // Compute the planes
    computePlanes();

    // Init labels, features and rich features
    _labels = LabelTensor(0);
    _features = FeatTensor(0);
    _richFeatures = FeatTensor(0);
}

void VoxelArrangement::computePlanes()
{
    Simple_to_Epeck s2e;
    // Planes
    vector<double> ranges = {_bbox.xmax() - _bbox.xmin(),
                             _bbox.ymax() - _bbox.ymin(),
                             _bbox.zmax() - _bbox.zmin()};
    _width = round(ranges[0] / _voxelSide);
    _height = round(ranges[1] / _voxelSide);
    _depth = round(ranges[2] / _voxelSide);
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
            PlaneSimple p = {{lowLeft[0], lowLeft[1], lowLeft[2]}, normal, faces, area};
            _planes.push_back(p);
        }
    }
    // Sort the planes from the biggest to the smallest
    sort(_planes.begin(), _planes.end(),
         [](const PlaneSimple & a, const PlaneSimple & b) -> bool
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
    Simple_to_Epeck s2e;
    if(isArrangementComputed) return;
    isArrangementComputed = true;
    _arr.set_bbox(_bbox);
    auto tqPlanes = tq::tqdm(_planes);
    tqPlanes.set_prefix("Inserting " + to_string(_planes.size()) + " planes: ");
    for (const auto &plane: tqPlanes)
        _arr.insert(Kernel2::Plane_3(s2e(plane.inlier), s2e(plane.normal)));

    for (auto cellIt = _arr.cells_begin(); cellIt != _arr.cells_end(); cellIt++) {
        if (!_arr.is_cell_bounded(*cellIt)) continue;
        const auto &centroid = cellIt->point();
        int idx_x = max(0, (int) floor((CGAL::to_double(centroid.x()) - _bbox.xmin()) / _voxelSide));
        int idx_y = max(0, (int) floor((CGAL::to_double(centroid.y()) - _bbox.ymin()) / _voxelSide));
        int idx_z = max(0, (int) floor((CGAL::to_double(centroid.z()) - _bbox.zmin()) / _voxelSide));

        _node2index[_arr.cell_handle(*cellIt)] = make_tuple(idx_x, idx_y, idx_z);
//        _index2node[make_tuple(idx_x, idx_y, idx_z)] = _arr.cell_handle(*cellIt);

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

void VoxelArrangement::computeBboxPlanes() {
    if(areBboxPlanesComputed) return;
    areBboxPlanesComputed = true;

    Vector normalX(1., 0., 0.);
    Vector normalY(0., 1., 0.);
    Vector normalZ(0., 0., 1.);
    Point pointMin(_bbox.xmin(), _bbox.ymin(), _bbox.zmin());
    Point pointMax(_bbox.xmax(), _bbox.ymax(), _bbox.zmax());
    vector<vector<int>> emptyFaces;
    double nullPercentage(0.);
    _bboxPlanes.push_back({pointMin, normalX, emptyFaces, nullPercentage});
    _bboxPlanes.push_back({pointMin, normalY, emptyFaces, nullPercentage});
    _bboxPlanes.push_back({pointMin, normalZ, emptyFaces, nullPercentage});
    _bboxPlanes.push_back({pointMax, normalX, emptyFaces, nullPercentage});
    _bboxPlanes.push_back({pointMax, normalY, emptyFaces, nullPercentage});
    _bboxPlanes.push_back({pointMax, normalZ, emptyFaces, nullPercentage});
}

int VoxelArrangement::getLabel(const VoxelArrangement::triplet& voxelIdx){

    // If we have rich features
    if(!_richFeatures.empty())
    {
        const vector<double> &curFeat = _richFeatures[get<0>(voxelIdx)][get<1>(voxelIdx)][get<2>(voxelIdx)];
        double featSum = 0.;
        double bestScore = -1.;
        int bestIdx = -1;
        for(int i=0; i < curFeat.size() - 1; i++) {
            if(curFeat[i] > bestScore)
            {
                bestScore = curFeat[i];
                bestIdx = i;
            }
            featSum += curFeat[i];
        }
        if(featSum == 0.) return -1;
        return bestIdx;
    }

    // If we have labels
    if(!_labels.empty())
        return _labels[get<0>(voxelIdx)][get<1>(voxelIdx)][get<2>(voxelIdx)];

    cerr << "Warning: no labels are riched features are available!!!" << endl;
    return -1;
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

bool VoxelArrangement::areRichFeaturesEmpty() const {
    bool empty = true;
    for(int i=0; i < _width; i++) {
        for (int j = 0; j < _height; j++) {
            for (int k = 0; k < _depth; k++) {
                int curRichFeatureAccum = 0;
                for(int l=0; l < _richFeatures[i][j][k].size() - 1; l++)
                    curRichFeatureAccum += _richFeatures[i][j][k][l];
                if (curRichFeatureAccum != 0) {
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

bool VoxelArrangement::areFeaturesEmpty() const {
    bool empty = true;
    for(int i=0; i < _width; i++) {
        for (int j = 0; j < _height; j++) {
            for (int k = 0; k < _depth; k++) {
                int curRichFeatureAccum = 0;
                for(int l=0; l < _features[i][j][k].size() - 1; l++)
                    curRichFeatureAccum += _features[i][j][k][l];
                if (curRichFeatureAccum != 0) {
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

int VoxelArrangement::numberOfCells() const
{
    return _width * _height * _depth;
}

VoxelArrangement::triplet VoxelArrangement::findVoxel(const Point &query) const {
    int idx_x = floor((CGAL::to_double(query.x()) - _bbox.xmin())/_voxelSide);
    int idx_y = floor((CGAL::to_double(query.y()) - _bbox.ymin())/_voxelSide);
    int idx_z = floor((CGAL::to_double(query.z()) - _bbox.zmin())/_voxelSide);
    return make_tuple(idx_x, idx_y, idx_z);
}

void VoxelArrangement::getIntersections(const vector<PlaneSimple> &planes, const Segment &segment, vector<Point> &points) const
{
    for(const auto &plane: planes)
    {
        PlaneCgal cgalPlane(plane.inlier, plane.normal);
        PlaneSegmentIntersection result = CGAL::intersection(cgalPlane, segment);
        if(!result) continue;
        if(const Point* p = boost::get<Point>(&*result)) {
            if(CGAL::do_overlap(_bbox, p->bbox())) points.push_back(*p);
        }
    }
}

vector<VoxelArrangement::triplet> VoxelArrangement::intersectSegment(const Point& p, const Point& q) const
{
    // Intersect the line segment vs all the planes
    Segment segment(p, q);
    vector<Point> allPoints;
    if(CGAL::do_overlap(_bbox, p.bbox())) allPoints.push_back(p);
    if(CGAL::do_overlap(_bbox, q.bbox())) allPoints.push_back(q);
    getIntersections(_planes, segment, allPoints);
    getIntersections(_bboxPlanes, segment, allPoints);

    vector<VoxelArrangement::triplet> validVoxels;
    if(allPoints.size() < 2) return validVoxels;

    // Finding a non-constant dimension
    int validDimension = -1;
    for(int dim = 0; dim < 3; dim++)
        if(allPoints[0][dim] != allPoints[2][dim])
        {
            validDimension = dim;
            break;
        }
    // Sorting the points according to this dimension
    sort(allPoints.begin(), allPoints.end(),
         [&](const Point & a, const Point & b) -> bool
         {
             return a[validDimension] > b[validDimension];
         });

    // Querying mid-points
    for(int i=0; i < allPoints.size() - 1; i++) {
        Point midPoint = CGAL::ORIGIN + ((allPoints[i + 1] + (allPoints[i] - CGAL::ORIGIN)) - CGAL::ORIGIN) / 2.;
        validVoxels.push_back(findVoxel(midPoint));
    }
    return validVoxels;

}

void VoxelArrangement::assignLabel(vector<facesLabelName> &labeledShapes, int nbClasses, bool verbose)
{
    Epeck_to_Simple e2s;
    // Gather all the points
    vector<pair<Point, int>> points;

    for(int i=0; i < _width; i++)
        for(int j=0; j < _height; j++)
            for(int k=0; k < _depth; k++) {
                Point cellPoint(_bbox.xmin() + _voxelSide / 2. + i * _voxelSide,
                                _bbox.ymin() + _voxelSide / 2. + j * _voxelSide,
                                _bbox.zmin() + _voxelSide / 2. + k * _voxelSide);
                points.emplace_back(cellPoint, -1);
            }



    // Label the points
    vector<int> labels = assignLabelToPoints(points, labeledShapes, nbClasses, _bbox);
    // Store them
    _labels = vector<vector<vector<int>>>(_width, vector<vector<int>>(_height, vector<int>(_depth, -1)));
    for (int i = 0; i < points.size(); i++) {
        tuple<int, int, int> idx = findVoxel(points[i].first);
        _labels[get<0>(idx)][get<1>(idx)][get<2>(idx)] = labels[i] == nbClasses ? -1: labels[i];
    }
}

void VoxelArrangement::computeRichFeatures(vector<facesLabelName> &labeledShapes, int nbClasses, bool verbose)
{
    Epeck_to_Simple e2s;
    // Gather all the points
    vector<pair<Point, int>> points;
    double offset = _voxelSide / 3.;
    vector<double> factors = {-1., 0., 1.};
    for(int i=0; i < _width; i++)
        for(int j=0; j < _height; j++)
            for(int k=0; k < _depth; k++) {
                Point cellPoint(_bbox.xmin() + _voxelSide / 2. + i * _voxelSide,
                                _bbox.ymin() + _voxelSide / 2. + j * _voxelSide,
                                _bbox.zmin() + _voxelSide / 2. + k * _voxelSide);
                for(auto ofX: factors)
                    for(auto ofY: factors)
                        for(auto ofZ: factors) {
                            Vector offsetVector(ofX*offset, ofY*offset, ofZ*offset);
                            points.emplace_back(cellPoint + offsetVector, -1);
                        }
            }



    // Label the points
    vector<int> labels = assignLabelToPoints(points, labeledShapes, nbClasses, _bbox);
    // Store them

    // Initialize the features
    _richFeatures = vector<vector<vector<vector<double>>>>(_width,
                      vector<vector<vector<double>>>(_height,
                              vector<vector<double>>(_depth,
                                      vector<double>(nbClasses + 1, 0.))));
    for (int i = 0; i < points.size(); i++) {
        tuple<int, int, int> idx = findVoxel(points[i].first);
        _richFeatures[get<0>(idx)][get<1>(idx)][get<2>(idx)][labels[i]]++;
    }
}

void VoxelArrangement::computeFeaturesRegular(const std::vector<Point> &points, const std::vector<Point> &pointOfViews,
                                              const std::vector<int> &labels, int nbClasses, bool verbose) {
    Simple_to_Epeck s2e;
    Epeck_to_Simple e2s;
    // Make sure that the bbox planes have been built
    computeBboxPlanes();
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
#pragma omp parallel for
    for(int i=0; i < validIdx.size(); i++) {
        const Point &point = endPoints[i];
        const Point &pov = beginPoints[i];
        const int &label = labels[validIdx[i]];

        bool isPointInBbox = CGAL::do_overlap(point.bbox(), _bbox);
        vector<triplet> intersectedVoxels = intersectSegment(pov, point);
        triplet pointVoxel = make_tuple(-1, -1, -1);
        if(isPointInBbox) pointVoxel = findVoxel(point);
        for(const auto& voxel: intersectedVoxels) {
            if (voxel != pointVoxel)
#pragma omp critical
                _features[get<0>(voxel)][get<1>(voxel)][get<2>(voxel)][nbClasses]++;
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

    // Planes
    Simple_to_Epeck s2e;
    vector<Plane> epeckPlanes;
    for(const auto& plane: _planes)
        epeckPlanes.push_back({s2e(plane.inlier), s2e(plane.normal), plane.faces, plane.cumulatedPercentage});
    Json planes = epeckPlanes;

    // NbPlanes
    Json nbPlanes = _planes.size();

    // Bbox
    Json bbox = _bbox;

    // Voxel side size
    Json voxelSide = _voxelSide;

    // Compiling the output data
    Json outputData = {{"features", features}, {"labels", labels}/*, {"map", map}*/, {"planes", planes},
                       {"nbPlanes", nbPlanes}, {"bbox", bbox}, {"voxelSide", voxelSide}};

    // Output the json file
    ofstream outFile(path);
    outFile << outputData;
}

void VoxelArrangement::saveAsHdf(const std::string &path, bool withRichFeatures) {
    H5Easy::File file(path, H5Easy::File::Overwrite);
    auto options = H5Easy::DumpOptions(H5Easy::Compression(), H5Easy::DumpMode::Overwrite);

    //Features
    if(!_features.empty())
        H5Easy::dump(file, "/features", _features, options);
    else
        cerr << "Warning !!! Saving a chunk with empty features!" << endl;

    // Labels or rich features
    if(withRichFeatures)
        H5Easy::dump(file, "/richFeatures", _richFeatures, options);
    else if(!_labels.empty())
        H5Easy::dump(file, "/labels", _labels, options);


    // Planes
    if(!_planes.empty()) {
        vector<vector<double>> inliers(_planes.size(), vector<double>(3, 0.));
        vector<vector<double>> normals(_planes.size(), vector<double>(3, 0.));
        vector<vector<vector<int>>> faces;
        vector<double> cumulatedPercentages;
        int planeIt = 0;
        for (const auto &pl: _planes) {
            Point inlier = pl.inlier;
            Vector normal = pl.normal;
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
    }

    // NbPlanes
    H5Easy::dump(file, "/nbPlanes", _planes.size(), options);

    // Bbox
    vector<double> bbox = {_bbox.xmin(), _bbox.ymin(), _bbox.zmin(), _bbox.xmax(), _bbox.ymax(), _bbox.zmax()};
    H5Easy::dump(file, "/bbox", bbox, options);

    // Voxel side size
    H5Easy::dump(file, "/voxelSide", _voxelSide, options);
}

void VoxelArrangement::saveAsPly(const string &path, const vector<classKeywordsColor> &classesWithColor) {
    Epeck_to_Simple e2s;
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
        triplet idxCh0 = findVoxel(e2s(_arr.cell(ch0).point()));
        triplet idxCh1 = findVoxel(e2s(_arr.cell(ch1).point()));
        int label1 = getLabel(idxCh0);
        int label2 = getLabel(idxCh1);
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
    Epeck_to_Simple e2s;
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
        triplet idxCh0 = findVoxel(e2s(_arr.cell(ch0).point()));
        triplet idxCh1 = findVoxel(e2s(_arr.cell(ch1).point()));
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

const std::vector<PlaneSimple> &VoxelArrangement::planes() const
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

const VoxelArrangement::FeatTensor & VoxelArrangement::richFeatures() const {
    return _richFeatures;
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
    const vector<PlaneSimple> &planes = globalVoxels.planes();
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
                                    int nbClasses, const string &path, int nbVoxelsAlongAxis,
                                    bool withRichFeatures, bool verbose)
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

        if(withRichFeatures)
        {
            // We compute the rich features
            fullArrangement.computeRichFeatures(labeledShapes, nbClasses, verbose);

            // If we have an empty chunk, we discard it
            if(fullArrangement.areRichFeaturesEmpty()) continue;
        }
        else {

            // We compute the labels
            fullArrangement.assignLabel(labeledShapes, nbClasses, verbose);

            // If we have an empty chunk, we discard it
            if (fullArrangement.isLabelEmpty()) continue;
        }

        // We compute the features
        fullArrangement.computeFeaturesRegular(pointCloud, pointOfViews, pointCloudLabels, nbClasses, verbose);

        // We save the current chunk
        string outPath(path + padTo(to_string(i), 4) + ".h5");
        fullArrangement.saveAsHdf(outPath, withRichFeatures);
    }
    return bboxes.size();
}


int splitLabeledPointCloud(const std::vector<Point> &pointOfViews,
                           const std::vector<Point> &pointCloud,
                           const std::vector<int> &pointCloudLabels,
                           double voxelSide, int nbClasses, const std::string &path, int nbVoxelsAlongAxis,
                           bool verbose)
{
    // Compute initial bounding box
    CGAL::Bbox_3 initialBbox;
    for (const auto &point: pointCloud)
        initialBbox += point.bbox();

    // Split it
    vector<CGAL::Bbox_3> bboxes = splitBigBbox(initialBbox, nbVoxelsAlongAxis, voxelSide);

    // Generate the chunks
    for(int i=0; i < bboxes.size(); i++) {
        const CGAL::Bbox_3 &curBbox = bboxes[i];
        if (verbose) {
            cout << endl << "Bbox \033[1;31m" << i << "\033[0m out of " << bboxes.size() << endl;
            cout << "Current bbox: " << curBbox << endl;
        }

        // We build the arrangement corresponding to the current bounding box
        auto fullArrangement = VoxelArrangement(curBbox, voxelSide);

        // We compute the features
        fullArrangement.computeFeaturesRegular(pointCloud, pointOfViews, pointCloudLabels, nbClasses, verbose);

        // If we have an empty chunk, we discard it
        if(fullArrangement.areFeaturesEmpty()) continue;

        // We save the current chunk
        string outPath(path + padTo(to_string(i), 4) + ".h5");
        fullArrangement.saveAsHdf(outPath);
    }
    return bboxes.size();
}
