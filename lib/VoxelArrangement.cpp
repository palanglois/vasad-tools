#include "VoxelArrangement.h"

using namespace std;
using Json = nlohmann::json;

VoxelArrangement::VoxelArrangement(const std::string &name) : isArrangementComputed(false), _width(0), _height(0),
                                                              _depth(0) {
    cout << "Loading voxels!" << endl;
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
        int nbPlanes = ceil(ranges[i] / _voxelSide);
        Vector& normal = normals[i];
        for (int j = 0; j < nbPlanes + 1; j++) {
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
        _labels[get<0>(idx)][get<1>(idx)][get<2>(idx)] = labels[i] == nbClasses ? -1: labels[i];
    }
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

    // Select the points that lie in the bounding box
    vector<Point> beginPoints;
    vector<Point> endPoints;
    vector<int> validIdx = addSegmentIfInBbox(pointOfViews, points, back_inserter(beginPoints), back_inserter(endPoints), _bbox);

    // Go through the points
#pragma omp parallel for
    for(int i=0; i < validIdx.size(); i++)
    {
        const Point &point = endPoints[i];
        const Point &pov = beginPoints[i];
        const int &label = labels[validIdx[i]];

        // Retrieve the closest facet to the current point
        int facetHandle = closestFacet(s2e(point));

        // Retrieve the index of the opposite cell of the points of view w.r.t the facet
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
#pragma omp critical
        _features[get<0>(coordinates)][get<1>(coordinates)][get<2>(coordinates)][label]++;

        // Visibility

        // Retrieving the cell in which lies the point of view
        int idx_x = floor((CGAL::to_double(pov.x()) - _bbox.xmin())/_voxelSide);
        int idx_y = floor((CGAL::to_double(pov.y()) - _bbox.ymin())/_voxelSide);
        int idx_z = floor((CGAL::to_double(pov.z()) - _bbox.zmin())/_voxelSide);
        int povCell = _index2node[make_tuple(idx_x, idx_y, idx_z)];

        // Intersect the (point_of_view <-> target facet point) segment with the plane arrangement
        Arrangement::Face_handle begin_cell;
        Arrangement::Face_handle end_cell;
        vector<pair<Arrangement::Face_handle, int>> intersectedFacets;
        vector<pair<Arrangement::Face_handle, double>> intersectedCellsAndDists;
        segment_search_advanced(_arr, s2e(pov), facetPoint, begin_cell, back_inserter(intersectedFacets),
                                back_inserter(intersectedCellsAndDists), end_cell, povCell, true);

        // Add visibility information
        for(const auto& cellAndDist: intersectedCellsAndDists){
            if(!_arr.is_cell_bounded(cellAndDist.first)) continue;
            triplet cellIdx = _node2index[cellAndDist.first];
#pragma omp critical
            _features[get<0>(cellIdx)][get<1>(cellIdx)][get<2>(cellIdx)][nbClasses]++;
        }
    }
    // Normalization
    for(int i=0; i < _width; i++)
        for(int j=0; j < _height; j++)
            for(int k=0; k < _depth; k++) {
                double nbElems = 0.;
                for (int l = 0; l < nbClasses + 1; l++)
                    nbElems += _features[i][j][k][l];
                if(nbElems == 0.) continue;
                for (int l = 0; l < nbClasses + 1; l++)
                    _features[i][j][k][l] /= nbElems;
            }
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
        vector<int> validPlaneIdx = computePlanesInBoundingBox(planes, pointCloudPlanes, curBbox, 1.);

        // If we have too many planes or no plane at all, we subdivide the current bounding box
        if(validPlaneIdx.size() > maxNbPlanes || validPlaneIdx.empty())
        {
            subdivideBboxLongestAxis(bboxes, curBbox);
            continue;
        }
        if(verbose)
            cout << "Found " << validPlaneIdx.size() << " valid planes in current bbox which is: " << curBbox << endl;

        // We build the arrangement corresponding to the current bounding box
        auto fullArrangement = VoxelArrangement(curBbox, voxelSide);

        // Compute the number of cells in the current arrangement
        int nbCells = fullArrangement.numberOfCells();

        // We need at least 2 nodes to have a valid chunk
        if(nbCells < 2) continue;

        // If we have too many cells, we update maxNbPlanes accordingly and we subdivide the current bounding box
        if(nbCells > maxNodes)
        {
            maxNbPlanes = validPlaneIdx.size();
            subdivideBboxLongestAxis(bboxes, curBbox);
            continue;
        }

        // We compute the labels
        fullArrangement.assignLabel(labeledShapes, nbClasses, verbose);

        // We compute the features
        fullArrangement.computeFeatures(pointCloud, pointOfViews, pointCloudLabels, nbClasses, verbose);

        // We save the current chunk
        string outPath(path + padTo(to_string(chunkIterator), 4) + ".json");
        fullArrangement.saveAsJson(outPath);
        chunkIterator++;
    }
    return chunkIterator;
}
