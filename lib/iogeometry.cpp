#include "iogeometry.h"

using namespace std;
using Json = nlohmann::json;

/* Load points from an obj files */
pair<vector<Triangle>, TriangleClassMap>
loadTrianglesFromObj(const string &objFile, const vector<classKeywordsColor> &classes) {
    vector<Triangle> triangles;
    vector<Point> points;
    vector<vector<int>> faces;


    //Loading the obj data
    ifstream inputStream(objFile.c_str());
    if (!inputStream) {
        cerr << "Could not load file located at : " << objFile << endl;
        return pair<vector<Triangle>, TriangleClassMap>();
    }

    //Loading the triangles
    string currentLine;
    int cur_class_idx(-1);
    vector<int> triClasses;
    TriangleClassMap triangleToColors;
    while (getline(inputStream, currentLine)) {
        stringstream ss(currentLine);
        string firstCaracter;
        float vx, vy, vz;
        string f1, f2, f3, obj_name;
        ss >> firstCaracter;
        if (firstCaracter == "v") {
            // Vertice
            ss >> vx >> vy >> vz;
            points.emplace_back(vx, vy, vz);
        } else if (firstCaracter == "f") {
            // Face
            ss >> f1 >> f2 >> f3;
            vector<string> curFace = {f1, f2, f3};
            vector<int> curFaceIdx;
            for (auto idxStr: curFace)
                curFaceIdx.push_back(stoi(idxStr.substr(0, idxStr.find("/"))) - 1);
            if(cur_class_idx != -1) {
                faces.emplace_back(curFaceIdx);
                triClasses.push_back(cur_class_idx);
            }
        } else if (firstCaracter == "o") {
            // Object - Finding the corresponding class
            ss >> obj_name;
            cur_class_idx = -1;
            bool classFound = false;
            for (int i=0; i < classes.size(); i++) {
                const auto& cl = classes[i];
                for (auto keyword: get<1>(cl)) {
                    if (obj_name.find(keyword) != string::npos) {
                        cur_class_idx = i;
                        classFound = true;
                        break;
                    }
                }
                if (classFound) break;
            }
            if(obj_name.find("807510") != string::npos)
                cout << "debug" << endl;
            if(!classFound)
                cout << "Could not find class for piece: " << obj_name << endl;
        }
    }

    for (int i = 0; i < faces.size(); i++) {
        Triangle cur_triangle = Triangle(points[faces[i][0]], points[faces[i][1]], points[faces[i][2]]);
        triangles.push_back(cur_triangle);
        triangleToColors[cur_triangle] = triClasses[i];
    }

    return pair<vector<Triangle>, TriangleClassMap>(triangles, triangleToColors);
}

vector<Point> loadPointOfViews(const string &jsonFile) {
    vector<Point> pointOfViews;

    //Loading json data
    Json inputData;
    ifstream inputStream(jsonFile.c_str());
    if (!inputStream) {
        cerr << "Could not load file located at : " << jsonFile << endl;
        return vector<Point>();
    }
    inputStream >> inputData;

    //Loading point of views
    Json pointOfViewArray = inputData.at("cam_loc");
    for (const auto &povJson : pointOfViewArray)
        pointOfViews.emplace_back((double) povJson[0], (double) povJson[1], (double) povJson[2]);

    return pointOfViews;
}

vector<Point> loadPointCloudObj(const string &inFile)
{
    //Loading the obj data
    ifstream inputStream(inFile.c_str());
    if (!inputStream) {
        cerr << "Could not load file located at : " << inFile << endl;
        return vector<Point>();
    }
    vector<Point> points;
    string currentLine;
    while (getline(inputStream, currentLine)) {
        stringstream ss(currentLine);
        string firstCaracter;
        float vx, vy, vz;
        ss >> firstCaracter;
        if (firstCaracter == "v") {
            ss >> vx >> vy >> vz;
            points.emplace_back(vx, vy, vz);
        }
    }
    return points;
}

pair<vector<Point>, vector<int>> loadPointsWithLabel(const string &inFile)
{
    //Loading the obj data
    ifstream inputStream(inFile.c_str());
    if (!inputStream) {
        cerr << "Could not load file located at : " << inFile << endl;
        return make_pair(vector<Point>(), vector<int>());
    }
    vector<Point> points;
    vector<int> labels;
    string currentLine;
    while (getline(inputStream, currentLine)) {
        stringstream ss(currentLine);
        string firstCaracter;
        float vx, vy, vz, idx;
        ss >> firstCaracter;
        if (firstCaracter == "v") {
            ss >> vx >> vy >> vz;
            points.emplace_back(vx, vy, vz);
        }
        else if(firstCaracter == "vla") {
            ss >> idx;
            labels.push_back(idx);
        }
    }
    return make_pair(points, labels);

}

vector<facesLabelName> loadTreesFromObj(const string &inFile, const vector<classKeywordsColor> &classes)
{
    vector<facesLabelName> allTrees;

    //Loading the obj data
    ifstream inputStream(inFile.c_str());
    if (!inputStream) {
        cerr << "Could not load file located at : " << inFile << endl;
        return allTrees;
    }

    //Loading the triangles
    string currentLine;
    vector<colorTuple> triClasses;
    TriangleClassMap triangleToColors;
    vector<Point> points;
    vector<vector<int>> faces;
    int curClass = -1;
    string curName;
    while (getline(inputStream, currentLine)) {
        stringstream ss(currentLine);
        string firstCaracter;
        float vx, vy, vz;
        string f1, f2, f3, obj_name;
        ss >> firstCaracter;
        if (firstCaracter == "v") {
            // Vertice
            ss >> vx >> vy >> vz;
            points.emplace_back(vx, vy, vz);
        } else if (firstCaracter == "f") {
            // Face
            ss >> f1 >> f2 >> f3;
            vector<string> curFace = {f1, f2, f3};
            vector<int> curFaceIdx;
            for (const auto& idxStr: curFace)
                curFaceIdx.push_back(stoi(idxStr.substr(0, idxStr.find("/"))) - 1);
            faces.emplace_back(curFaceIdx);
        } else if (firstCaracter == "o") {
            // Object - Finding the corresponding class
            ss >> obj_name;
            bool classFound = false;
            int newClass = -1;
            string newName;
            for (int i=0; i < classes.size(); i++) {
                auto &cl = classes[i];
                for (auto &keyword: get<1>(cl)) {
                    if (obj_name.find(keyword) != string::npos) {
                        newClass = i;
                        newName = obj_name;
                        classFound = true;
                        break;
                    }
                }
            }
            if(!classFound){
                cout << "Could not assign a class to: " << obj_name << endl;
            }
            // First we make the previous tree
            vector<Triangle> triangles;
            for(auto &triIdx: faces) {
                auto curTriangle = new Triangle(points[triIdx[0]], points[triIdx[1]], points[triIdx[2]]);

                if(!Kernel().is_degenerate_3_object()(*curTriangle))
                    triangles.push_back(*curTriangle);
            }
            if(!triangles.empty() && curClass != -1)
                allTrees.emplace_back(triangles, curClass, curName);
            //Empty the faces even if the class has not been found
            faces = vector<vector<int>>();
            if (classFound) {
                // Class has been found
                // The object we're now going to read has class newClass
                curClass = newClass;
                curName = newName;
            } else
                // Class has not been found
                curClass = -1;
        }
    }
    // We make the last tree
    vector<Triangle> triangles;
    for(auto &triIdx: faces) {
//        auto curTriangle = new Triangle(points[triIdx[0]], points[triIdx[1]], points[triIdx[2]]);
        auto curTriangle = Triangle(points[triIdx[0]], points[triIdx[1]], points[triIdx[2]]);

        if(!Kernel().is_degenerate_3_object()(curTriangle))
            triangles.push_back(curTriangle);
    }
    if(!triangles.empty() && curClass != -1) {
//        Tree *curTree = new Tree(triangles.begin(), triangles.end());
        /*// Weird stuff I need to do to make it work...
        for(auto triPtr: triangles) {
            vector<Triangle> triVec = {triPtr};
            curTree->insert(AABB_triangle_traits::Primitive(triVec.begin()));
        }*/
        allTrees.emplace_back(triangles, curClass, curName);

//        //DEBUG
//        Kernel::Ray_3 query(Kernel::Point_3(0.2, 0.2, 0.2), Kernel::Vector_3(0., 1., 1.));
//        list<Ray_intersection> intersections;
//        curTree->all_intersections(query, back_inserter(intersections));

//        for(auto triPtr: triangles)
//            cout << triPtr << endl;
//        cout << endl;
//        for(auto prim: allTrees[allTrees.size() - 1].first->m_primitives)
//            cout << prim.datum() << endl;
//        cout << endl;
//        cout << "Nb of intersections: " << intersections.size() << endl;
        //cout << "debug" << endl;
    }
    //Empty the faces
    //Debug
//    cout << "input bbox : " << CGAL::bbox_3(points.begin(), points.end()) << endl;
    return allTrees;
}

void savePointsAsObj(const vector<Point>& points, const string &outPath) {
    stringstream fileOut(outPath);
    for (const auto point: points)
        fileOut << "v " << point.x() << " " << point.y() << " " << point.z() << endl;
    ofstream realFileOut(outPath.c_str());
    realFileOut << fileOut.rdbuf();
    realFileOut.close();
}

void savePointsAsObjWithLabel(const pair<vector<Point>, map<Point, int>> &pointsWithLabel, const string &outPath) {
    stringstream fileOut(outPath);
    for(const auto point: pointsWithLabel.first)
    {
        fileOut << "v " << point.x() << " " << point.y() << " " << point.z() << endl;
        fileOut << "vla " << pointsWithLabel.second.at(point) << endl;
    }
    ofstream realFileOut(outPath.c_str());
    realFileOut << fileOut.rdbuf();
    realFileOut.close();
}

void savePointsAsObjWithColors(vector<Point> points, vector<colorTuple> colors, const string &outPath) {
    stringstream fileOut(outPath);
    for (int i = 0; i < points.size(); i++) {
        const Point &point = points[i];
        const colorTuple &color = colors[i];
        fileOut << "v " << point.x() << " " << point.y() << " " << point.z()
                << " " << get<0>(color) << " " << get<1>(color) << " " << get<2>(color) << endl;
    }
    ofstream realFileOut(outPath.c_str());
    realFileOut << fileOut.rdbuf();
    realFileOut.close();
}

void saveTrianglesAsObj(const vector<Triangle>& triangles, const string &outPath, TriangleClassMap triangleClasses,
                        const vector<classKeywordsColor> &classes) {
    stringstream fileOut;
    for (auto triangle : triangles) {
        colorTuple color = get<2>(classes[triangleClasses[triangle]]);
        for (int i = 0; i < 3; i++)
            fileOut << "v " << triangle[i].x() << " " << triangle[i].y() << " " << triangle[i].z() << " "
                    << get<0>(color) << " " << get<1>(color) << " " << get<2>(color) << endl;
    }
    for (int i = 0; i < triangles.size(); i++)
        fileOut << "f " << 3 * i + 1 << " " << 3 * i + 2 << " " << 3 * i + 3 << endl;
    ofstream realFileOut(outPath.c_str());
    realFileOut << fileOut.rdbuf();
    realFileOut.close();
}

void saveSeparatedObj(vector<Triangle> triangles, const string &outPath, TriangleClassMap triangleClasses,
                      const vector<classKeywordsColor> &classes) {
    ofstream fileOut(outPath + to_string(0) + ".obj");
    int fileIdx(1);
    colorTuple curColor = get<2>(classes[triangleClasses[triangles[0]]]);
    int curTriangleIndex = 0;
    for (auto triangle : triangles) {
        colorTuple color = get<2>(classes[triangleClasses[triangle]]);
        if (curColor != color) {
            //fileOut = ofstream(outPath + to_string(fileIdx) + ".obj");
            fileIdx++;
            curColor = color;
            curTriangleIndex = 0;
        }
        for (int i = 0; i < 3; i++)
            fileOut << "v " << triangle[i].x() << " " << triangle[i].y() << " " << triangle[i].z() << " "
                    << get<0>(color) << " " << get<1>(color) << " " << get<2>(color) << endl;
        fileOut << "f "
                << 3 * curTriangleIndex + 1 << " "
                << 3 * curTriangleIndex + 2 << " "
                << 3 * curTriangleIndex + 3 << endl;
        curTriangleIndex++;
    }
}

void savePlyFromEdgeFeatures(const std::string &filename, Arrangement &arr, const std::map<int, int> &cell2label,
                             const EdgeFeatures &edgeFeatures, const std::vector<classKeywordsColor> &classesWithColor)
{
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

    for(auto itf = arr.facets_begin(); itf != arr.facets_end(); itf++){
        // We don't draw the unbounded / bbox facets
        Arrangement::Face& f = *itf;
        f._info = -1;
        itf->to_draw = false;
        if(!arr.is_facet_bounded(f)){continue;}
        Arrangement::Face_handle ch0 = f.superface(0), ch1 = f.superface(1);
        if(!arr.is_cell_bounded(ch0) || !arr.is_cell_bounded(ch1)) continue;
        pair<int, int> key1(cell2label.at(ch0), cell2label.at(ch1));
        pair<int, int> key2(cell2label.at(ch1), cell2label.at(ch0));
        pair<int, int> goodKey(-1, -1);
        if(edgeFeatures.find(key1) != edgeFeatures.end())
            goodKey = key1;
        else if(edgeFeatures.find(key2) != edgeFeatures.end())
            goodKey = key2;
        assert(goodKey.first != -1);
        f.to_draw = true;

        // We find the dominating label of the edge feature
        int goodLabel = -1;
        double bestScore = -1;
        for(int i=0; i < classesWithColor.size() + 1; i++)
            if(edgeFeatures.at(goodKey)[i] > bestScore)
            {
                bestScore = edgeFeatures.at(goodKey)[i];
                goodLabel = i;
            }
        if(goodLabel != -1)
            f._info = goodLabel;

        // We don't draw edges whose dominating label is the void label and edges with null edge vectors
        if(goodLabel == classesWithColor.size() || bestScore == 0.)
            f.to_draw = false;
    }

    typedef Polyhedral_complex_3::Mesh_3<> Mesh;
    typedef Polyhedral_complex_3::Mesh_extractor_3<Arrangement,Mesh> Extractor;
    Mesh meshGC;
    Extractor extractorGC(arr);
    extractorGC.extract(meshGC,false);
    {
        std::ofstream stream(filename.c_str());
        if (!stream.is_open())
            return ;
        Polyhedral_complex_3::print_mesh_with_facet_color_PLY(stream, meshGC, colormap);
        stream.close();
    }
}

void saveArrangement(const string &name, const vector<Kernel::Plane_3> &planes, int maxNumberOfPlanes,
        const CGAL::Bbox_3 &bbox, const map<int, int> &cell2label, const vector<bool> &labels)
{
    // Planes
    Json planesData;
    for(auto &planeIt: planes)
    {
        Vector ortDir = planeIt.orthogonal_vector();
        Point inlier = planeIt.projection(CGAL::ORIGIN);
        Json planeNormal = {ortDir.x(), ortDir.y(), ortDir.z()};
        Json planeInlier = {inlier.x(), inlier.y(), inlier.z()};
        Json planeData = {{"normal", planeNormal}, {"inlier", planeInlier}};
        planesData.push_back(planeData);
    }

    // Bounding box
    Json bboxData = {bbox.xmin(), bbox.ymin(), bbox.zmin(), bbox.xmax(), bbox.ymax(), bbox.zmax()};

    // Mapping
    Json mapping;
    for(auto &mapIt: cell2label)
        mapping.push_back({to_string(mapIt.first).c_str(), mapIt.second});

    // Labels
    Json labelsData(labels);

    // Compile data
    Json outputData = {{"planes", planesData}, {"bbox", bboxData}, {"map", mapping}, {"labels", labelsData},
                       {"nbPlanes", maxNumberOfPlanes}};

    // Output arrangement
    ofstream outFile(name);
    outFile << outputData;
}

PlaneArrangement::PlaneArrangement(const string& name) : isArrangementComputed(false), computedSamplesPerCell(0)
{
    cout << "Loading arrangement!" << endl;
    fstream inStream(name);
    Json data;
    inStream >> data;

    // Bounding box
    vector<double> box = data["bbox"];
    _bbox = CGAL::Bbox_3(box[0], box[1], box[2], box[3], box[4], box[5]);

    // Planes
    vector<Json> planes = data["planes"];
    _nbPlanes = data["nbPlanes"].get<int>();


    int nbPlanesUsed = 0;

    for(int i = 0; i < _nbPlanes; i++)
    {
        Json &norm = planes[i]["normal"];
        Kernel2::Vector_3 normal((double) norm[0], (double) norm[1], (double) norm[2]);
        Json &inl = planes[i]["inlier"];
        Kernel2::Point_3 inlier((double) inl[0], (double) inl[1], (double) inl[2]);

        if (planes[i].find("faces") != planes[i].end() &&
        planes[i].find("cumulatedPercentage") != planes[i].end()) {
            double cumulatedPercentage = planes[i]["cumulatedPercentage"];
            vector<vector<int>> faces(planes[i]["faces"].size(), vector<int>(3));
            for(int j = 0; j < planes[i]["faces"].size(); j++)
                for(int k=0; k < 3; k++)
                    faces[j][k] = planes[i]["faces"][j][k];
            _planes.push_back({inlier, normal, faces, cumulatedPercentage});
        }
        nbPlanesUsed++;
    }
    cout << "Nb of planes found " << nbPlanesUsed << endl;

    // Point cloud of the underlying mesh
    if (data.find("pointCloud") != data.end())
    {
        _points = vector<Point>(data["pointCloud"].size(), Point());
        for(int i=0; i < data["pointCloud"].size(); i++)
            _points[i] = Point(data["pointCloud"][i][0], data["pointCloud"][i][1], data["pointCloud"][i][2]);
    }

    //Mapping and labels
    if (data["map"].find("NOMAP") != data["map"].end())
    {
        // If the mapping does not exist, we create it
        int cellIter = 0;
        for(auto cellIt = _arr.cells_begin(); cellIt != _arr.cells_end(); cellIt++)
            if(_arr.is_cell_bounded(*cellIt))
                _cell2label[_arr.cell_handle(*cellIt)] = cellIter++;

        //Labels
        _labels = vector<int>(_cell2label.size(), false);
    }
    else {
        // If the mapping exists we load it
        for (auto elem = data["map"].begin(); elem != data["map"].end(); elem++)
            _cell2label[stoi(elem.key())] = elem.value().get<int>();

        //Labels
        if (data.find("labels") != data.end())
            _labels = data["labels"].get<vector<int>>();

        //gtLabels
        if (data.find("gtLabels") != data.end())
            _gtLabels = data["gtLabels"].get<vector<int>>();
    }

    // Node features
    if (data.find("NodeFeatures") != data.end())
        _nodeFeatures = data["NodeFeatures"].get<NodeFeatures>();

    // Edge features
    if (data.find("EdgeFeatures") != data.end())
    {
        for(auto edgeFeat = data["EdgeFeatures"].begin(); edgeFeat != data["EdgeFeatures"].end(); edgeFeat++)
        {
            vector<int> keyVec = edgeFeat[0][0].get<vector<int>>();
            pair<int, int> key(keyVec[0], keyVec[1]);
            vector<double> val = edgeFeat[0][1].get<vector<double>>();
            _edgeFeatures[key] = val;

//            // DEBUG
//            bool display = true;
//            for(auto bin: val)
//                if(bin != 0. && bin != 1.)
//                    display = true;
//            if(display) {
//                cout << "Key (" << key.first << ", " << key.second << ") has value ";
//                for (auto bin: val)
//                    cout << bin << " ";
//                cout << endl;
//            }
//            // END DEBUG
        }
    }

    // Node features
    if (data.find("NodePoints") != data.end()) {
        for(auto point: data["NodePoints"])
            _cellPoints.push_back(point.get<vector<double>>());
    }

    // Node Bboxes
    if (data.find("NodeBbox") != data.end()) {
        for(auto bbox: data["NodeBbox"])
            _nodeBboxes.emplace_back(bbox[0], bbox[1], bbox[2], bbox[3], bbox[4], bbox[5]);
    }

    cout << "Arrangement loaded" << endl;
}

PlaneArrangement::PlaneArrangement(const vector<Plane> &inPlanes, const map<int, int> &cell2label,
                                   const CGAL::Bbox_3 &inBbox) : isArrangementComputed(false),
                                   computedSamplesPerCell(0), _nbPlanes(inPlanes.size()), _bbox(inBbox),
                                   _cell2label(cell2label) {

    // Planes
    for(const auto &plane: inPlanes)
        _planes.push_back(plane);
}


void PlaneArrangement::saveAsJson(const string &outPath) const {
    // Compiling data into json
    Json data;
    Json cell2labelJ;
    for(auto idx: _cell2label)
        cell2labelJ[to_string(idx.first)] = idx.second;
    data["map"] = cell2labelJ;
    data["NodeFeatures"] = _nodeFeatures;
    data["EdgeFeatures"] = _edgeFeatures;
    data["labels"] = _labels;
    data["gtLabels"] = _gtLabels;
    if(_cellPoints.empty())
        data["NodePoints"] = getCellsPoints(_cell2label, _arr);
    else
        data["NodePoints"] = _cellPoints;
    if(_nodeBboxes.empty())
        data["NodeBbox"] = getCellsBbox(_cell2label, _arr);
    else
        data["NodeBbox"] = _nodeBboxes;
    data["planes"] = _planes;
    data["bbox"] = _bbox;
    data["nbPlanes"] = _nbPlanes;
    vector<vector<double>> pointCloud(_points.size(), vector<double>(3));
    for(int i=0; i < _points.size(); i++)
        pointCloud[i] = {_points[i].x(), _points[i].y(), _points[i].z()};
    data["pointCloud"] = pointCloud;

    // Save to disk
    ofstream outFile(outPath);
    outFile << data;
    outFile.close();
}

void PlaneArrangement::setEdgeLabels(const EdgeFeatures& edgeFeatures)
{
    if(!_edgeFeatures.empty() && _edgeFeatures.size() != edgeFeatures.size())
        cout << "Careful! Replacing edge features by new features with different size..." << endl;
    _edgeFeatures = edgeFeatures;
}

PlaneArrangement::PlaneArrangement(const vector<Plane> &inPlanes, const vector<int>& validPlaneIdx,
        const CGAL::Bbox_3 &inBbox) : isArrangementComputed(false), computedSamplesPerCell(0) {

    // Planes
    _nbPlanes = validPlaneIdx.size();
    for(auto idx: validPlaneIdx)
        _planes.push_back(inPlanes[idx]);

    // Bbox
    _bbox = inBbox;

    // cell2Label
    int idx = 0;
    auto &arr = arrangement();
    for(auto cellIt = _arr.cells_begin(); cellIt != _arr.cells_end(); cellIt++)
    {
        if(!_arr.is_cell_bounded(*cellIt)) continue;
        _cell2label[_arr.cell_handle(*cellIt)] = idx++;
    }

}

Arrangement &PlaneArrangement::arrangement() {
    if(!isArrangementComputed)
    {
        isArrangementComputed = true;
        _arr.set_bbox(_bbox);
        auto tqPlanes = tq::tqdm(_planes);
        tqPlanes.set_prefix("Inserting " + to_string(_planes.size()) + " planes: ");
        for(const auto& plane: tqPlanes)
            _arr.insert(Kernel2::Plane_3(plane.inlier, plane.normal));
    }
    return _arr;
}

const std::map<int, int> &PlaneArrangement::cell2label() const {
    return _cell2label;
}

const std::vector<int> &PlaneArrangement::labels() const {
    return _labels;
}

const std::vector<int> &PlaneArrangement::gtLabels() const {
    return _gtLabels;
}

const CGAL::Bbox_3 &PlaneArrangement::bbox() const {
    return _bbox;
}

const vector<Plane> &PlaneArrangement::planes() const {
    return _planes;
}

const vector<Point> &PlaneArrangement::points() const {
    return _points;
}

const EdgeFeatures &PlaneArrangement::edgeFeatures() const {
    return _edgeFeatures;
}

vector<classKeywordsColor> loadSemanticClasses(const string& path)
{
    vector<classKeywordsColor> semanticClasses;

    //Loading json data
    Json inputData;
    ifstream inputStream(path.c_str());
    if(!inputStream)
    {
        cerr << "Could not load file located at : " << path << endl;
        return vector<classKeywordsColor>();
    }
    inputStream >> inputData;
    for( auto itr = inputData.begin() ; itr != inputData.end() ; itr++ ) {

        //Color
        Json colorArray = itr.value().at("color");
        vector<int> colorVec;
        for (auto &color_itr : colorArray)
            colorVec.push_back(color_itr);
        colorTuple color(colorVec[0], colorVec[1], colorVec[2]);

        // Keywords
        Json keywordsArray = itr.value().at("keywords");
        vector<string> keywords;
        for (auto &str_it : keywordsArray)
            keywords.push_back(str_it);
        semanticClasses.emplace_back(itr.key(), keywords, color);
    }
    return semanticClasses;
}


vector<vector<double>> getCellsPoints(const map<int, int> &cell2label, const Arrangement &arr) {
    vector<vector<double>> cellsPoints(cell2label.size(), {0., 0., 0.});
    Epeck_to_Simple e2s;

    for(auto cellIt = arr.cells_begin(); cellIt != arr.cells_end(); cellIt++) {
        if(!arr.is_cell_bounded(*cellIt)) continue;

        auto pt = e2s(cellIt->point());
        cellsPoints[cell2label.at(arr.cell_handle(*cellIt))] = {pt.x(), pt.y(), pt.z()};
    }

    return cellsPoints;
}

vector<vector<double>> getCellsBbox(const map<int, int> &cell2label, const Arrangement &arr) {
    vector<vector<double>> cellsPoints(cell2label.size(), {0., 0., 0.});
    Epeck_to_Simple e2s;

    for(auto cellIt = arr.cells_begin(); cellIt != arr.cells_end(); cellIt++) {
        if(!arr.is_cell_bounded(*cellIt)) continue;
        cellsPoints[cell2label.at(arr.cell_handle(*cellIt))] = {DBL_MAX, DBL_MAX, DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX};
        for(auto facetIt = cellIt->subfaces_begin(); facetIt != cellIt->subfaces_end(); facetIt++)
        {
            auto facet = arr.facet(*facetIt);
            for(auto edgeIt = facet.subfaces_begin(); edgeIt != facet.subfaces_end(); edgeIt++) {
                auto edge = arr.edge(*edgeIt);
                for(auto pointIt = edge.subfaces_begin(); pointIt != edge.subfaces_end(); pointIt++)
                {
                    cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][0] = min(cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][0], CGAL::to_double(arr.point(*pointIt).x()));
                    cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][1] = min(cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][1], CGAL::to_double(arr.point(*pointIt).y()));
                    cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][2] = min(cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][2], CGAL::to_double(arr.point(*pointIt).z()));
                    cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][3] = max(cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][3], CGAL::to_double(arr.point(*pointIt).x()));
                    cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][4] = max(cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][4], CGAL::to_double(arr.point(*pointIt).y()));
                    cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][5] = max(cellsPoints[cell2label.at(arr.cell_handle(*cellIt))][5], CGAL::to_double(arr.point(*pointIt).z()));
                }
            }
        }
    }

    return cellsPoints;
}

/*
 * Hit and run algorithm
 * */
void PlaneArrangement::sampleInConvexCell(int cellHandle, int nbSamples)
{
    Epeck_to_Simple e2s;
    const auto& cell = _arr.cell(cellHandle);
    default_random_engine generator(random_device{}());
    normal_distribution<double> normalDist(0., 1.);

    Point curPoint = e2s(cell.point());
    for(int i=0; i < nbSamples - 1; i++)
    {
        // Add new point to the samples
        _samples.emplace_back(curPoint, cellHandle);

        // Shoot a ray in a random direction
        Vector direction(normalDist(generator), normalDist(generator), normalDist(generator));
        Ray rayP(curPoint, direction);
        Ray rayN(curPoint, -direction);

        double distP = DBL_MAX;
        double distN = DBL_MAX;
        for(auto facetIt = cell.subfaces_begin(); facetIt != cell.subfaces_end(); facetIt++)
        {
            const auto& facet = _arr.facet(*facetIt);
            const auto& curPlane = e2s(_arr.plane(_arr.facet_plane(_arr.facet(*facetIt))));

            // Intersection with the positive ray
            auto intersectionP = CGAL::intersection(curPlane, rayP);
            if(intersectionP) {
                if (const Point *s = boost::get<Point>(&*intersectionP)) {
                    double dist = (*s - curPoint).squared_length();
                    if (dist < distP)
                        distP = dist;
                }
            }

            // Intersection with the negative ray
            auto intersectionN = CGAL::intersection(curPlane, rayN);
            if(intersectionN) {
                if (const Point *s = boost::get<Point>(&*intersectionN)) {
                    double dist = (*s - curPoint).squared_length();
                    if (dist < distN)
                        distN = dist;
                }
            }
        }

        // Make the new point
        uniform_real_distribution<double> unifDist(-sqrt(distN), sqrt(distP));
        double distToCurPoint = unifDist(generator);
        curPoint = curPoint + distToCurPoint * direction / sqrt(direction.squared_length());
    }
}

const vector<pair<Point, int>> &PlaneArrangement::getSamples(int nbSamplesPerCell)
{
    if(computedSamplesPerCell != nbSamplesPerCell)
    {
        computedSamplesPerCell = nbSamplesPerCell;
        _samples = std::vector<std::pair<Point, int>>();

        for(auto cellIt = _arr.cells_begin(); cellIt != _arr.cells_end(); cellIt++) {
            if (!_arr.is_cell_bounded(*cellIt)) continue;
            sampleInConvexCell(_arr.cell_handle(*cellIt), nbSamplesPerCell);
        }
    }
    return _samples;
}