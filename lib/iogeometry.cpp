#include "iogeometry.h"

using namespace std;
using Json = nlohmann::json;
using namespace tinyply;


vector<string> splitString(const string& s, const string& sep) {
    size_t last = 0;
    size_t next = 0;
    vector<string> outputSplit;
    while ((next = s.find(sep, last)) != string::npos) {
        outputSplit.push_back(s.substr(last, next-last));
        last = next + 1;
    }
    outputSplit.push_back(s.substr(last, next-last));
    return outputSplit;
}

pair<vector<Point>, vector<vector<double>>> loadLightConvPointOutput(const string& path)
{
    //Loading the obj data
    ifstream inputStream(path.c_str());
    if (!inputStream) {
        cerr << "Could not load file located at : " << path << endl;
        return make_pair(vector<Point>(), vector<vector<double>>());
    }
    vector<Point> points;
    vector<vector<double>> labels;
    string currentLine;
    while (getline(inputStream, currentLine)) {
        stringstream ss(currentLine);
        vector<string> split = splitString(currentLine, " ");
        double vx = stod(split[0]);
        double vy = stod(split[1]);
        double vz = stod(split[2]);
        vector<double> prediction;
        prediction.reserve(split.size() - 4);
        for(int i = 4; i < split.size(); i++)
            prediction.push_back(stod(split[i]));
        points.emplace_back(vx, vy, vz);
        labels.push_back(prediction);
    }
    return make_pair(points, labels);
}

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

/*
 * Returns true if the file has hard labels associated to the points and false if it has features instead
 * */
bool hasLabels(const string &inFile)
{

    // Test if we have labels or rich features
    ifstream inputStream(inFile.c_str());
    if (!inputStream) {
        cerr << "Could not load file located at : " << inFile << endl;
        return 1;
    }
    string currentLine, tag;
    while(getline(inputStream, currentLine)) {
        stringstream ss(currentLine);
        ss >> tag;
        if (tag == "vla")
            break;
    }
    vector<string> firstLine = splitString(currentLine, " ");
    return (firstLine.size() == 2);

}

pair<vector<Point>, vector<int>> loadPointsWithLabel(const string &inFile) {
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
        } else if (firstCaracter == "vla") {
            ss >> idx;
            labels.push_back(idx);
        }
    }
    return make_pair(points, labels);
}

pair<vector<Point>, vector<Vector>> loadPointsWithNormals(const string &inFile)
{
    //Loading the obj data
    ifstream inputStream(inFile.c_str());
    if (!inputStream) {
        cerr << "Could not load file located at : " << inFile << endl;
        return make_pair(vector<Point>(), vector<Vector>());
    }
    vector<Point> points;
    vector<Vector> normals;
    string currentLine;
    while (getline(inputStream, currentLine)) {
        stringstream ss(currentLine);
        string firstCaracter;
        float vx, vy, vz, nx, ny, nz;
        ss >> firstCaracter;
        if (firstCaracter == "v") {
            ss >> vx >> vy >> vz;
            points.emplace_back(vx, vy, vz);
        }
        else if(firstCaracter == "vn") {
            ss >> nx >> ny >> nz;
            normals.emplace_back(nx, ny, nz);
        }
    }
    if(points.size() != normals.size())
        cerr << "Warning: Loaded " << points.size() << " points and " << normals.size() << " normals." << endl;
    return make_pair(points, normals);
}

tuple<vector<Point>, vector<Vector>, vector<vector<double>>> loadPointsWithNormalsAndRichFeatures(const string &inFile)
{
    //Loading the obj data
    ifstream inputStream(inFile.c_str());
    if (!inputStream) {
        cerr << "Could not load file located at : " << inFile << endl;
        return make_tuple(vector<Point>(), vector<Vector>(), vector<vector<double>>());
    }
    vector<Point> points;
    vector<Vector> normals;
    vector<vector<double>> richFeatures;
    string currentLine;
    while (getline(inputStream, currentLine)) {
        stringstream ss(currentLine);
        string firstCaracter;
        float vx, vy, vz, idx, nx, ny, nz;
        ss >> firstCaracter;
        if (firstCaracter == "v") {
            ss >> vx >> vy >> vz;
            points.emplace_back(vx, vy, vz);
        }
        else if(firstCaracter == "vn") {
            ss >> nx >> ny >> nz;
            normals.emplace_back(nx, ny, nz);
        }
        else if(firstCaracter == "vla") {

            vector<double> richFeature;
            vector<string> split = splitString(currentLine, " ");
            for(int i=1; i < split.size(); i++)
                if(!split[i].empty())
                    richFeature.push_back(stod(split[i]));
            richFeatures.push_back(richFeature);
        }
    }
    // Check that features are cogent
    if (!richFeatures.empty()) {
        size_t featuresSize = richFeatures[0].size();
        for(const auto& feature: richFeatures)
            if(feature.size() != featuresSize)
                cerr << "WARNING: the point features don't have a cogent size: " << featuresSize
                     << " vs " << feature.size() << endl;
    }
    return make_tuple(points, normals, richFeatures);
}

pair<vector<Point>, vector<vector<double>>> loadPointsWithRichFeatures(const string &inFile)
{
    //Loading the obj data
    ifstream inputStream(inFile.c_str());
    if (!inputStream) {
        cerr << "Could not load file located at : " << inFile << endl;
        return make_pair(vector<Point>(), vector<vector<double>>());
    }
    vector<Point> points;
    vector<vector<double>> richFeatures;
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

            vector<double> richFeature;
            vector<string> split = splitString(currentLine, " ");
            for(int i=1; i < split.size(); i++)
                if(!split[i].empty())
                    richFeature.push_back(stod(split[i]));
            richFeatures.push_back(richFeature);
        }
    }
    // Check that features are cogent
    if (!richFeatures.empty()) {
        size_t featuresSize = richFeatures[0].size();
        for(const auto& feature: richFeatures)
            if(feature.size() != featuresSize)
                cerr << "WARNING: the point features don't have a cogent size: " << featuresSize
                     << " vs " << feature.size() << endl;
    }
    return make_pair(points, richFeatures);
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
                if (classFound) break;
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

vector<Triangle> loadTrianglesFromPly(const string &inFile)
{
    unique_ptr<istream> file_stream;
    vector<uint8_t> byte_buffer;
    bool threeChannels = false;

    vector<float3> _points;
    vector<uint3> _faces;
    vector<Triangle> triangles;
    try
    {
        // Preload into memory
        byte_buffer = read_file_binary(inFile);
        file_stream = make_unique<memory_stream>((char*)byte_buffer.data(), byte_buffer.size());

        // Check if the stream was correctly open
        if (!file_stream || file_stream->fail()) throw std::runtime_error("file_stream failed to open " + inFile);

        // Set the cursor at the beginning of the file
        file_stream->seekg(0, std::ios::beg);

        // Parse the PLY file's header
        PlyFile file;
        file.parse_header(*file_stream);

        // Prepare the data containers
        shared_ptr<PlyData> vertices, faces, colors;

        // Requesting the relevant properties
        try { vertices = file.request_properties_from_element("vertex", { "x", "y", "z" }); }
        catch (const exception & e) { cerr << "tinyply exception: " << e.what() << endl; }
        try { faces = file.request_properties_from_element("face", { "vertex_indices" }, 3); }
        catch (const exception & e) {
            try { faces = file.request_properties_from_element("face", { "vertex_index" }, 3); }
            catch (const exception & e) {cerr << "tinyply exception: " << e.what() << endl;}
        }

        // Actual parsing
        file.read(*file_stream);

        // Process the vertices
        const size_t numVerticesBytes = vertices->buffer.size_bytes();
        _points = vector<float3>(vertices->count);
        memcpy(_points.data(), vertices->buffer.get(), numVerticesBytes);

        // Processing the faces
        const size_t numFacesBytes = faces->buffer.size_bytes();
        _faces = vector<uint3>(faces->count);
        memcpy(_faces.data(), faces->buffer.get(), numFacesBytes);
    }
    catch (const exception & e)
    {
        cerr << "Caught tinyply exception: " << e.what() << endl;
    }

    // Filling the triangles
    for(int i=0; i < _faces.size(); i++)
    {
        Point x(_points[_faces[i].x].x, _points[_faces[i].x].y, _points[_faces[i].x].z);
        Point y(_points[_faces[i].y].x, _points[_faces[i].y].y, _points[_faces[i].y].z);
        Point z(_points[_faces[i].z].x, _points[_faces[i].z].y, _points[_faces[i].z].z);
        triangles.emplace_back(x, y, z);
    }
    return triangles;
}

void savePointsAsObj(const vector<Point>& points, const string &outPath) {
    stringstream fileOut(outPath);
    for (const auto point: points)
        fileOut << "v " << point.x() << " " << point.y() << " " << point.z() << endl;
    ofstream realFileOut(outPath.c_str());
    realFileOut << fileOut.rdbuf();
    realFileOut.close();
}

void savePointsAsObjWithLabel(const pair<vector<Point>, map<Point, vector<double>>> &pointsWithLabel, const string &outPath,
                              const vector<Vector> &normals, const vector<double> &features) {
    stringstream fileOut(outPath);
    for(int i=0; i < pointsWithLabel.first.size(); i++)
    {
        auto& point = pointsWithLabel.first[i];
        fileOut << "v " << point.x() << " " << point.y() << " " << point.z() << endl;
        fileOut << "vla ";
        for(const auto& feat: pointsWithLabel.second.at(point))
            fileOut << feat << " ";
        fileOut << endl;
        if(!normals.empty())
            fileOut << "vn " << normals[i].x() << " " << normals[i].y() << " " << normals[i].z() << endl;
        if(!features.empty())
            fileOut << "vf " << features[i] << endl;
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
        if(goodKey.first == -1) continue; // Merged edge
        f.to_draw = true;

        // We find the dominating label of the edge feature
        int goodLabel = -1;
        double bestScore = -1;
        for(int i=0; i < classesWithColor.size() + 1; i++)
            if(edgeFeatures.at(goodKey)[0][i] > bestScore)
            {
                bestScore = edgeFeatures.at(goodKey)[0][i];
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
            vector<vector<double>> val = edgeFeat[0][1].get<vector<vector<double>>>();
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
            _nodePoints.push_back(point.get<Point>());
    }

    // Node Bboxes
    if (data.find("NodeBbox") != data.end()) {
        for(auto bbox: data["NodeBbox"])
            _nodeBboxes.emplace_back(bbox[0], bbox[1], bbox[2], bbox[3], bbox[4], bbox[5]);
    }

    // Node Volumes
    if (data.find("NodeVolumes") != data.end())
        _nodeVolumes = data["NodeVolumes"].get<vector<double>>();
    else
        _nodeVolumes = vector<double>(0);

    //Merged to nodes
    if (data.find("Merged2Node") != data.end())
        _merged2Nodes = data["Merged2Node"].get<vector<vector<int>>>();
    else
        _merged2Nodes = vector<vector<int>>(0);

    //Nodes to merged
    if (data.find("Node2Merged") != data.end())
        _nodes2merged = data["Node2Merged"].get<vector<int>>();
    else
        _nodes2merged = vector<int>(0);

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


void PlaneArrangement::saveAsJson(const string &outPath) {
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
    if(_nodePoints.empty())
        data["NodePoints"] = getCellsPoints(_cell2label, _arr);
    else
        data["NodePoints"] = _nodePoints;
    if(_nodeBboxes.empty())
        data["NodeBbox"] = getCellsBbox(_cell2label, _arr);
    else
        data["NodeBbox"] = _nodeBboxes;
    if(_nodeVolumes.empty())
        data["NodeVolumes"] = computeAllNodesVolumes();
    else
        data["NodeVolumes"] = _nodeVolumes;
    data["planes"] = _planes;
    data["bbox"] = _bbox;
    data["nbPlanes"] = _nbPlanes;
    vector<vector<double>> pointCloud(_points.size(), vector<double>(3));
    for(int i=0; i < _points.size(); i++)
        pointCloud[i] = {_points[i].x(), _points[i].y(), _points[i].z()};
    data["pointCloud"] = pointCloud;
    data["Merged2Node"] = _merged2Nodes;
    data["Node2Merged"] = _nodes2merged;

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

const std::map<int, int> &PlaneArrangement::label2cell() {
    if(_label2cell.size() != _cell2label.size())
        for(const auto mapIt: _cell2label)
            _label2cell[mapIt.second] = mapIt.first;
    return _label2cell;
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

const std::vector<Point> &PlaneArrangement::nodePoints() {
    if(!_nodes2merged.empty()) {
        static_cast<void>(arrangement());
        _nodePoints = vector<Point>(_merged2Nodes.size(), Point(0., 0., 0.));
        vector<int> nbPoints(_merged2Nodes.size(), 0);
        Epeck_to_Simple e2s;
        for (auto cellIt = _arr.cells_begin(); cellIt != _arr.cells_end(); cellIt++) {
            if (!_arr.is_cell_bounded(*cellIt)) continue;
            int mergedNodeIdx = _nodes2merged.at(_cell2label.at(_arr.cell_handle(*cellIt)));
            _nodePoints[mergedNodeIdx] += (e2s(cellIt->point()) - CGAL::ORIGIN);
            nbPoints[mergedNodeIdx]++;
        }
        for(int i=0; i < _nodePoints.size(); i++)
            _nodePoints[i] = CGAL::ORIGIN + (_nodePoints[i] - CGAL::ORIGIN) / nbPoints[i];
    }
    if(_nodePoints.empty()) {
        static_cast<void>(arrangement());
        Epeck_to_Simple e2s;
        _nodePoints = vector<Point>(_cell2label.size(), Point(0., 0., 0.));
        for (auto cellIt = _arr.cells_begin(); cellIt != _arr.cells_end(); cellIt++) {
            if (!_arr.is_cell_bounded(*cellIt)) continue;

            _nodePoints[_cell2label.at(_arr.cell_handle(*cellIt))] = e2s(cellIt->point());
        }
    }
    return _nodePoints;
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


vector<Point> getCellsPoints(const map<int, int> &cell2label, const Arrangement &arr) {
    vector<Point> cellsPoints(cell2label.size(), Point(0., 0., 0.));
    Epeck_to_Simple e2s;

    for(auto cellIt = arr.cells_begin(); cellIt != arr.cells_end(); cellIt++) {
        if(!arr.is_cell_bounded(*cellIt)) continue;

        cellsPoints[cell2label.at(arr.cell_handle(*cellIt))] = e2s(cellIt->point());
    }

    return cellsPoints;
}

vector<vector<double>> getCellsBbox(const map<int, int> &cell2label, const Arrangement &arr) {
    vector<double> initialBbox = {DBL_MAX, DBL_MAX, DBL_MAX, -DBL_MAX, -DBL_MAX, -DBL_MAX};
    vector<vector<double>> cellsPoints(cell2label.size(), initialBbox);
    Epeck_to_Simple e2s;

    for(auto cellIt = arr.cells_begin(); cellIt != arr.cells_end(); cellIt++) {
        if(!arr.is_cell_bounded(*cellIt)) continue;
        vector<double> &curBbox = cellsPoints[cell2label.at(arr.cell_handle(*cellIt))];
        for(auto facetIt = cellIt->subfaces_begin(); facetIt != cellIt->subfaces_end(); facetIt++)
        {
            auto facet = arr.facet(*facetIt);
            for(auto edgeIt = facet.subfaces_begin(); edgeIt != facet.subfaces_end(); edgeIt++) {
                auto edge = arr.edge(*edgeIt);
                for(auto pointIt = edge.subfaces_begin(); pointIt != edge.subfaces_end(); pointIt++)
                {
                    Point curPoint = e2s(arr.point(*pointIt));
                    curBbox[0] = min(curBbox[0], curPoint.x());
                    curBbox[1] = min(curBbox[1], curPoint.y());
                    curBbox[2] = min(curBbox[2], curPoint.z());
                    curBbox[3] = max(curBbox[3], curPoint.x());
                    curBbox[4] = max(curBbox[4], curPoint.y());
                    curBbox[5] = max(curBbox[5], curPoint.z());
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

const std::vector<std::vector<int>> &PlaneArrangement::merged2Nodes() const
{
    return _merged2Nodes;
}

const std::vector<int> &PlaneArrangement::nodes2Merged() const
{
    return _nodes2merged;
}

std::map<int, int> PlaneArrangement::mergedMapping()
{
    if(!_nodes2merged.empty()) {
        map<int, int> correctMapping;
        // If the nodes have been merged, we need to compose cell2label and nodes2merged
        static_cast<void>(arrangement()); // Make sure that the arrangement has been built
        for (auto cellIt = _arr.cells_begin(); cellIt != _arr.cells_end(); cellIt++) {
            if (!_arr.is_cell_bounded(*cellIt)) continue;
            int cellLabel = _arr.cell_handle(*cellIt);
            correctMapping[cellLabel] = _nodes2merged[_cell2label.at(cellLabel)];
        }
        return correctMapping;
    }
    return _cell2label;
}

double PlaneArrangement::computeNodeVolume(const Arrangement::Face_handle &cellHandle) const
{
    // Gathering the vertex of the current cell
    Epeck_to_Epick e2i;
    double volume = 0.;
    auto cell = _arr.cell(cellHandle);
    set<Kernel3::Point_3> pointSet;
    for(auto facetIt = cell.subfaces_begin(); facetIt != cell.subfaces_end(); facetIt++)
    {
        const auto &facet = _arr.facet(*facetIt);
        for(auto edgeIt = facet.subfaces_begin(); edgeIt != facet.subfaces_end(); edgeIt++)
        {
            const auto &edge = _arr.edge(*edgeIt);
            for(auto vertexIt = edge.subfaces_begin(); vertexIt != edge.subfaces_end(); vertexIt++)
                pointSet.insert(e2i(_arr.vertex(*vertexIt).point()));
        }
    }

    // Triangulating and adding the volumes of the tetrahedrons
    Triangulation tri(pointSet.begin(), pointSet.end());
    for(const auto& tetra: tri.finite_cell_handles())
        volume += CGAL::to_double(CGAL::volume((tetra->vertex(0))->point(), (tetra->vertex(1))->point(),
                                               (tetra->vertex(2))->point(), (tetra->vertex(3))->point()));

    return volume;
}

vector<double> PlaneArrangement::computeAllNodesVolumes()
{
    if(!isArrangementComputed)
        static_cast<void>(arrangement());
    vector<double> nodeVolumes(cell2label().size());
    for(auto cellIt = _arr.cells_begin(); cellIt != _arr.cells_end(); cellIt++)
    {
        if(!_arr.is_cell_bounded(*cellIt)) continue;
        auto cellHandle = _arr.cell_handle(*cellIt);
        nodeVolumes[cell2label().at(cellHandle)] = computeNodeVolume(cellHandle);
    }
    return nodeVolumes;
}

vector<UniqueEdges> PlaneArrangement::euclidianNeighbourhoods(const vector<double> &maxDistances)
{
    vector<UniqueEdges> neighbourhoods(maxDistances.size());
    // We transform the distances to avoid using square roots
    vector<double> squareDistances;
    transform(maxDistances.begin(), maxDistances.end(), back_inserter(squareDistances), [](double v) {return v*v;});

    // idx2point and point2idx
    const vector<Point> &nodesPts = nodePoints();
    map<Point, int> point2idx;
    for(int i=0; i < nodesPts.size(); i++)
        point2idx[nodesPts[i]] = i;

    // We build the neighbourhoods using an incremental kd tree
#pragma omp parallel for
    for(int i=0; i < nodesPts.size(); i++)
    {
        incrementalKdTree tree(nodesPts.begin(), nodesPts.end());
        const auto& point = nodesPts[i];
        NN_incremental_search NN(tree, point);
        auto incrementalIt = NN.begin();
        incrementalIt++; // We ignore the current point
        size_t distanceIdx = 0;
        while(incrementalIt != NN.end())
        {
            auto curElem = incrementalIt++;
            if(curElem->second >= squareDistances[distanceIdx])
                distanceIdx++;
            if(distanceIdx >= maxDistances.size()) break;
            int j = point2idx[curElem->first];
#pragma omp critical
            neighbourhoods[distanceIdx].insert(pair<int, int>(min(i, j), max(i, j)));
        }
    }

    return neighbourhoods;
}

vector<double> getThicknessFeatures(const vector<Point> &points, const vector<Vector> &normals, double threshold)
{
    vector<double> features(points.size());

    // Build a point 2 normal map
    map<Point, Vector> point2Normal;
    for(int i=0; i < points.size(); i++)
        point2Normal[points[i]] = normals[i];

    // Build a kd tree
    incrementalKdTree tree(points.begin(), points.end());

    int count = 0;
#pragma omp parallel for
    for(int i=0; i < points.size(); i++) {
        const auto& point = points[i];
        const auto& normal = normals[i];

        NN_incremental_search NN(tree, point);
        auto incrementalIt = NN.begin();
        incrementalIt++; // We ignore the current point

        bool criteriaMet = false;
        double feature = 1.;
        while (!criteriaMet && incrementalIt != NN.end()) {
            auto curElem = incrementalIt++;
            if(curElem->second >= 1.) break;
            const auto& neighbourPoint = curElem->first;
            const auto& neighbourNormal = point2Normal[neighbourPoint];

            // Criteria one: opposite normal directions
            double dotProd = normal * neighbourNormal;
            if(dotProd >= threshold) continue;

            // Criteria two: normals are not facing each other
            bool arePointsFacing = (neighbourPoint - point) * normal > 0.;
            if(arePointsFacing) continue;

            criteriaMet = true;
            // Distance from point to the intersection of
            // line (point, normal) and plane (neighbourPoint, neighbourNormal)
            double dist = abs(((neighbourPoint - point) * neighbourNormal) / (normal * neighbourNormal));
            feature = min(feature, dist);
        }
#pragma omp atomic
        count++;
#pragma omp critical
{
        if(count % 1000 == 0) cout << "Done " << 100.*double(count) / points.size() << "%" << endl;
        features[i] = feature;
}
    }

    return features;
}
