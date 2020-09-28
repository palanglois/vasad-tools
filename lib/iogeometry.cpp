#include "iogeometry.h"

using namespace std;
using Json = nlohmann::json;

/* Load points from an obj files */
pair<vector<Triangle>, TriangleColorMap>
loadTrianglesFromObj(const string &objFile, const vector<classKeywordsColor> &classes) {
    vector<Triangle> triangles;
    vector<Point> points;
    vector<vector<int>> faces;


    //Loading the obj data
    ifstream inputStream(objFile.c_str());
    if (!inputStream) {
        cerr << "Could not load file located at : " << objFile << endl;
        return pair<vector<Triangle>, TriangleColorMap>();
    }

    //Loading the triangles
    string currentLine;
    colorTuple cur_class_color(0, 0, 0);
    vector<colorTuple> triClasses;
    TriangleColorMap triangleToColors;
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
            faces.emplace_back(curFaceIdx);
            triClasses.push_back(cur_class_color);
        } else if (firstCaracter == "o") {
            // Object - Finding the corresponding class
            ss >> obj_name;
            cur_class_color = colorTuple(0, 0, 0);
            for (auto cl: classes) {
                bool classFound = false;
                for (auto keyword: get<1>(cl)) {
                    if (obj_name.find(keyword) != string::npos) {
                        cur_class_color = get<2>(cl);
                        classFound = true;
                        break;
                    }
                }
                if (classFound) break;
            }
        }
    }

    for (int i = 0; i < faces.size(); i++) {
        Triangle cur_triangle = Triangle(points[faces[i][0]], points[faces[i][1]], points[faces[i][2]]);
        triangles.push_back(cur_triangle);
        triangleToColors[cur_triangle] = triClasses[i];
    }

    return pair<vector<Triangle>, TriangleColorMap>(triangles, triangleToColors);
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

vector<pair<Tree*, int>> loadTreesFromObj(const string &inFile, const vector<classKeywordsColor> &classes)
{
    vector<pair<Tree*, int>> allTrees;

    //Loading the obj data
    ifstream inputStream(inFile.c_str());
    if (!inputStream) {
        cerr << "Could not load file located at : " << inFile << endl;
        return allTrees;
    }

    //Loading the triangles
    string currentLine;
    int cur_class_id(0);
    vector<colorTuple> triClasses;
    TriangleColorMap triangleToColors;
    vector<Point> points;
    vector<vector<int>> faces;
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
            for (int i=0; i < classes.size(); i++) {
                auto &cl = classes[i];
                bool classFound = false;
                for (auto &keyword: get<1>(cl)) {
                    if (obj_name.find(keyword) != string::npos) {
                        // Class has been found
                        classFound = true;
                        // Make tree
                        vector<Triangle> triangles;
                        for(auto &triIdx: faces) {
                            auto curTriangle = new Triangle(points[triIdx[0]], points[triIdx[1]], points[triIdx[2]]);

                            if(!Kernel().is_degenerate_3_object()(*curTriangle))
                                triangles.push_back(*curTriangle);
                        }
                        if(!triangles.empty()) {
                            Tree *curTree = new Tree();
                            // Weird stuff I need to do to make it work...
                            for(auto triPtr: triangles) {
                                vector<Triangle> triVec = {triPtr};
                                curTree->insert(AABB_triangle_traits::Primitive(triVec.begin()));
                            }
                            allTrees.emplace_back(curTree, i);
                        }
                        //Empty the faces
                        faces = vector<vector<int>>();
                        break;
                    }
                }
                if (classFound) break;
            }
        }
    }

    return allTrees;
}

void savePointsAsObj(vector<Point> points, const string &outPath) {
    ofstream fileOut(outPath);
    for (const auto point: points)
        fileOut << "v " << point.x() << " " << point.y() << " " << point.z() << endl;
    fileOut.close();
}

void savePointsAsObjWithColors(vector<Point> points, vector<colorTuple> colors, const string &outPath) {
    ofstream fileOut(outPath);
    for (int i = 0; i < points.size(); i++) {
        const Point &point = points[i];
        const colorTuple &color = colors[i];
        fileOut << "v " << point.x() << " " << point.y() << " " << point.z()
                << " " << get<0>(color) << " " << get<1>(color) << " " << get<2>(color) << endl;
    }
    fileOut.close();
}

void saveTrianglesAsObj(vector<Triangle> triangles, const string &outPath, TriangleColorMap colors) {
    ofstream fileOut(outPath);
    for (auto triangle : triangles) {
        colorTuple color = colors[triangle];
        for (int i = 0; i < 3; i++)
            fileOut << "v " << triangle[i].x() << " " << triangle[i].y() << " " << triangle[i].z() << " "
                    << get<0>(color) << " " << get<1>(color) << " " << get<2>(color) << endl;
    }
    for (int i = 0; i < triangles.size(); i++)
        fileOut << "f " << 3 * i + 1 << " " << 3 * i + 2 << " " << 3 * i + 3 << endl;
    fileOut.close();
}

void saveSeparatedObj(vector<Triangle> triangles, const string &outPath, TriangleColorMap colors) {
    ofstream fileOut(outPath + to_string(0) + ".obj");
    int fileIdx(1);
    colorTuple curColor = colors[triangles[0]];
    int curTriangleIndex = 0;
    for (auto triangle : triangles) {
        colorTuple color = colors[triangle];
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

void loadArrangement(const string &name, Arrangement &arr, map<int, int> &cell2label, vector<bool> &labels, CGAL::Bbox_3 &bbox)
{
    cout << "Loading arrangement!" << endl;
    fstream i(name);
    Json data;
    i >> data;

    // Bounding box
    vector<double> box = data["bbox"];
    bbox = CGAL::Bbox_3(box[0], box[1], box[2], box[3], box[4], box[5]);
    arr.set_bbox(bbox);

    // Planes
    vector<Json> planes = data["planes"];
#ifndef NDEBUG
    int nbPlanes = min(data["nbPlanes"].get<int>(), 30);
#else
    int nbPlanes = data["nbPlanes"].get<int>();
#endif
    //DEBUG
    int nbPlanesUsed = 0;
    // END DEBUG
    for(int i=0; i < nbPlanes; i++)
    {
        Json &norm = planes[i]["normal"];
        Kernel2::Vector_3 normal((double) norm[0], (double) norm[1], (double) norm[2]);
        Json &inl = planes[i]["inlier"];
        Kernel2::Point_3 inlier((double) inl[0], (double) inl[1], (double) inl[2]);
        arr.insert(Kernel2::Plane_3(inlier, normal));
        //DEBUG
        nbPlanesUsed++;
        //END DEBUG
        cout << "Inserted plane " << i + 1 << " out of " << nbPlanes << endl;
    }
    //DEBUG
    cout << "Nb of planes used " << nbPlanesUsed << endl;
    //END DEBUG

    //Mapping
    for(auto &elem: data["map"])
        cell2label[stoi(elem[0].get<string>())] = elem[1].get<int>();

    //Labels
    labels = data["labels"].get<vector<bool>>();
    cout << "Arrangement loaded" << endl;
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
