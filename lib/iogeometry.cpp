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
            bool classFound = false;
            for (auto cl: classes) {
                for (auto keyword: get<1>(cl)) {
                    if (obj_name.find(keyword) != string::npos) {
                        cur_class_color = get<2>(cl);
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
    TriangleColorMap triangleToColors;
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

void saveTrianglesAsObj(const vector<Triangle>& triangles, const string &outPath, TriangleColorMap colors) {
    stringstream fileOut;
    for (auto triangle : triangles) {
        colorTuple color = colors[triangle];
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

void loadArrangement(const string &name, Arrangement &arr, map<int, int> &cell2label, vector<int> &gtLabels,
        vector<bool> &labels, CGAL::Bbox_3 &bbox)
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
    auto tqNbPlanes = tq::trange(nbPlanes);
    tqNbPlanes.set_prefix("Inserting " + to_string(nbPlanes) + " planes: ");
    for(int i: tqNbPlanes)
    {
        Json &norm = planes[i]["normal"];
        Kernel2::Vector_3 normal((double) norm[0], (double) norm[1], (double) norm[2]);
        Json &inl = planes[i]["inlier"];
        Kernel2::Point_3 inlier((double) inl[0], (double) inl[1], (double) inl[2]);
        arr.insert(Kernel2::Plane_3(inlier, normal));
        //DEBUG
        nbPlanesUsed++;
        //END DEBUG
    }
    //DEBUG
    cout << "Nb of planes used " << nbPlanesUsed << endl;
    //END DEBUG

    //Mapping and labels
    if (data["map"].find("NOMAP") != data["map"].end())
    {
        // If the mapping does not exist, we create it
        int cellIter = 0;
        for(auto cellIt = arr.cells_begin(); cellIt != arr.cells_end(); cellIt++)
            if(arr.is_cell_bounded(*cellIt))
                cell2label[arr.cell_handle(*cellIt)] = cellIter++;

        //Labels
        labels = vector<bool>(cell2label.size(), false);
    }
    else {
        // If the mapping exists we load it
        for (auto elem = data["map"].begin(); elem != data["map"].end(); elem++)
            cell2label[stoi(elem.key())] = elem.value().get<int>();

        //Labels
        if (data.find("labels") != data.end())
            labels = data["labels"].get<vector<bool>>();

        //gtLabels
        if (data.find("gtLabels") != data.end())
            gtLabels = data["gtLabels"].get<vector<int>>();
    }

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

