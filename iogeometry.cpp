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

vector<classKeywordsColor> loadSemanticClasses(string path)
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
