#include "iogeometry.h"

using namespace std;
using Json = nlohmann::json;

/* Load points from an obj files */
pair<vector<Triangle>, TriangleColorMap> loadTrianglesFromObj(const string &objFile, const vector<classKeywordsColor>& classes)
{
    vector<Triangle> triangles;
    vector<Point> points;
    vector<vector<int>> faces;


    //Loading the obj data
    ifstream inputStream(objFile.c_str());
    if(!inputStream)
    {
        cerr << "Could not load file located at : " << objFile << endl;
        return pair<vector<Triangle>, TriangleColorMap>();
    }

    //Loading the triangles
    string currentLine;
    vector<int> cur_class_color = {0, 0, 0};
    vector<vector<int>> triClasses;
    TriangleColorMap triangleToColors;
    while(getline(inputStream, currentLine))
    {
        stringstream ss(currentLine);
        string firstCaracter;
        float vx, vy, vz;
        string f1, f2, f3, obj_name;
        ss >> firstCaracter;
        if(firstCaracter == "v") {
            // Vertice
            ss >> vx >> vy >> vz;
            points.emplace_back(vx, vy, vz);
        }
        else if (firstCaracter == "f") {
            // Face
            ss >> f1 >> f2 >> f3;
            vector<string> curFace = {f1, f2, f3};
            vector<int> curFaceIdx;
            for(auto idxStr: curFace)
                curFaceIdx.push_back(stoi(idxStr.substr(0, idxStr.find("/"))) - 1);
            faces.emplace_back(curFaceIdx);
            triClasses.push_back(cur_class_color);
        }
        else if (firstCaracter == "o"){
            // Object - Finding the corresponding class
            ss >> obj_name ;
            cur_class_color = {0, 0, 0};
            for(auto cl: classes) {
                bool classFound = false;
                for(auto keyword: get<1>(cl)) {
                    if (obj_name.find(keyword) != string::npos) {
                        cur_class_color = get<2>(cl);
                        classFound = true;
                        break;
                    }
                }
                if(classFound) break;
            }
        }
    }

    for(int i=0; i < faces.size(); i++) {
        Triangle cur_triangle = Triangle(points[faces[i][0]], points[faces[i][1]], points[faces[i][2]]);
        triangles.push_back(cur_triangle);
        triangleToColors[cur_triangle] = colorTuple(triClasses[i][0], triClasses[i][1], triClasses[i][2]);
    }

    return pair<vector<Triangle>, TriangleColorMap>(triangles, triangleToColors);
}

vector<Point> loadPointOfViews(const string &jsonFile)
{
    vector<Point> pointOfViews;

    //Loading json data
    Json inputData;
    ifstream inputStream(jsonFile.c_str());
    if(!inputStream)
    {
        cerr << "Could not load file located at : " << jsonFile << endl;
        return vector<Point>();
    }
    inputStream >> inputData;

    //Loading point of views
    Json pointOfViewArray = inputData.at("cam_loc");
    for (const auto &povJson : pointOfViewArray)
        pointOfViews.emplace_back(povJson[0], povJson[1], povJson[2]);

    return pointOfViews;
}

void savePointsAsObj(vector<Point> points, const string &outPath) {
    ofstream fileOut(outPath);
    for(const auto point: points)
        fileOut << "v " << point.x() << " " << point.y() << " " << point.z() << endl;
    fileOut.close();
}

void savePointsAsObjWithColors(vector<Point> points, vector<colorTuple> colors, const string &outPath) {
    ofstream fileOut(outPath);
    for(int i=0; i< points.size(); i++) {
        const Point& point = points[i];
        const colorTuple& color = colors[i];
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
    for(int i=0; i < triangles.size(); i++)
        fileOut << "f " << 3 * i + 1 << " " << 3 * i + 2 << " " << 3 * i + 3 << endl;
    fileOut.close();
}
