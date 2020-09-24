#include "BimObj.h"

using namespace std;

void BimObj::loadFromObj(string path, const vector<classKeywordsColor> &classes) {
    //Loading the obj data
    ifstream inputStream(path.c_str());
    if (!inputStream) {
        cerr << "Could not load file located at : " << path << endl;
    }

    //Loading the triangles
    string currentLine;
    string curObject;
    colorTuple cur_class_color(0, 0, 0);
    vector<Point> curPoints;
    string curClassName;
    vector<vector<size_t>> curPolygons;
    vector<vector<int>> triClasses;
    map<string, bool> objAndCloseness;
    int nbClosed = 0;
    int nbClosedAfter = 0;
    int nbTotal = 0;
    while(getline(inputStream, currentLine))
    {
        stringstream ss(currentLine);
        string firstCaracter;
        float vx, vy, vz;
        string f1, f2, f3;
        ss >> firstCaracter;
        if(firstCaracter == "v") {
            // Vertice
            ss >> vx >> vy >> vz;
            curPoints.emplace_back(vx, vy, vz);
        }
        else if (firstCaracter == "f") {
            // Face
            ss >> f1 >> f2 >> f3;
            vector<string> curFace = {f1, f2, f3};
            vector<size_t> curFaceIdx;
            for(const auto &idxStr: curFace)
                curFaceIdx.push_back(stoul(idxStr.substr(0, idxStr.find("/"))) - 1);
            curPolygons.emplace_back(curFaceIdx);
        }
        else if (firstCaracter == "o"){
            // New object - Check the previous one and reinit the variables
            if(curObject.empty()) {
                ss >> curObject ;
                cur_class_color = colorTuple(0, 0, 0);
                bool classFound = false;
                for (auto cl: classes) {
                    for (auto keyword: get<1>(cl)) {
                        if (curObject.find(keyword) != string::npos) {
                            cur_class_color = get<2>(cl);
                            curClassName = get<0>(cl);
                            classFound = true;
                            break;
                        }
                    }
                    if (classFound) break;
                }
                if(!classFound)
                {
                    curClassName = "Other";
                }
                continue;
            }
            if (curObject.find("Flow") == string::npos /*&& curObject.find("Seam") != string::npos*/)
            {
                // Check that current object is closed
                auto pointCopies = curPoints;
                CGAL::Polygon_mesh_processing::orient_polygon_soup(pointCopies, curPolygons);
//                Polyhedron mesh;
                SemPolyhedron mesh(curObject, curClassName, cur_class_color);
                CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(pointCopies, curPolygons, *mesh.toCgal());
                bool closedBefore = CGAL::is_closed(mesh);
                nbTotal++;
                nbClosed += (int) closedBefore;
                nbClosedAfter += (int) closedBefore;
                objAndCloseness[curObject] = closedBefore;
                // Otherwise try to repair it
                if (!closedBefore) {
                    try {
                        repairMesh(mesh); // Hole filling
                    }
                    catch (CGAL::Failure_exception e) {
                        cout << "CGAL exception:\n library: " << e.library()
                             << "\n expression:" << e.expression()
                             << "\n filename:" << e.filename()
                             << "\n line_number:" << e.line_number()
                             << "\n message:" << e.message();
                    }
                    bool closedAfter = CGAL::is_closed(mesh);
                    nbClosedAfter += (int) closedAfter;
                    replace(curObject.begin(), curObject.end(), '/', '_');
                    cout << "Surface " << curObject << " is now "
                         << (closedAfter ? "\x1B[32mclosed\033[0m\t\t" : "\x1B[31mopen\033[0m\t\t") << endl;
//                    if(!closedAfter) {
//                        writePolyhedron(mesh, outPath + curObject + ".obj");
//                        CGAL::draw(mesh);
//                    }
                }
                allMeshes.push_back(mesh);
            }
            ss >> curObject ;
            curPolygons = vector<vector<size_t>>();
        }
    }
    cout << "Before repairing: " << nbClosed << " closed mesh out of " << nbTotal << endl;
    cout << "After repairing: " << nbClosedAfter << " closed mesh out of " << nbTotal << endl;

}

SemPolyhedron::SemPolyhedron(std::string _name, std::string _category, colorTuple _color) :
name(_name), category(_category), color(_color), Polyhedron()

{

}

bool SemPolyhedron::isClosed() const
{
    return CGAL::is_closed(*this);
}

string SemPolyhedron::getName() const
{
    return name;
}

SemPolyhedron::Base* SemPolyhedron::toCgal()
{
    return (Base*)(this);
}

vector<SemPolyhedron>::iterator BimObj::begin()
{
    return allMeshes.begin();
}

vector<SemPolyhedron>::iterator BimObj::end()
{
    return allMeshes.end();
}
