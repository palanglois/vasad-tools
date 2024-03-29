#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <map>
#include <algorithm>
#include <set>

// External
#include "OptionParser/option_parser.h"

//#include "ui_ImageInterface.h"
// CGAL
//#define CGAL_USE_BASIC_VIEWER
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/IO/print_wavefront.h>
#include <CGAL/draw_polyhedron.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>


using namespace std;


//typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;

/* Typedefs for the geometrical primitives */
typedef typename Kernel::Point_3 Point;
typedef typename Kernel::Triangle_3 Triangle;
typedef CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_with_id_3> Polyhedron;


typedef Polyhedron::Halfedge_handle    Halfedge_handle;
typedef Polyhedron::Facet_handle       Facet_handle;
typedef Polyhedron::Vertex_handle      Vertex_handle;

// Write a polyhedron to an obj file
void writePolyhedron(Polyhedron mesh, const string& outPath)
{

    // Gather useful idx
    vector<Point> usefulPoints;
    vector<vector<int>> faces;
    int verticeIdx = 1;
    for(auto faces_it = mesh.facets_begin(); faces_it != mesh.facets_end(); faces_it++)
    {
        vector<int> curFace;
        auto pHalfedge = faces_it->facet_begin();
        do {
            Point curPoint = pHalfedge->vertex()->point();
            auto curPointIt = find(usefulPoints.begin(), usefulPoints.end(), curPoint);
            if(!(curPointIt != usefulPoints.end())) {
                usefulPoints.push_back(pHalfedge->vertex()->point());
                curFace.push_back(verticeIdx);
                verticeIdx++;
            }
            else
            {
                int pointIdx = curPointIt - usefulPoints.begin() + 1;
                curFace.push_back(pointIdx);
            }
        }
        while(++pHalfedge != faces_it->facet_begin());
        faces.push_back(curFace);
    }

    // Write vertice
    ofstream file_out(outPath);
    for(auto point: usefulPoints)
    {
        file_out << "v " << point.x() << " " << point.y()
                 << " " << point.z() << endl;
    }

    // Write faces
    for(auto face: faces) {
        file_out << "f ";
        for (auto vertIdx: face)
            file_out << vertIdx << " ";
        file_out << endl;
    }
    file_out.close();
}

void repairMesh(Polyhedron &mesh, bool verbose=false)
{
    // Incrementally fill the holes
    unsigned int nb_holes = 0;
    for(Halfedge_handle h : halfedges(mesh))
    {
        if(h->is_border())
        {
            vector<Facet_handle>  patch_facets;
            vector<Vertex_handle> patch_vertices;
            bool success = get<0>(
                    CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(
                            mesh,
                            h,
                            back_inserter(patch_facets),
                            back_inserter(patch_vertices),
                            CGAL::Polygon_mesh_processing::parameters::vertex_point_map(get(CGAL::vertex_point, mesh)).
                                    geom_traits(Kernel())));
            if (verbose) {
                cout << " Number of facets in constructed patch: " << patch_facets.size() << endl;
                cout << " Number of vertices in constructed patch: " << patch_vertices.size() << endl;
                cout << " Fairing : " << (success ? "succeeded" : "failed") << endl;
                ++nb_holes;
            }
        }
    }
    if(verbose) {
        cout << endl;
        cout << nb_holes << " holes have been filled" << endl;
    }
}

/* Load points from an obj file */
map<string, bool> loadObjAndCheckCloseness(const string &objFile, const string &outPath)
{
    //Loading the obj data
    ifstream inputStream(objFile.c_str());
    if(!inputStream)
    {
        cerr << "Could not load file located at : " << objFile << endl;
        return map<string, bool>();
    }

    //Loading the triangles
    string currentLine;
    string curObject;
    vector<Point> curPoints;
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
                continue;
            }
            if (curObject.find("Flow") == string::npos /*&& curObject.find("Seam") != string::npos*/)
            {
                // Check that current object is closed
                auto pointCopies = curPoints;
                CGAL::Polygon_mesh_processing::orient_polygon_soup(pointCopies, curPolygons);
                Polyhedron mesh;
                CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(pointCopies, curPolygons, mesh);
                bool closedBefore = CGAL::is_closed(mesh);
                nbTotal++;
                nbClosed += (int)closedBefore;
                nbClosedAfter += (int)closedBefore;
                objAndCloseness[curObject] = closedBefore;
                // Otherwise try to repair it
                if(!closedBefore) {
                    repairMesh(mesh); // Hole filling
                    bool closedAfter = CGAL::is_closed(mesh);
                    nbClosedAfter += (int)closedAfter;
                    replace( curObject.begin(), curObject.end(), '/', '_');
                    cout << "Surface " << curObject << " is now "
                         << (closedAfter ? "\x1B[32mclosed\033[0m\t\t" : "\x1B[31mopen\033[0m\t\t") << endl;
                    if(!closedAfter) {
                        writePolyhedron(mesh, outPath + curObject + ".obj");
                        //CGAL::draw(mesh);
                    }
                }
            }
            ss >> curObject ;
            curPolygons = vector<vector<size_t>>();
        }
    }
    cout << "Before repairing: " << nbClosed << " closed mesh out of " << nbTotal << endl;
    cout << "After repairing: " << nbClosedAfter << " closed mesh out of " << nbTotal << endl;
    return objAndCloseness;
}

int main(int argc, char *argv[])
{
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input obj mesh", "");
    opt.add_option("-o", "--output", "Path to the output directory", ".");

    //Parsing options
    bool correctParsing = opt.parse_options(argc, argv);
    if (!correctParsing)
        return EXIT_FAILURE;

    if (op::str2bool(opt["-h"])) {
        opt.show_help();
        return 0;
    }

    string outPath = opt["-o"][opt["-o"].size()-1] == '/' ? opt["-o"] : opt["-o"] + "/";

    const string inputPath = opt["-i"];

    auto objAndCloseness = loadObjAndCheckCloseness(inputPath, outPath);
    return 0;
}
