#ifndef BIM_DATA_IOGEOMETRY_H
#define BIM_DATA_IOGEOMETRY_H

//STD
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <list>
#include <tuple>
#include <unordered_map>

// CGAL
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_integer.h>
//#include <CGAL/Polyhedron_3.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/Surface_mesh.h>

// Boost
#include <boost/functional/hash.hpp>

// External
#include "json/json.hpp"

//typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Homogeneous<CGAL::Exact_integer>  Kernel;

/* Typedefs for the geometrical primitives */
typedef typename Kernel::Point_3 Point;
typedef typename Kernel::Segment_3 Segment;
typedef typename Kernel::Triangle_3 Triangle;
typedef typename Kernel::Vector_3 Vector;
typedef typename Kernel::Ray_3 Ray;

/* Typedefs for polyhedron meshes */
typedef CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_with_id_3> Polyhedron;
//typedef CGAL::Surface_mesh<Point> Polyhedron;
typedef CGAL::Nef_polyhedron_3<Kernel> Nef_Polyhedron;
typedef Polyhedron::Halfedge_handle    Halfedge_handle;
typedef Polyhedron::Facet_handle       Facet_handle;
typedef Polyhedron::Vertex_handle      Vertex_handle;


// Hash for Triangle
namespace std
{
    // Turning Triangle into tuple
    template<>
    struct hash<Triangle>
    {
        size_t operator()(Triangle const& arg) const noexcept
        {
            std::tuple<double, double, double, double, double, double, double, double, double>
                    triangle(CGAL::to_double(arg[0].x()), CGAL::to_double(arg[0].y()), CGAL::to_double(arg[0].z()),
                             CGAL::to_double(arg[1].x()), CGAL::to_double(arg[1].y()), CGAL::to_double(arg[1].z()),
                             CGAL::to_double(arg[2].x()), CGAL::to_double(arg[2].y()), CGAL::to_double(arg[2].z()));
            return boost::hash_value(triangle);
        }
    };
}

// Typedefs for triangles and colors
typedef std::tuple<int, int, int> colorTuple;
typedef std::unordered_map<Triangle, colorTuple> TriangleColorMap;

// Typedef for storing semantic classes (name, keywords, color)
typedef std::tuple<std::string, std::vector<std::string>, colorTuple> classKeywordsColor;

// Input functions
std::pair<std::vector<Triangle>, TriangleColorMap> loadTrianglesFromObj(const std::string &objFile,
                                                                 const std::vector<classKeywordsColor> &classes);
std::vector<Point> loadPointOfViews(const std::string &jsonFile);

// Output functions
void savePointsAsObj(std::vector<Point> points, const std::string &outPath);
void savePointsAsObjWithColors(std::vector<Point> points, std::vector<colorTuple> colors, const std::string &outPath);
void saveTrianglesAsObj(std::vector<Triangle> triangles, const std::string &outPath, TriangleColorMap colors);
void saveSeparatedObj(std::vector<Triangle> triangles, const std::string &outPath, TriangleColorMap colors);

// Semantics
std::vector<classKeywordsColor> loadSemanticClasses(std::string path);

#endif
