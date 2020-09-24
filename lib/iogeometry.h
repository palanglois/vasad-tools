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
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_integer.h>
//#include <CGAL/Polyhedron_3.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

// Boost
#include <boost/functional/hash.hpp>

//Polyhedral complex
#include <Polyhedral_complex_3/Arrangement_3.hpp>
#include <Polyhedral_complex_3/Mesh_3.hpp>
#include <Polyhedral_complex_3/Mesh_extractor_3.hpp>
#include <Polyhedral_complex_3/print_PLY.hpp>
#include "Polyhedral_complex_3/Polyhedral_complex_queries_3.hpp"

// External
#include "json/json.hpp"

typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel2;
//typedef CGAL::Homogeneous<CGAL::Exact_integer>  Kernel;
typedef CGAL::Cartesian_converter<Kernel,Kernel2>         Simple_to_Epeck;

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

/* Typedef for AABBTree */
typedef std::vector<Triangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<Kernel, Iterator> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;
typedef boost::optional< Tree::Intersection_and_primitive_id<Segment>::Type > Segment_intersection;
typedef boost::optional<Tree::Intersection_and_primitive_id<Ray>::Type> Ray_intersection;

/* Typedef for plane arrangement */
typedef Polyhedral_complex_3::Arrangement_3<Kernel2> Arrangement;


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
std::vector<Point> loadPointCloudObj(const std::string &inFile);

// Output functions
void savePointsAsObj(std::vector<Point> points, const std::string &outPath);
void savePointsAsObjWithColors(std::vector<Point> points, std::vector<colorTuple> colors, const std::string &outPath);
void saveTrianglesAsObj(std::vector<Triangle> triangles, const std::string &outPath, TriangleColorMap colors);
void saveSeparatedObj(std::vector<Triangle> triangles, const std::string &outPath, TriangleColorMap colors);
void saveArrangement(const std::string &name, const std::vector<Kernel::Plane_3> &planes, int maxNumberOfPlanes,
        const CGAL::Bbox_3 &bbox, const std::map<int, int> &cell2label, const std::vector<bool> &labels);
void loadArrangement(const std::string &name, Arrangement &arr, std::map<int, int> &cell2label, std::vector<bool> &labels);

// Semantics
std::vector<classKeywordsColor> loadSemanticClasses(const std::string& path);

#endif
