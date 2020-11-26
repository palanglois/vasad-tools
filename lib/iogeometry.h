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
#include <random>

// CGAL
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_integer.h>
//#include <CGAL/Polyhedron_3.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_3.h>

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
#include "tqdm/tqdm.hpp"

// Ours
#include "Colormap.h"

typedef CGAL::Simple_cartesian<double> Kernel;
//typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel2;
//typedef CGAL::Homogeneous<CGAL::Exact_integer>  Kernel;
typedef CGAL::Cartesian_converter<Kernel,Kernel2>         Simple_to_Epeck;
typedef CGAL::Cartesian_converter<Kernel2,Kernel>         Epeck_to_Simple;

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

/* Typedef for nearest neighbour search */
typedef CGAL::Search_traits_3<Kernel> TreeTraits;
typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> Neighbor_search;
typedef Neighbor_search::Tree kdTree;

/* Typedefs for the graph representation */
typedef std::vector<std::vector<int>> Nodes;
typedef std::vector<std::pair<int, int>> Edges;
typedef std::vector<std::vector<double>> NodeFeatures;
typedef std::map<std::pair<int, int>, std::vector<double>> EdgeFeatures;


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
typedef std::unordered_map<Triangle, int> TriangleClassMap;

// Typedef for storing semantic classes (name, keywords, color)
typedef std::tuple<std::string, std::vector<std::string>, colorTuple> classKeywordsColor;

// Typedef for storing a shape (faces, label, name)
typedef std::tuple<std::vector<Triangle>, int, std::string> facesLabelName;

struct Plane
{
    Kernel2::Point_3 inlier;
    Kernel2::Vector_3 normal;
    std::vector<std::vector<int>> faces;
    double cumulatedPercentage;
};

// Input/Output json for the Plane type
inline void to_json(nlohmann::json& j, const Plane& p) {
    j = nlohmann::json{{"inlier", {CGAL::to_double(p.inlier.x()), CGAL::to_double(p.inlier.y()), CGAL::to_double(p.inlier.z())}},
                       {"normal", {CGAL::to_double(p.normal.x()), CGAL::to_double(p.normal.y()), CGAL::to_double(p.normal.z())}},
                       {"cumulatedPercentage", p.cumulatedPercentage}};
    nlohmann::json faces;
    for(const auto & face : p.faces)
    {
        nlohmann::json faceJson;
        for(int coordinate : face)
            faceJson.push_back(coordinate);
        faces.push_back(faceJson);
    }
    j["faces"] = faces;
}

inline void from_json(const nlohmann::json& j, Plane& p) {
    p.inlier = Kernel2::Point_3(j.at("inlier")[0].get<double>(), j.at("inlier")[1].get<double>(), j.at("inlier")[2].get<double>());
    p.normal = Kernel2::Vector_3(j.at("normal")[0].get<double>(), j.at("normal")[1].get<double>(), j.at("normal")[2].get<double>());
    p.cumulatedPercentage = j.at("cumulatedPercentage").get<double>();
    p.faces = j.at("faces").get<std::vector<std::vector<int>>>();
}

// Input/Output json for the CGAL::Bbox_3 type
namespace CGAL {
    inline void to_json(nlohmann::json &j, const CGAL::Bbox_3 &b) {
        j = {b.xmin(), b.ymin(), b.zmin(), b.xmax(), b.ymax(), b.zmax()};
    }

    inline void from_json(const nlohmann::json &j, CGAL::Bbox_3 &b) {
        b = CGAL::Bbox_3(j[0].get<double>(), j[1].get<double>(), j[2].get<double>(),
                         j[3].get<double>(), j[4].get<double>(), j[5].get<double>());
    }
}

// A class for the Plane Arrangement and its attributes
class PlaneArrangement
{
public:
    explicit PlaneArrangement(const std::string& name);
    PlaneArrangement(const std::vector<Plane> &inPlanes, const std::vector<int>& validPlaneIdx, const CGAL::Bbox_3 &inBbox);
    PlaneArrangement(const std::vector<Plane> &inPlanes, const std::map<int, int> &cell2label, const CGAL::Bbox_3 &inBbox);

    void saveAsJson(const std::string& outPath) const;

    // Setters
    void setEdgeLabels(const EdgeFeatures& edgeFeatures);

    // Accessors
    [[nodiscard]] Arrangement &arrangement();
    [[nodiscard]] const std::map<int, int> &cell2label() const;
    [[nodiscard]] const std::vector<int> &labels() const;
    [[nodiscard]] const std::vector<int> &gtLabels() const;
    [[nodiscard]] const CGAL::Bbox_3 &bbox() const;
    [[nodiscard]] const std::vector<Plane> &planes() const;
    [[nodiscard]] const std::vector<Point> &points() const;
    [[nodiscard]] const EdgeFeatures &edgeFeatures() const;
    [[nodiscard]] const std::vector<std::pair<Point, int>> &getSamples(int nbSamplesPerCell=40);

    // Hit and run sampling for the plane arrangement
    void sampleInConvexCell(int cellHandle, int nbSamples=40);

private:
    Arrangement _arr;
    std::map<int, int> _cell2label;
    std::vector<int> _labels;
    std::vector<int> _gtLabels;
    CGAL::Bbox_3 _bbox;
    std::vector<Point> _points;
    std::vector<Plane> _planes;
    NodeFeatures _nodeFeatures;
    EdgeFeatures _edgeFeatures;
    std::vector<std::vector<double>> _cellPoints;
    std::vector<CGAL::Bbox_3> _nodeBboxes;
    int _nbPlanes;
    std::vector<std::pair<Point, int>> _samples;


    bool isArrangementComputed;
    int computedSamplesPerCell;

};

// Input functions
std::pair<std::vector<Triangle>, TriangleClassMap> loadTrianglesFromObj(const std::string &objFile,
                                                                        const std::vector<classKeywordsColor> &classes);
std::vector<Point> loadPointOfViews(const std::string &jsonFile);
std::vector<Point> loadPointCloudObj(const std::string &inFile);
std::vector<facesLabelName> loadTreesFromObj(const std::string &inFile,
        const std::vector<classKeywordsColor> &classes);
std::pair<std::vector<Point>, std::vector<int>> loadPointsWithLabel(const std::string &inFile);

// Output functions
void savePointsAsObj(const std::vector<Point>& points, const std::string &outPath);
void savePointsAsObjWithLabel(const std::pair<std::vector<Point>, std::map<Point, int>> &pointsWithLabel, const std::string &outPath);
void savePointsAsObjWithColors(std::vector<Point> points, std::vector<colorTuple> colors, const std::string &outPath);
void saveTrianglesAsObj(const std::vector<Triangle>& triangles, const std::string &outPath, TriangleClassMap triangleClasses, const std::vector<classKeywordsColor> &classes);
void saveSeparatedObj(std::vector<Triangle> triangles, const std::string &outPath, TriangleClassMap triangleClasses, const std::vector<classKeywordsColor> &classes);
void saveArrangement(const std::string &name, const std::vector<Kernel::Plane_3> &planes, int maxNumberOfPlanes,
        const CGAL::Bbox_3 &bbox, const std::map<int, int> &cell2label, const std::vector<bool> &labels);
void savePlyFromEdgeFeatures(const std::string &filename, Arrangement &arr, const std::map<int, int> &cell2label,
                             const EdgeFeatures &edgeFeatures, const std::vector<classKeywordsColor> &classesWithColor);

template <class T>
void savePlyFromLabel(const std::string &filename, Arrangement &arr, const std::map<int, int> &fh_to_node,
        const std::vector<T> &labels, const std::vector<classKeywordsColor> &classesWithColor)
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
        Arrangement::Face& f = *itf;
        f._info = -1;
        itf->to_draw = false;
        if(! arr.is_facet_bounded(f)){continue;}
        Arrangement::Face_handle ch0 = f.superface(0), ch1 = f.superface(1);
        //if(!(is_cell_bounded(ch0) && is_cell_bounded(ch1))){continue;}
        if(fh_to_node.count((int)ch0) ==0 || fh_to_node.count((int)ch1) == 0){continue;}
        T label1 = labels[fh_to_node.at(int(ch0))];
        T label2 = labels[fh_to_node.at(int(ch1))];
        if(labels[fh_to_node.at(int(ch0))] != labels[fh_to_node.at(int(ch1))]){
            f.to_draw = true;
        }
        if(label1 == -1 && label2 != -1)
            f._info = label2;
        if(label1 != -1 && label2 == -1)
            f._info = label1;
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

// Semantics
std::vector<classKeywordsColor> loadSemanticClasses(const std::string& path);

// Utilities

std::vector<std::vector<double>> getCellsPoints(const std::map<int, int> &cell2label, const Arrangement &arr);
std::vector<std::vector<double>> getCellsBbox(const std::map<int, int> &cell2label, const Arrangement &arr);

#endif
