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

// Boost
#include <boost/functional/hash.hpp>

// External
#include "json/json.hpp"

typedef CGAL::Simple_cartesian<double> Kernel;

/* Typedefs for the geometrical primitives */
typedef typename Kernel::Point_3 Point;
typedef typename Kernel::Segment_3 Segment;
typedef typename Kernel::Triangle_3 Triangle;
typedef typename Kernel::Vector_3 Vector;
typedef typename Kernel::Ray_3 Ray;


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
                    triangle(arg[0].x(), arg[0].y(), arg[0].z(),
                             arg[1].x(), arg[1].y(), arg[1].z(),
                             arg[2].x(), arg[2].y(), arg[2].z());
            return boost::hash_value(triangle);
        }
    };
}

// Typedef for storing semantic classes (name, keywords, color)
typedef std::tuple<std::string, std::vector<std::string>, std::vector<int>> classKeywordsColor;

// Typedefs for triangles and colors
typedef std::tuple<int, int, int> colorTuple;
typedef std::unordered_map<Triangle, colorTuple> TriangleColorMap;

// Input functions
std::pair<std::vector<Triangle>, TriangleColorMap> loadTrianglesFromObj(const std::string &objFile,
                                                                 const std::vector<classKeywordsColor> &classes);
std::vector<Point> loadPointOfViews(const std::string &jsonFile);

// Output functions
void savePointsAsObj(std::vector<Point> points, const std::string &outPath);
void savePointsAsObjWithColors(std::vector<Point> points, std::vector<colorTuple> colors, const std::string &outPath);
void saveTrianglesAsObj(std::vector<Triangle> triangles, const std::string &outPath, TriangleColorMap colors);

#endif
