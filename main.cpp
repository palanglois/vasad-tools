// STD
#include <iostream>
#include <random>

// External
#include "OptionParser/option_parser.h"

// Own
#include "iogeometry.h"

// CGAL
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

// Typedefs for AABB trees
typedef std::vector<Triangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<Kernel, Iterator> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;
typedef boost::optional<Tree::Intersection_and_primitive_id<Ray>::Type> Ray_intersection;
typedef Tree::Primitive_id Primitive_id;

using namespace std;
using Json = nlohmann::json;


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
        cout << itr.key() << endl;

        //Color
        Json colorArray = itr.value().at("color");
        vector<int> color;
        for(auto color_itr = colorArray.begin(); color_itr != colorArray.end(); color_itr++)
            color.push_back(*color_itr);

        // Keywords
        Json keywordsArray = itr.value().at("keywords");
        vector<string> keywords;
        for(auto str_it = keywordsArray.begin(); str_it != keywordsArray.end(); str_it++)
            keywords.push_back(*str_it);
        semanticClasses.emplace_back(itr.key(), keywords, color);
    }
    return semanticClasses;
}

int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input obj mesh", "");
    opt.add_option("-o", "--output", "Path to the output directory", ".");
    opt.add_option("-c", "--input", "Path to the json file containing the camera positions", "");
    opt.add_option("-n", "--number", "Number of ray to shoot per point of view", "10000");
    opt.add_option("-gt", "--groundtruth", "Output the ground truth mesh with colored classes");

    //Parsing options
    bool correctParsing = opt.parse_options(argc, argv);
    if (!correctParsing)
        return EXIT_FAILURE;

    if (op::str2bool(opt["-h"])) {
        opt.show_help();
        return 0;
    }

    string outPath = opt["-o"][opt["-o"].size()-1] == '/' ? opt["-o"] : opt["-o"] + "/";

    int nbShoot = op::str2int(opt["-n"]);

    // Load point of views
    vector<Point> pointOfViews;
    pointOfViews = loadPointOfViews(opt["-c"]);

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses("../semantic_classes.json");

    // Load 3D model
    vector<Triangle> triangles;
    const string inputPath = opt["-i"];
    auto trianglesAndColors = loadTrianglesFromObj(inputPath, classesWithColor);
    triangles = trianglesAndColors.first;
    if(op::str2bool(opt["-gt"]))
        saveTrianglesAsObj(vector<Triangle>(triangles.begin(), triangles.end()),
                           outPath + "coloredTriangles.obj", trianglesAndColors.second);

    //Build intersection tree
    Tree tree(triangles.begin(), triangles.end());

    //Random shoots from given point of views
    vector<Point> sampledPoints;
    vector<colorTuple> pointColors;
    sampledPoints.reserve(nbShoot*pointOfViews.size());

    default_random_engine generator;
    normal_distribution<double> distribution(0.0, 1.0);
    for (auto pov: pointOfViews) {
#pragma omp parallel for schedule(dynamic, 1)
        for (int i = 0; i < nbShoot; i++) {

            // Compute random ray
            double x = distribution(generator);
            double y = distribution(generator);
            double z = distribution(generator);
            Ray ray(pov, pov + Vector(x, y, z));

            // Get intersection
            Ray_intersection intersection = tree.first_intersection(ray);
            if (intersection) {
#pragma omp critical
                {
                    if (boost::get<Point>(&(intersection->first))) {
                        const Primitive_id &primitive_id = boost::get<Primitive_id>(intersection->second);
                        colorTuple color = trianglesAndColors.second[*primitive_id];
                        pointColors.push_back(color);
                        const Point *p = boost::get<Point>(&(intersection->first));
                        sampledPoints.push_back(*p);
                    }
                }
            }
        }
    }
    savePointsAsObjWithColors(sampledPoints, pointColors, outPath + "out.obj");

    return 0;
}
