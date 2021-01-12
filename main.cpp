// STD
#include <iostream>
#include <random>

// External
#include "OptionParser/option_parser.h"

// Own
#include "lib/iogeometry.h"

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

// External
#include "OptionParser/option_parser.h"
using Json = nlohmann::json;


int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input obj mesh", "");
    opt.add_option("-o", "--output", "Path to the output directory", ".");
    opt.add_option("-c", "--input", "Path to the json file containing the camera positions", "");
    opt.add_option("-n", "--number", "Total number of points to shoot", "10000");
    opt.add_option("-gt", "--groundtruth", "Output the ground truth mesh with colored classes");
    opt.add_option("-pov", "--pointofview", "Save the point of views for each point");
    opt.add_option("-so", "--save_objects", "Output the separated_images");
    opt.add_option("-no", "--normal", "Save normals");

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

    const bool withNormal = op::str2bool(opt["-no"]);

    // Load point of views
    vector<Point> pointOfViews;
    pointOfViews = loadPointOfViews(opt["-c"]);

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");

    // Load 3D model
    vector<Triangle> triangles;
    const string inputPath = opt["-i"];
    auto trianglesAndClasses = loadTrianglesFromObj(inputPath, classesWithColor);
    triangles = trianglesAndClasses.first;
    if (op::str2bool(opt["-gt"]))
        saveTrianglesAsObj(vector<Triangle>(triangles.begin(), triangles.end()),
                           outPath + "coloredTriangles.obj", trianglesAndClasses.second, classesWithColor);
    if (op::str2bool(opt["-so"]))
        saveSeparatedObj(vector<Triangle>(triangles.begin(), triangles.end()),
                         outPath + "sep_obj", trianglesAndClasses.second, classesWithColor);

    //Build intersection tree
    Tree tree(triangles.begin(), triangles.end());

    //Random shoots from given point of views
    int nbShootPerPov = nbShoot / pointOfViews.size();
    vector<Point> sampledPoints;
    vector<Point> pointOfViews2;
    vector<Vector> normals;
    vector<colorTuple> pointColors;
    map<Point, int> pointClasses;
    pointColors.reserve(nbShoot);
    sampledPoints.reserve(nbShoot);
    pointOfViews.reserve(nbShoot);

    default_random_engine generator;
    normal_distribution<double> distribution(0.0, 1.0);
    int curPt = 0;
    for (auto pov: pointOfViews) {
        curPt++;
        cout << "Doing pt " << curPt << " out of " << pointOfViews.size() << endl;
#pragma omp parallel for schedule(dynamic, 1)
        for (int i = 0; i < nbShootPerPov; i++) {

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
                        Triangle& curTri = *boost::get<Primitive_id>(intersection->second);
                        Vector curNormal;
                        if(withNormal) {
                            curNormal = CGAL::normal(curTri.vertex(0), curTri.vertex(1), curTri.vertex(2));
                            curNormal /= sqrt(curNormal.squared_length());
                            if (ray.to_vector() * curNormal > 0)
                                curNormal *= -1.;
                            normals.push_back(curNormal);
                        }
                        int classIdx = trianglesAndClasses.second[curTri];
                        pointColors.push_back(get<2>(classesWithColor[classIdx]));
                        const Point *p = boost::get<Point>(&(intersection->first));
                        sampledPoints.push_back(*p);
                        pointClasses[*p] = classIdx;
                        if(op::str2bool(opt["-pov"]))
                            pointOfViews2.push_back(pov);
                    }
                }
            }
        }
    }
    savePointsAsObjWithColors(sampledPoints, pointColors, outPath + "out.obj");
    savePointsAsObjWithLabel(make_pair(sampledPoints, pointClasses), outPath + "samplesWithLabel.obj", normals);
    if(op::str2bool(opt["-pov"]))
        savePointsAsObj(pointOfViews2, outPath + "pov.obj");
    return 0;
}
