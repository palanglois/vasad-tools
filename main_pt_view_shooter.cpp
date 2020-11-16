// External
#include "OptionParser/option_parser.h"
#include "json/json.hpp"

// Own
#include "lib/graphStats.h"

using namespace std;
using Json = nlohmann::json;

int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input obj mesh", "");
    opt.add_option("-o", "--output", "Path to the output directory", ".");
    opt.add_option("-n", "--number", "Number of point of views to shoot", "200");

    //Parsing options
    bool correctParsing = opt.parse_options(argc, argv);
    if (!correctParsing)
        return EXIT_FAILURE;

    if (op::str2bool(opt["-h"])) {
        opt.show_help();
        return 0;
    }

    if (opt["-i"].empty()) {
        cerr << "Please provide the input obj mesh (-i)" << endl;
        return EXIT_FAILURE;
    }

    const string inputPath = opt["-i"];
    const string outPath = opt["-o"][opt["-o"].size()-1] == '/' ? opt["-o"] : opt["-o"] + "/";

    int nbShoot = op::str2int(opt["-n"]);

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");

    // Load 3D model
    auto trianglesAndClasses = loadTrianglesFromObj(inputPath, classesWithColor);
    vector<Triangle> &triangles = trianglesAndClasses.first;
    vector<facesLabelName> shapesAndClasses = loadTreesFromObj(inputPath, classesWithColor);

    // BBox Triangles
    CGAL::Bbox_3 bbox;
    for(const auto& triangle: triangles)
        bbox += triangle.bbox();
    vector<Triangle> bboxMesh = meshBbox(bbox);

    // Concatenate all triangles
    triangles.insert(triangles.end(), bboxMesh.begin(), bboxMesh.end());

    // Shoot the point of views
    vector<Point> ptViews = findPtViewInBbox(bbox, shapesAndClasses, triangles, nbShoot);

    // Output computed point of views
    Json outData;
    for(const auto& point: ptViews)
        outData.push_back({point.x(), point.y(), point.z()});
    Json finalData;
    finalData["cam_loc"] = outData;
    ofstream fileOut(outPath + "ptView.json");
    fileOut << finalData;
    fileOut.close();

}
