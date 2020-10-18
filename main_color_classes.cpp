// External
#include "OptionParser/option_parser.h"


// Own
#include "lib/iogeometry.h"

using namespace std;

int main(int argc, char *argv[]) {
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

    if (opt["-i"].empty()) {
        cerr << "Please provide the path to the input mesh with -i" << endl;
        return EXIT_FAILURE;
    }

    string outPath = opt["-o"][opt["-o"].size() - 1] == '/' ? opt["-o"] : opt["-o"] + "/";
    outPath += "coloredTriangles.obj";

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");

    // Load 3D Model
    auto trianglesAndColors = loadTrianglesFromObj(opt["-i"], classesWithColor);

    // Save it
    saveTrianglesAsObj(vector<Triangle>(trianglesAndColors.first.begin(), trianglesAndColors.first.end()),
                       outPath, trianglesAndColors.second);

    cout << "Saved model in " << outPath << endl;

    return 0;
}