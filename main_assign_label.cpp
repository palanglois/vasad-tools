// External
#include "OptionParser/option_parser.h"

// Own
#include "lib/graphStats.h"

using namespace std;


int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input json arrangement", "");
    opt.add_option("-m", "--model", "Path to the ground truth model obj file", "");

    //Parsing options
    bool correctParsing = opt.parse_options(argc, argv);
    if (!correctParsing)
        return EXIT_FAILURE;

    if (op::str2bool(opt["-h"])) {
        opt.show_help();
        return 0;
    }

    if (opt["-i"].empty()) {
        cerr << "Input file (-i) required !" << endl;
        opt.show_help();
        return EXIT_FAILURE;
    }

    if (opt["-m"].empty()) {
        cerr << "Input model file (-m) required !" << endl;
        opt.show_help();
        return EXIT_FAILURE;
    }

    const string inputPath = opt["-i"];
    const string modelPath = opt["-m"];

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");

    // Loading plane arrangement
    PlaneArrangement currentArrangement(inputPath);

    // Load the ground truth
    cout << "Loading ground truth..." << endl;
    vector<facesLabelName> shapesAndClasses = loadTreesFromObj(modelPath, classesWithColor);
    cout << "Ground truth loaded." << endl;

    for(const auto &shape: shapesAndClasses)
        cout << "Shape: " << get<2>(shape) << " has label " << get<1>(shape) << endl;

    // Assign label
    vector<int> labels = assignLabel(currentArrangement, shapesAndClasses, classesWithColor.size(), 40, true);

    // Display classes
    for(int i=0; i < classesWithColor.size(); i++)
        cout << "Class " << i << " is " << get<0>(classesWithColor[i]) << endl;
    return 0;
}