// External
#include "OptionParser/option_parser.h"

// Own
#include "lib/graphStats.h"
using namespace std;


int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input json arrangement", "");
    opt.add_option("-o", "--output", "Path to the output ply file to produce", "");

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

    if (opt["-o"].empty()) {
        cerr << "Output file (-o) required !" << endl;
        opt.show_help();
        return EXIT_FAILURE;
    }

    const string inputPath = opt["-i"];
    const string outputPath = opt["-o"];

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");

    // Loading plane arrangement
    PlaneArrangement currentArrangement(inputPath);

    // Save the edge features representation
    savePlyFromEdgeFeatures(outputPath, currentArrangement.arrangement(),
                            currentArrangement.mergedMapping(), currentArrangement.edgeFeatures(), classesWithColor);

    return 0;
}