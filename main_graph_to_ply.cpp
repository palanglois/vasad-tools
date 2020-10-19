// External
#include "OptionParser/option_parser.h"

// Own
#include "lib/iogeometry.h"

using namespace std;

int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input plane arrangement", "");
    opt.add_option("-o", "--output", "Path to the output ply file", "");

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
        return EXIT_FAILURE;
    }

    if (opt["-o"].empty()) {
        cerr << "Output file (-o) required !" << endl;
        return EXIT_FAILURE;
    }

    const string inputPath = opt["-i"];
    const string outputPath = opt["-o"];

    // Loading plane arrangement
    map<int, int> cell2label;
    vector<bool> labels;
    vector<int> gtLabels;
    Arrangement arr;
    CGAL::Bbox_3 bbox;
    loadArrangement(inputPath, arr, cell2label, gtLabels, labels, bbox);

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");

    // Extract mesh
    savePlyFromLabel(outputPath, arr, cell2label, gtLabels, classesWithColor);

    return 0;
}
