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
    opt.add_option("-v", "--verbose", "Verbosity trigger");

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
    const bool verbose = op::str2bool(opt["-v"]);

    // Loading plane arrangement
    PlaneArrangement currentArrangement(inputPath);
    Arrangement &arr = currentArrangement.arrangement();
    const map<int, int> &cell2label = currentArrangement.cell2label();
    const std::vector<int> &gtLabels = currentArrangement.gtLabels();

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");
    if(verbose)
        for(int i=0; i < classesWithColor.size(); i++)
            cout << endl << "Class " << i << " is " << get<0>(classesWithColor[i]);
        cout << endl;

    // If nodes have been merged, we need to build the correct mapping
    const vector<int> &nodes2Merge = currentArrangement.nodes2Merged();
    map<int, int> correctMapping;
    if(!nodes2Merge.empty()) {
        for (auto cellIt = arr.cells_begin(); cellIt != arr.cells_end(); cellIt++) {
            if (!arr.is_cell_bounded(*cellIt)) continue;
            int cellLabel = arr.cell_handle(*cellIt);
            correctMapping[cellLabel] = nodes2Merge[cell2label.at(cellLabel)];
        }
    }
    else
        correctMapping = cell2label;

    // Extract mesh
    savePlyFromLabel(outputPath, arr, correctMapping, gtLabels, classesWithColor);

    return 0;
}
