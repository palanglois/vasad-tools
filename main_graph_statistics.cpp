#include <iostream>

// External
#include "OptionParser/option_parser.h"

// Own
#include "lib/iogeometry.h"
#include "lib/graphStats.h"

using namespace std;


int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input plane arrangement", "");
    opt.add_option("-m", "--mesh", "Path to the input obj ground truth", "");

    //Parsing options
    bool correctParsing = opt.parse_options(argc, argv);
    if (!correctParsing)
        return EXIT_FAILURE;

    if (op::str2bool(opt["-h"])) {
        opt.show_help();
        return 0;
    }

    if(opt["-i"].empty())
    {
        cerr << "Input file (-i) required !" << endl;
        return EXIT_FAILURE;
    }

    if(opt["-m"].empty())
    {
        cerr << "Input obj ground truth (-m) required !" << endl;
        return EXIT_FAILURE;
    }

    const string inputPath = opt["-i"];

    // Loading plane arrangement
    map<int, int> cell2label;
    vector<bool> labels;
    Arrangement arr;
    CGAL::Bbox_3 bbox;
    loadArrangement(inputPath, arr, cell2label, labels, bbox);

    pair<Nodes, Edges> nodesEdges = computeGraphStatistics(labels, cell2label, arr, true);

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses("../semantic_classes.json");

    // Load the ground truth
    const string gtPath = opt["-m"];
    cout << "Loading ground truth..." << endl;
    auto treesAndClasses = loadTreesFromObj(gtPath, classesWithColor);
    cout << "Ground truth loaded." << endl;

    vector<int> gtLabels = assignLabel(arr, cell2label, bbox, treesAndClasses, 1000000, true);


//    cout << count << " " << visitedCell.size() << endl;
//    cout << "Label2Cell size: " << label2cell.size() << endl;
//    cout << "Labels size: " << labels.size() << endl;
//    cout << "Number of cells: " << arr.number_of_cells() << endl;
//    int cellNb = 0;
//    for(auto cellIt = arr.cells_begin(); cellIt != arr.cells_end(); cellIt++)
//        cellNb++;
//    cout << "Nb cells arr: " << cellNb << endl;

    return 0;
}