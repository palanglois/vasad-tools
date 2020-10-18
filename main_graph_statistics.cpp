#include <iostream>

// External
#include "OptionParser/option_parser.h"

// Own
#include "lib/iogeometry.h"
#include "lib/graphStats.h"

using namespace std;
using Json = nlohmann::json;



int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input plane arrangement", "");
    opt.add_option("-o", "--output", "Path to the output folder", "");
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

    if(opt["-o"].empty())
    {
        cerr << "Output file (-o) required !" << endl;
        return EXIT_FAILURE;
    }

    const string inputPath = opt["-i"];
    const string outputPath = opt["-o"][opt["-o"].size() - 1] == '/' ? opt["-o"] : opt["-o"] + '/';

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");

    // Load the ground truth
    const string gtPath = opt["-m"];
    cout << "Loading ground truth..." << endl;
    auto shapesAndClasses = loadTreesFromObj(gtPath, classesWithColor);
    cout << "Ground truth loaded." << endl;

    // Loading plane arrangement
    map<int, int> cell2label;
    vector<bool> labels;
    vector<int> gtLabelsOr;
    Arrangement arr;
    CGAL::Bbox_3 bbox;
    loadArrangement(inputPath, arr, cell2label, gtLabelsOr, labels, bbox);

#ifndef NDEBUG
    const int nbSamples = 100000;
#else
//        const int nbSamples = 4000000;
        const int nbSamples = 400000;
#endif

//    for(auto prim: shapesAndClasses[shapesAndClasses.size() - 1].first->m_primitives)
//        cout << prim.datum() << endl;
//    cout << endl;
    vector<int> gtLabels = assignLabel(arr, cell2label, bbox, shapesAndClasses, nbSamples, true, true);

    cout << "Saving reconstruction..." << endl;
    savePlyFromLabel("gt_reconstruction.ply", arr, cell2label, gtLabels, classesWithColor);
    cout << "Reconstruction saved." << endl;

    cout << "Computing statistics" << endl;
    int nbClasses = classesWithColor.size();
    pair<NodeFeatures, EdgeFeatures> nodesEdges = computeGraph(gtLabels, cell2label, arr, nbClasses, true);
    cout << "Statistics Computed" << endl;

    cout << "Save the arrangement with labels and features" << endl;
    fstream i(inputPath);
    Json data;
    i >> data;
    Json cell2labelJ;
    for(auto idx: cell2label)
        cell2labelJ[to_string(idx.first)] = idx.second;
    data["map"] = cell2labelJ;
    data["NodeFeatures"] = nodesEdges.first;
    data["EdgeFeatures"] = nodesEdges.second;
    data["gtLabels"] = gtLabels;
    data["NodePoints"] = getCellsPoints(cell2label, arr);
    ofstream o(outputPath + "arrangementWithGtAndLabels.json");
    o << data;


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
