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
    opt.add_option("-p", "--proba", "Proba of giving the empty occupency information for each empty cell", "1.");
    opt.add_option("-g", "--geom", "Adds the bounding box dimensions of each cell as a node feature");

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
    const double proba = op::str2double(opt["-p"]);
    const bool geom = op::str2bool(opt["-g"]);

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");

    // Load the ground truth
    const string gtPath = opt["-m"];
    cout << "Loading ground truth..." << endl;
    auto shapesAndClasses = loadTreesFromObj(gtPath, classesWithColor);
    cout << "Ground truth loaded." << endl;

    // Loading plane arrangement
    PlaneArrangement currentArrangement(inputPath);
    Arrangement &arr = currentArrangement.arrangement();
    const map<int, int> &cell2label = currentArrangement.cell2label();
    const CGAL::Bbox_3 &bbox = currentArrangement.bbox();

#ifndef NDEBUG
    const int nbSamples = 40;
#else
//        const int nbSamples = 4000000;
        const int nbSamples = 40;
#endif

//    for(auto prim: shapesAndClasses[shapesAndClasses.size() - 1].first->m_primitives)
//        cout << prim.datum() << endl;
//    cout << endl;
    vector<int> gtLabels = assignLabel(currentArrangement, shapesAndClasses, classesWithColor.size(), nbSamples, true);

    cout << "Saving reconstruction..." << endl;
    savePlyFromLabel("gt_reconstruction.ply", arr, cell2label, gtLabels, classesWithColor);
    cout << "Reconstruction saved." << endl;

    cout << "Computing statistics" << endl;
    int nbClasses = classesWithColor.size();
    NodeFeatures nodeFeats = computeNodeFeatures(currentArrangement, gtLabels, proba, geom, true);
    EdgeFeatures edgeFeats = computeTrivialEdgeFeatures(currentArrangement, gtLabels, nbClasses, true);
    cout << "Statistics Computed" << endl;

    cout << "Save the arrangement with labels and features" << endl;
    fstream i(inputPath);
    Json data;
    i >> data;
    Json cell2labelJ;
    for(auto idx: cell2label)
        cell2labelJ[to_string(idx.first)] = idx.second;
    data["map"] = cell2labelJ;
    data["NodeFeatures"] = nodeFeats;
    data["EdgeFeatures"] = edgeFeats;
    data["gtLabels"] = gtLabels;
    data["NodePoints"] = getCellsPoints(cell2label, arr);
    data["NodeBbox"] = getCellsBbox(cell2label, arr);
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
