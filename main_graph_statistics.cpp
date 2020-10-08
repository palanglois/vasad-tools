#include <iostream>

// External
#include "OptionParser/option_parser.h"

// Own
#include "lib/iogeometry.h"
#include "lib/graphStats.h"

using namespace std;
using Json = nlohmann::json;



void savePlyFromLabel(const string &filename, Arrangement &arr, map<int, int> &fh_to_node, const vector<bool> &labels)
{

    for(auto itf = arr.facets_begin(); itf != arr.facets_end(); itf++){
        Arrangement::Face& f = *itf;
        itf->to_draw = false;
        if(! arr.is_facet_bounded(f)){continue;}
        Arrangement::Face_handle ch0 = f.superface(0), ch1 = f.superface(1);
        //if(!(is_cell_bounded(ch0) && is_cell_bounded(ch1))){continue;}
        if(fh_to_node.count((int)ch0) ==0 || fh_to_node.count((int)ch1) == 0){continue;}
        if(labels[fh_to_node[int(ch0)]] != labels[fh_to_node[int(ch1)]]){
            f.to_draw = true;
        }
    }

    typedef Polyhedral_complex_3::Mesh_3<> Mesh;
    typedef Polyhedral_complex_3::Mesh_extractor_3<Arrangement,Mesh> Extractor;
    Mesh meshGC;
    Extractor extractorGC(arr);
    extractorGC.extract(meshGC,false);
    {
        std::ofstream stream(filename.c_str());
        if (!stream.is_open())
            return ;
        Polyhedral_complex_3::print_mesh_PLY(stream, meshGC);
        stream.close();
    }
}

int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input plane arrangement", "");
    opt.add_option("-o", "--output", "Path to the output plane arrangement", "");
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
        cerr << "Output file (-i) required !" << endl;
        return EXIT_FAILURE;
    }

    const string inputPath = opt["-i"];
    const string outputPath = opt["-o"][opt["-o"].size() - 1] == '/' ? opt["-o"] : opt["-o"] + '/';

    // Loading plane arrangement
    map<int, int> cell2label;
    vector<bool> labels;
    Arrangement arr;
    CGAL::Bbox_3 bbox;
    loadArrangement(inputPath, arr, cell2label, labels, bbox);

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses("../semantic_classes.json");

    // Load the ground truth
    const string gtPath = opt["-m"];
    cout << "Loading ground truth..." << endl;
    auto shapesAndClasses = loadTreesFromObj(gtPath, classesWithColor);
    cout << "Ground truth loaded." << endl;

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
    vector<bool> gtLabelsBool;
    for(auto label: gtLabels)
        gtLabelsBool.push_back(label != -1);



    cout << "Saving reconstruction..." << endl;
    savePlyFromLabel("gt_reconstruction.ply", arr, cell2label, gtLabelsBool);
    cout << "Reconstruction saved." << endl;

    cout << "Computing statistics" << endl;
    int nbClasses = classesWithColor.size();
    pair<NodeFeatures, EdgeFeatures> nodesEdges = computeGraph(gtLabels, cell2label, arr, nbClasses, true);
    cout << "Statistics Computed" << endl;

    cout << "Save the arrangement with labels and features" << endl;
    fstream i(inputPath);
    Json data;
    i >> data;
    data["map"] = cell2label;
    data["NodeFeatures"] = nodesEdges.first;
    data["EdgeFeatures"] = nodesEdges.second;
    data["gtLabels"] = gtLabels;
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
