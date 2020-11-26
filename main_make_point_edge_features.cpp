// External
#include "OptionParser/option_parser.h"

// Own
#include "lib/graphStats.h"


//Polyhedral complex
#include <Polyhedral_complex_3/Arrangement_3.hpp>
#include <Polyhedral_complex_3/Mesh_3.hpp>
#include <Polyhedral_complex_3/Mesh_extractor_3.hpp>
#include <Polyhedral_complex_3/print_PLY.hpp>
#include "Polyhedral_complex_3/Polyhedral_complex_queries_3.hpp"

using namespace std;


int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input json arrangement", "");
    opt.add_option("-s", "--scan", "Path to the input obj scan (points with label)", "");
    opt.add_option("-p", "--pov", "Path to the input obj point of views", "");

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

    if (opt["-s"].empty()) {
        cerr << "Scan obj file (-s) required !" << endl;
        opt.show_help();
        return EXIT_FAILURE;
    }

    const string inputPath = opt["-i"];
    const string scanPath = opt["-s"];
    const string povPath = opt["-p"];

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");

    // Loading plane arrangement
    PlaneArrangement currentArrangement(inputPath);

    // Loading points with label
    pair<vector<Point>, vector<int>> pointsWithLabel = loadPointsWithLabel(scanPath);

    // Loading point of views
    auto pointOfViews = povPath.empty() ? vector<Point>(0) : loadPointOfViews(povPath);

    // Compute the new features
    EdgeFeatures edgeFeatures = computeFeaturesFromLabeledPoints(currentArrangement,
                                                                 pointsWithLabel.first, pointsWithLabel.second,
                                                                 classesWithColor.size(), 40,
                                                                 pointOfViews, true);

    // DEBUG
    const EdgeFeatures& oldFeatures = currentArrangement.edgeFeatures();
    savePlyFromEdgeFeatures((string) TEST_DIR + "debugEdgeFeatures.ply", currentArrangement.arrangement(),
                     currentArrangement.cell2label(), edgeFeatures, classesWithColor);
    savePlyFromEdgeFeatures((string) TEST_DIR + "debugEdgeFeaturesOld.ply", currentArrangement.arrangement(),
                     currentArrangement.cell2label(), oldFeatures, classesWithColor);

    int diff = 0;
    int sameF = 0;
    int notFound = 0;
    int sameFull = 0;
    int emptyToFull = 0;
    int fullToEmpty = 0;
    int fullInNew = 0;
    for(auto edgeFeat: edgeFeatures)
    {
        const vector<double>& newFeat = edgeFeat.second;
        pair<int, int> firstKey = edgeFeat.first;
        pair<int, int> secondKey(firstKey.second, firstKey.first);
        pair<int, int> goodKey(0, 0);
        if(oldFeatures.find(firstKey) != oldFeatures.end())
            goodKey = firstKey;
        else if(oldFeatures.find(secondKey) != oldFeatures.end())
            goodKey = secondKey;
        else
        {
            notFound++;
            cout << "Could not find key !!" << endl;
            continue;
        }
        const vector<double>& oldFeat = currentArrangement.edgeFeatures().at(goodKey);
        bool same = true;
        for(int i=0; i < newFeat.size(); i++)
        {
            same = newFeat[i] == oldFeat[i];
            if(!same) break;
        }
        sameFull += same && newFeat[newFeat.size() - 1] != 1;
        fullToEmpty += (!same) && newFeat[newFeat.size() - 1] == 1;
        emptyToFull += (!same) && oldFeat[oldFeat.size() - 1] == 1;
        fullInNew += newFeat[newFeat.size() - 1] != 1;
        if(same) {
            sameF++;
        }
        else
        {
            diff++;
            bool display = (newFeat[newFeat.size() - 1] != 1) && (oldFeat[oldFeat.size() - 1] == 1);
            if(display) {
                cout << "Different: " << endl;
                cout << "new: ";
                for (int i = 0; i < newFeat.size(); i++)
                    cout << newFeat[i] << " ";
                cout << endl;
                cout << "old: ";
                for (int i = 0; i < oldFeat.size(); i++)
                    cout << oldFeat[i] << " ";
                cout << endl;
            }

        }
    }
    cout << diff << " different features." << endl;
    cout << sameF << " same features." << endl;
    cout << "Number of full among same: " << sameFull << endl;
    cout << "Number of edges which went from empty to full " << emptyToFull << endl;
    cout << "Number of edges which went from full to empty " << fullToEmpty << endl;
    cout << "Number of full edges in new: " << fullInNew << endl;
    cout << notFound << " features not found." << endl;
    cout << oldFeatures.size() - diff - sameF - notFound << " features missing." << endl;
    for(int i=0; i < classesWithColor.size(); i++)
        cout << i << " " << get<0>(classesWithColor[i]) << endl;
    // END DEBUG

    // Replacing features and output the results
    currentArrangement.setEdgeLabels(edgeFeatures);
//    currentArrangement.saveAsJson(inputPath);

    return 0;
}