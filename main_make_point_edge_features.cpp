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


void savePlyFromLabel(const std::string &filename, Arrangement &arr, const std::map<int, int> &cell2label,
                      const EdgeFeatures &edgeFeatures, const std::vector<classKeywordsColor> &classesWithColor)
{
    // Making colormap
    std::map<int, Colormap::Color> map;
    for(int i = 0; i < classesWithColor.size(); i++)
    {
        const auto& classColor = get<2>(classesWithColor[i]);
        map[i] = Colormap::Color(static_cast<unsigned char>(get<0>(classColor)),
                                 static_cast<unsigned char>(get<1>(classColor)),
                                 static_cast<unsigned char>(get<2>(classColor)));
    }
    Colormap colormap(map);

    for(auto itf = arr.facets_begin(); itf != arr.facets_end(); itf++){
        Arrangement::Face& f = *itf;
        f._info = -1;
        itf->to_draw = false;
        if(!arr.is_facet_bounded(f)){continue;}
        Arrangement::Face_handle ch0 = f.superface(0), ch1 = f.superface(1);
        if(!arr.is_cell_bounded(ch0) || !arr.is_cell_bounded(ch1)) continue;
        pair<int, int> key1(cell2label.at(ch0), cell2label.at(ch1));
        pair<int, int> key2(cell2label.at(ch1), cell2label.at(ch0));
        pair<int, int> goodKey(-1, -1);
        if(edgeFeatures.find(key1) != edgeFeatures.end())
            goodKey = key1;
        else if(edgeFeatures.find(key2) != edgeFeatures.end())
            goodKey = key2;
        if(goodKey.first == -1) continue;
        f.to_draw = true;

        int goodLabel = -1;
        double bestScore = -1;
        for(int i=0; i < edgeFeatures.at(goodKey).size(); i++)
            if(edgeFeatures.at(goodKey)[i] > bestScore)
            {
                bestScore = edgeFeatures.at(goodKey)[i];
                goodLabel = i;
            }
        if(goodLabel != -1)
            f._info = goodLabel;
        if(goodLabel == classesWithColor.size())
            f.to_draw = false;
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
        Polyhedral_complex_3::print_mesh_with_facet_color_PLY(stream, meshGC, colormap);
        stream.close();
    }
}

int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input json arrangement", "");
    opt.add_option("-s", "--scan", "Path to the input obj scan (points with label)", "");

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

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");

    // Loading plane arrangement
    PlaneArrangement currentArrangement(inputPath);

    // Loading points with label
    pair<vector<Point>, vector<int>> pointsWithLabel = loadPointsWithLabel(scanPath);

    // Compute the new features
    EdgeFeatures edgeFeatures = computeFeaturesFromLabeledPoints(currentArrangement,
                                                                 pointsWithLabel.first, pointsWithLabel.second,
                                                                 classesWithColor.size(), 40, true);

    // DEBUG
    const EdgeFeatures& oldFeatures = currentArrangement.edgeFeatures();
    savePlyFromLabel((string) TEST_DIR + "debugEdgeFeatures.ply", currentArrangement.arrangement(),
                     currentArrangement.cell2label(), edgeFeatures, classesWithColor);
    savePlyFromLabel((string) TEST_DIR + "debugEdgeFeaturesOld.ply", currentArrangement.arrangement(),
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