// STD
#include <experimental/filesystem>

// External
#include "OptionParser/option_parser.h"

// Own
#include "lib/graphStats.h"

namespace fs = std::experimental::filesystem;

int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the dataset to update", "");
    opt.add_option("-r", "--raw", "Path to the raw dataset", "");

    //Parsing options
    bool correctParsing = opt.parse_options(argc, argv);
    if (!correctParsing)
        return EXIT_FAILURE;

    if (op::str2bool(opt["-h"])) {
        opt.show_help();
        return 0;
    }

    if (opt["-i"].empty()) {
        cerr << "Input dataset path (-i) required !" << endl;
        opt.show_help();
        return EXIT_FAILURE;
    }

    if (opt["-r"].empty()) {
        cerr << "Raw dataset path (-r) required !" << endl;
        opt.show_help();
        return EXIT_FAILURE;
    }

    const string inputPath = opt["-i"][opt["-i"].size() - 1] == '/' ? opt["-i"] : opt["-i"] + '/';
    const string rawPath = opt["-r"][opt["-r"].size() - 1] == '/' ? opt["-r"] : opt["-r"] + '/';

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");

    const fs::path pathToShow(inputPath);
    map<string, pair<vector<Point>, vector<int>>> scans;

    for (const auto& valTrain : fs::directory_iterator(pathToShow)) {
        const auto valTrainStr = valTrain.path().filename().string();
        if (fs::is_directory(valTrain)) {
            if(valTrainStr != "train" && valTrainStr != "val") continue;
            std::cout << "Entering dir:  " << valTrainStr << '\n';
            for (const auto& jsonFile : fs::directory_iterator(valTrain.path()))
            {
                if(jsonFile.path().extension().string() != ".json") continue;
                const auto jsonFileStr = jsonFile.path().filename().string();
                string modelName = jsonFileStr.substr(0, jsonFileStr.size() - 10);
                string inModelPath = jsonFile.path().string();
                if(scans.find(modelName) == scans.end())
                {
                    string scanPath = fs::path(rawPath);
                    string appendice = modelName + "/processed/" + modelName + "/samplesWithLabel.obj";
                    scanPath += appendice;
                    cout << "Loading scan located at " << scanPath << endl;
                    scans[modelName] = loadPointsWithLabel(scanPath);
                }
                pair<vector<Point>, vector<int>> curScan = scans[modelName];

                // Loading plane arrangement
                cout << endl << "Processing file: " << inModelPath << endl;
                PlaneArrangement currentArrangement(inModelPath);

                // Compute the new features
                EdgeFeatures edgeFeatures = computeFeaturesFromLabeledPoints(currentArrangement,
                                                                             curScan.first, curScan.second,
                                                                             classesWithColor.size(), true);

                // Replacing features and output the results
                currentArrangement.setEdgeLabels(edgeFeatures);
                currentArrangement.saveAsJson(inModelPath);

            }
        }
        else if (fs::is_regular_file(valTrain)) {
            std::cout << "file: " << valTrainStr << '\n';
        }
        else
            std::cout << "??    " << valTrainStr << '\n';
    }

    return 0;
}