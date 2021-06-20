// External
#include "OptionParser/option_parser.h"

// STD
#include <experimental/filesystem>

// Own
#include "VoxelArrangement.h"

using namespace std;
namespace fs = std::experimental::filesystem;

int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input voxels directory", "");
    opt.add_option("-o", "--output", "Path to the output directory", "");
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

    const string inputPath = opt["-i"][opt["-i"].size() - 1] == '/' ? opt["-i"] : opt["-i"] + '/';
    const string outputPath = opt["-o"][opt["-o"].size() - 1] == '/' ? opt["-o"] : opt["-o"] + '/';
    const bool verbose = op::str2bool(opt["-v"]);

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");
    if (verbose)
        for (int i = 0; i < classesWithColor.size(); i++)
            cout << endl << "Class " << i << " is " << get<0>(classesWithColor[i]);
    cout << endl;


    const fs::path pathToPred(inputPath);
    vector<string> fileNames;
    vector<fs::path> filePaths;

    // Find the pred files
    for (const auto &predFile : fs::directory_iterator(pathToPred)) {
        const auto predFileStr = predFile.path().filename().string();
        if (predFileStr.substr(predFileStr.size() - 2, 2) != "h5") continue;
        fileNames.push_back(predFileStr);
        filePaths.push_back(predFile);
    }

    cout << "Nb of found files: " << fileNames.size() << endl;

    // Read the pred files
    int fileItr = 0;
#pragma omp parallel for
    for(int i=0; i < fileNames.size(); i++) {
        fs::path &predFile = filePaths[i];
        string predFileStr = fileNames[i];

        // Load the voxel arrangement
        VoxelArrangement voxArr(predFile.string());

        string classIdx = predFileStr.substr(17, predFileStr.size() - 20);
        string paddedIdx = padTo(classIdx, 8);
        string totalPath = outputPath + paddedIdx;

        // Save as ply
        voxArr.saveAllLabelsAsPly(totalPath, classesWithColor);

#pragma omp atomic
        fileItr++;
#pragma omp critical
        cout << "Processed " << fileItr << "/" << fileNames.size() << " files." << endl;
    }

    return 0;
}