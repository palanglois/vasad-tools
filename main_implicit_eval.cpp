// External
#include "OptionParser/option_parser.h"

// STD
#include <experimental/filesystem>

// Own
#include "EvalMetrics.h"


using namespace std;
namespace fs = std::experimental::filesystem;

int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input",
                   "Path to the directory containing the raw implicit output ply files (one per chunk per class)", "");
    opt.add_option("-o", "--output", "Output directory", ".");
    opt.add_option("-m", "--mesh", "Path to the input obj ground truth", "");
    opt.add_option("-ns", "--number_surfacic", "Number of surfacic samples", "10000");
    opt.add_option("-nv", "--number_volumic", "Number of volumic samples", "10000");
    opt.add_option("-v", "--verbose", "Verbosity trigger");

    //Parsing options
    bool correctParsing = opt.parse_options(argc, argv);
    if (!correctParsing)
        return EXIT_FAILURE;

    if (op::str2bool(opt["-h"])) {
        opt.show_help();
        return 0;
    }

    const string gtPath = opt["-m"];
    const string inputPath = opt["-i"][opt["-i"].size() - 1] == '/' ? opt["-i"] : opt["-i"] + '/';
    const string outputPath = opt["-o"][opt["-o"].size() - 1] == '/' ? opt["-o"] : opt["-o"] + '/';
    const int nbSamplesSurfacic = op::str2int(opt["-ns"]);
    const int nbSamplesVolumic = op::str2int(opt["-nv"]);
    bool verbose = op::str2bool(opt["-v"]);
    const double percentage = 0.95;

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");
    if (verbose)
        for (int i = 0; i < classesWithColor.size(); i++)
            cout << "Class " << i << " is " << get<0>(classesWithColor[i]) << endl;

    // Load the ground truth
    cout << "Loading ground truth..." << endl;
    auto shapesAndClasses = loadTreesFromObj(gtPath, classesWithColor);
    cout << "Ground truth loaded." << endl;


    vector<facesLabelName> labeledShapesPred;
    const fs::path pathToPred(inputPath);
    vector<string> fileNames;
    vector<fs::path> filePaths;

    // Find the pred files
    for (const auto &predFile : fs::directory_iterator(pathToPred)) {
        const auto predFileStr = predFile.path().filename().string();
        if (predFileStr.substr(predFileStr.size() - 3, 3) != "ply") continue;
        fileNames.push_back(predFileStr);
        filePaths.push_back(predFile);
    }

    // Read the pred files
    auto tqPredFiles = tq::trange(fileNames.size());
    tqPredFiles.set_prefix("Loading prediction files: ");
    for (int i: tqPredFiles) {
        fs::path &predFile = filePaths[i];
        string predFileStr = fileNames[i];

        // Load the class idx
        int classIdx = stoi(predFileStr.substr(9, predFileStr.size() - 13));

        // Load the triangles
        vector<Triangle> faces = loadTrianglesFromPly(predFile.string());

        labeledShapesPred.emplace_back(faces, classIdx, predFileStr);
    }

    // Compute the metrics
    EvalMetrics metrics(labeledShapesPred, shapesAndClasses, classesWithColor,
                        nbSamplesVolumic, nbSamplesSurfacic);

    // IoU
    cout << "IoU: " << metrics.getIoU() << endl;
    cout << "IoU geometric: " << metrics.getIoUGeometric() << endl;

    // Confusion Matrix
    vector<vector<int>> confusionMatrix = metrics.getConfusionMatrix();
    for (int i = 0; i < confusionMatrix.size(); i++) {
        for (int j = 0; j < confusionMatrix[i].size(); j++)
            cout << confusionMatrix[i][j] << " ";
        cout << endl;
    }

    // Mean surfacic distance
    cout << "Mean surfacic distance: " << metrics.meanSurfacicDistance() << endl;

    // Max surfacic distance
    cout << "Max surfacic distance: " << metrics.maxSurfacicDistance() << endl;

    // Precision
    cout << "Distance d such that " << percentage * 100 << "% of NN distances are under d (precision): "
         << metrics.precision(percentage) << endl;

    // Precision
    cout << "Distance d such that " << percentage * 100 << "% of NN distances are under d (recall): "
         << metrics.recall(percentage) << endl;

    // Save
    metrics.saveAsJson(outputPath + "metrics.json", percentage);

    return 0;
}