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
    opt.add_option("-i", "--input", "Path to the folder containing the reconstruction PLY files.", "");
    opt.add_option("-o", "--output", "Path to the output folder", "");
    opt.add_option("-p", "--point", "Path to the LightConvPoint output point cloud with labels (txt)", "");

    //Parsing options
    bool correctParsing = opt.parse_options(argc, argv);
    if (!correctParsing)
        return EXIT_FAILURE;

    if (op::str2bool(opt["-h"])) {
        opt.show_help();
        return 0;
    }

    if (opt["-i"].empty()) {
        cerr << "Input directory (-i) required !" << endl;
        return EXIT_FAILURE;
    }

    if (opt["-o"].empty()) {
        cerr << "Output directory (-o) required !" << endl;
        return EXIT_FAILURE;
    }

    if (opt["-p"].empty()) {
        cerr << "LightConvPoint file (-p) required !" << endl;
        return EXIT_FAILURE;
    }

    const string inputPath = opt["-i"][opt["-i"].size() - 1] == '/' ? opt["-i"] : opt["-i"] + '/';
    const string pointCloudPath = opt["-p"];
    const string outputPath = (opt["-o"][opt["-o"].size() - 1] == '/' ? opt["-o"] : opt["-o"] + '/') + "semantic_improvement.txt";

    // Load the meshes
    vector<facesLabelName> shapes;
    const fs::path pathToPred(inputPath);
    for (const auto &predFile : fs::directory_iterator(pathToPred)) {
        const auto predFileStr = predFile.path().filename().string();
        if (predFileStr.substr(predFileStr.size() - 3, 3) != "ply") continue;
        vector<Triangle> triangles = loadTrianglesFromPly(predFile.path().string());
        int label = stoi(predFileStr.substr(9, predFileStr.size() - 13));
        shapes.emplace_back(triangles, label, predFileStr);
        cout << predFileStr << " " << label << endl;
    }

    // Load the point cloud
    tuple<vector<Point>, vector<int>, vector<vector<double>>> lcpData = loadLightConvPointOutput(pointCloudPath);
    vector<Point> &pointCloud = get<0>(lcpData);
    vector<int> &gtLabels = get<1>(lcpData);
    vector<int> predLabels;
    for(const auto& logits: get<2>(lcpData))
        predLabels.push_back(arg_max(logits));

    pair<vector<int>, vector<double>> labelsAndDist = computePcClosestLabel(pointCloud, shapes);
    vector<int> &newLabels = labelsAndDist.first;

    int nbGoodLabelOld = 0;
    int nbGoodLabelNew = 0;
    for(int i=0; i < gtLabels.size(); i++)
    {
        nbGoodLabelOld += (gtLabels[i] == predLabels[i]);
        nbGoodLabelNew += (gtLabels[i] == newLabels[i]);
    }

    cout << "Old perfo: " << double(nbGoodLabelOld) / double(gtLabels.size()) << endl;
    cout << "New perfo: " << double(nbGoodLabelNew) / double(gtLabels.size()) << endl;

    ofstream outStream(outputPath);
    for(int i=0; i < pointCloud.size(); i++)
    {
        outStream << pointCloud[i].x() << " " << pointCloud[i].y() << " " << pointCloud[i].z() << " "
                  << gtLabels[i] << " " << predLabels[i] << " " << newLabels[i] << endl;
    }

    return 0;
}
