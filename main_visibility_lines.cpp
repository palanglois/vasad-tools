// External
#include "OptionParser/option_parser.h"
#include "json/json.hpp"

// Own
#include "lib/graphStats.h"

using namespace std;
using Json = nlohmann::json;

int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input plane arrangement", "");
    opt.add_option("-o", "--output", "Path to the output json file", "");
    opt.add_option("-s", "--scan", "Path to the input scan obj file", "");
    opt.add_option("-p", "--pov", "Path to the input point of view obj file", "");
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

    if (opt["-s"].empty()) {
        cerr << "Input file (-s) required !" << endl;
        return EXIT_FAILURE;
    }

    if (opt["-p"].empty()) {
        cerr << "Input file (-p) required !" << endl;
        return EXIT_FAILURE;
    }

    const string inputPath = opt["-i"];
    const string outputPath = opt["-o"];
    const string scanPath = opt["-s"];
    const string povPath = opt["-p"];
    const bool verbose = op::str2bool(opt["-v"]);

    // Loading plane arrangement
    PlaneArrangement currentArrangement(inputPath);

    // Loading points with label
    pair<vector<Point>, vector<int>> pointsWithLabel = loadPointsWithLabel(scanPath);

    // Point of views
    vector<Point> pointOfViews = loadPointCloudObj(povPath);

    // Compute the visibility lines that pass through the current arrangement
    vector<Point> beginPoints;
    vector<Point> endPoints;
    addSegmentIfInBbox(pointOfViews, pointsWithLabel.first,
                       back_inserter(beginPoints), back_inserter(endPoints),
                       currentArrangement.bbox());

    // Save all the lines
    Json linesDataAll;
    vector<Json> allLines;
    for(int i=0; i < pointOfViews.size(); i++)
    {
        if(i % 1000 != 0) continue;
        Json currentLine;
        currentLine["pt1"] = pointOfViews[i];
        currentLine["pt2"] = pointsWithLabel.first[i];
        currentLine["pt_views"] = vector<double>(0);
        allLines.push_back(currentLine);
    }
    linesDataAll["lines"] = allLines;
    ofstream allFileOut(outputPath.substr(0, outputPath.size() - 5) + "_all.json");
    allFileOut << linesDataAll;
    allFileOut.close();

    // Save it
    Json linesData;
    vector<Json> lines;
    for(int i=0; i < beginPoints.size(); i++)
    {
        if(i % 100 != 0) continue;
        Json currentLine;
        currentLine["pt1"] = beginPoints[i];
        currentLine["pt2"] = endPoints[i];
        currentLine["pt_views"] = vector<double>(0);
        lines.push_back(currentLine);
    }

    linesData["lines"] = lines;
    ofstream fileOut(outputPath);
    fileOut << linesData;
    fileOut.close();


    return 0;
}
