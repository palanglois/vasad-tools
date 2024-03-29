// External
#include "OptionParser/option_parser.h"

// Own
#include "lib/iogeometry.h"
#include "lib/graphStatsInline.h"

using namespace std;

int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Input txt file from LightConvPoint output", "");
    opt.add_option("-s", "--scan", "Original scan (obj file)", "");
    opt.add_option("-p", "--pov", "Original point of views (obj file)", "");
    opt.add_option("-o", "--output", "Path to the output folder", "");
    opt.add_option("-v", "--verbose", "Verbosity trigger");

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

    if(opt["-s"].empty())
    {
        cerr << "Original scan file (-s) required !" << endl;
        return EXIT_FAILURE;
    }

    if(opt["-o"].empty())
    {
        cerr << "Output directory (-o) required !" << endl;
        return EXIT_FAILURE;
    }

    const string outputPath = opt["-o"][opt["-o"].size() - 1] == '/' ? opt["-o"] : opt["-o"] + '/';

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");

    // Loading Light conv point predictions
    tuple<vector<Point>, vector<int>, vector<vector<double>>> lcpData = loadLightConvPointOutput(opt["-i"]);
    vector<Point> &lcpPointCloud = get<0>(lcpData);
    vector<vector<double>> &richLabels = get<2>(lcpData);
    cout << "Loaded " << lcpPointCloud.size() << " points from LightConvPoint." << endl;

    // Loading points with label
    pair<vector<Point>, vector<Vector>> originalData = loadPointsWithNormals(opt["-s"]);
    cout << "Loaded " << originalData.first.size() << " points from the original scan." << endl;

    // Point of views
    vector<Point> pointOfViews = loadPointCloudObj(opt["-p"]);
    cout << "Loaded " << pointOfViews.size() << " point of views." << endl;

    // Make a mapping point -> original index
    map<Point, int> point2idx;
    for(int i=0; i < originalData.first.size(); i++)
        point2idx[originalData.first[i]] = i;

    // Make a kd tree based on the original data
    kdTree tree(originalData.first.begin(), originalData.first.end());
    cout << "KdTree built!" << endl;

    //Make the output data
    map<Point, vector<double>> outputPredLabels;
    vector<Point> outputPointOfViews;
    vector<Vector> normals;
    vector<colorTuple> pointColors;

    //Fill the data
    auto tqPoints = tq::trange(lcpPointCloud.size()); // works for rvalues too!
    tqPoints.set_prefix("Processing each point: ");
    for(int i: tqPoints) {
        Neighbor_search search(tree, lcpPointCloud[i], 1);
        int idx = point2idx.at(search.begin()->first);
        outputPredLabels[lcpPointCloud[i]] = richLabels[i];
        outputPointOfViews.push_back(pointOfViews[idx]);
        normals.push_back(originalData.second[idx]);
        int pseudoLabel = richLabels[i].size() == 1 ? richLabels[i][0] : arg_max(richLabels[i]);
        pointColors.push_back(get<2>(classesWithColor[pseudoLabel]));
    }
    vector<double> features(0);
    savePointsAsObjWithColors(lcpPointCloud, pointColors, outputPath + "goodVis.obj");
    savePointsAsObjWithLabel(make_pair(lcpPointCloud, outputPredLabels),
                             outputPath + "goodSamplesWithLabel.obj", normals, features);
    savePointsAsObj(outputPointOfViews, outputPath + "goodPov.obj");

    return 0;
}