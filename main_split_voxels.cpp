// External
#include "OptionParser/option_parser.h"

// Own
#include "lib/VoxelArrangement.h"

using namespace std;

int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-o", "--output", "Path to the output folder", "");
    opt.add_option("-pr", "--prefix", "Prefix to the output files", "");
    opt.add_option("-m", "--mesh", "Path to the input obj ground truth", "");
    opt.add_option("-pv", "--pov", "Path to the point of views", "");
    opt.add_option("-pt", "--pointCloud", "Path to the input point cloud.", "");
    opt.add_option("-vs", "--voxelSide", "Voxel side size in meters", "0.07");
    opt.add_option("-mn", "--max-nodes", "Max number of nodes per split", "30000");
    opt.add_option("-v", "--verbose", "Verbosity trigger");

    //Parsing options
    bool correctParsing = opt.parse_options(argc, argv);
    if (!correctParsing)
        return EXIT_FAILURE;

    if (op::str2bool(opt["-h"])) {
        opt.show_help();
        return 0;
    }

    if(opt["-pv"].empty())
    {
        cerr << "Point of views obj file (-pv) required !" << endl;
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

    const string outputPath = opt["-o"][opt["-o"].size() - 1] == '/' ? opt["-o"] : opt["-o"] + '/';
    const string prefix = opt["-pr"];
    const string gtPath = opt["-m"];
    const string pointOfViewPath = opt["-pv"];
    const string pointCloudPath = opt["-pt"];
    const double voxelSide = op::str2double(opt["-vs"]);
    const int maxNodes = op::str2int(opt["-mn"]);
    bool verbose = op::str2bool(opt["-v"]);

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");
    if(verbose)
        for(int i=0; i < classesWithColor.size(); i++)
            cout << "Class " << i << " is " << get<0>(classesWithColor[i]) << endl;

    // Load the ground truth
    cout << "Loading ground truth..." << endl;
    auto shapesAndClasses = loadTreesFromObj(gtPath, classesWithColor);
    cout << "Ground truth loaded." << endl;

    // Loading points with label
    pair<vector<Point>, vector<int>> pointsWithLabel;
    if(!pointCloudPath.empty())
        pointsWithLabel = loadPointsWithLabel(pointCloudPath);

    // Point of views
    vector<Point> pointOfViews = loadPointCloudObj(pointOfViewPath);

    // Make splits
    int nbSplit = splitArrangementInVoxels(shapesAndClasses, pointOfViews, pointsWithLabel.first,
                                           pointsWithLabel.second, voxelSide, classesWithColor.size(),
                                           outputPath + prefix, maxNodes, verbose);

    cout << "Made " << nbSplit << " out of model " << gtPath << endl;
    return 0;

}