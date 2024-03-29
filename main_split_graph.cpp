// External
#include "OptionParser/option_parser.h"

// Own
#include "lib/graphStats.h"

using namespace std;
using Json = nlohmann::json;

int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input plane arrangement", "");
    opt.add_option("-o", "--output", "Path to the output folder", "");
    opt.add_option("-pr", "--prefix", "Prefix to the output files", "");
    opt.add_option("-m", "--mesh", "Path to the input obj ground truth", "");
    opt.add_option("-pv", "--pov", "[Optional] Path to the point of views (if visibility)", "");
    opt.add_option("-vt", "--vis_threshold", "Threshold on visibility ", "0.7");
    opt.add_option("-mg", "--merging", "Enables node merging", "");
    opt.add_option("-g", "--geom", "Adds the bounding box dimensions of each cell as a node feature");
    opt.add_option("-pt", "--pointCloud", "Path to the input point cloud. If set, edge features are computed thanks to it.", "");
    opt.add_option("-s", "--step", "Subdivision step in meters", "4");
    opt.add_option("-mn", "--max-nodes", "Max number of nodes per split", "10000");
    opt.add_option("-mp", "--max-planes", "Max number of planes per split", "250");
    opt.add_option("-r", "--ration-recons", "Ratio of the total surface reconstructed", "0.98");
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

    const string inputPath = opt["-i"];
    const string pointCloudPath = opt["-pt"];
    const string outputPath = opt["-o"][opt["-o"].size() - 1] == '/' ? opt["-o"] : opt["-o"] + '/';
    const string prefix = opt["-pr"];
    const double visThreshold = op::str2double(opt["-vt"]);
    const double step = op::str2double(opt["-s"]);
    const bool geom = op::str2bool(opt["-g"]);
    const int nbSamplesPerCell = 40;
    const int maxNodes = op::str2int(opt["-mn"]);
    const int maxNbPlanes = op::str2int(opt["-mp"]);
    const double ratioReconstructed = op::str2double(opt["-r"]);
    const string pointOfViewPath = opt["-pv"];
    const bool withPov = !opt["-pv"].empty();
    const bool merging = op::str2bool(opt["-mg"]);
    bool verbose = op::str2bool(opt["-v"]);

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");
    if(verbose)
        for(int i=0; i < classesWithColor.size(); i++)
            cout << "Class " << i << " is " << get<0>(classesWithColor[i]) << endl;

    // Load the ground truth
    const string gtPath = opt["-m"];
    cout << "Loading ground truth..." << endl;
    auto shapesAndClasses = loadTreesFromObj(gtPath, classesWithColor);
    cout << "Ground truth loaded." << endl;

    // Loading plane arrangement
    PlaneArrangement currentArrangement(inputPath);
    const map<int, int> &cell2label = currentArrangement.cell2label();
    const CGAL::Bbox_3 &bbox = currentArrangement.bbox();

    // Loading points with label
    pair<vector<Point>, vector<int>> pointsWithLabel;
    if(!pointCloudPath.empty())
        pointsWithLabel = loadPointsWithLabel(pointCloudPath);

    // Point of views
    vector<Point> pointOfViews(0);
    if(withPov)
        pointOfViews = loadPointCloudObj(pointOfViewPath);

    // Make splits
    int nbSplit = splitArrangementInBatch(currentArrangement, shapesAndClasses, outputPath + prefix,
                                          classesWithColor.size(), step, maxNodes, pointsWithLabel, pointOfViews,
                                          maxNbPlanes, nbSamplesPerCell, visThreshold, geom, ratioReconstructed,
                                          merging, verbose);
    
    cout << "Made " << nbSplit << " out of model " << inputPath << endl;
    return 0;
}