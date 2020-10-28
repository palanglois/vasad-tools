// External
#include "OptionParser/option_parser.h"

// Own
#include "lib/graphStats.h"

using namespace std;
using Json = nlohmann::json;

string padTo(string str, const size_t num, const char paddingChar = '0')
{
    if(num > str.size())
        str.insert(0, num - str.size(), paddingChar);
    return str;
}

int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input plane arrangement", "");
    opt.add_option("-o", "--output", "Path to the output folder", "");
    opt.add_option("-m", "--mesh", "Path to the input obj ground truth", "");
    opt.add_option("-p", "--proba", "Proba of giving the empty occupency information for each empty cell", "1.");
    opt.add_option("-g", "--geom", "Adds the bounding box dimensions of each cell as a node feature");
    opt.add_option("-s", "--step", "Subdivision step in meters", "4");
    opt.add_option("-mn", "--max-nodes", "Max number of nodes per split", "10000");
    opt.add_option("-mp", "--max-planes", "Max number of planes per split", "250");
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
    const string outputPath = opt["-o"][opt["-o"].size() - 1] == '/' ? opt["-o"] : opt["-o"] + '/';
    const double proba = op::str2double(opt["-p"]);
    const double step = op::str2double(opt["-s"]);
    const bool geom = op::str2bool(opt["-g"]);
    const int nbSamplesPerCell = 40;
    const int maxNodes = op::str2int(opt["-mn"]);
    const int maxNbPlanes = op::str2int(opt["-mp"]);
    bool verbose = op::str2bool(opt["-v"]);

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");

    // Load the ground truth
    const string gtPath = opt["-m"];
    cout << "Loading ground truth..." << endl;
    auto shapesAndClasses = loadTreesFromObj(gtPath, classesWithColor);
    cout << "Ground truth loaded." << endl;

    // Loading plane arrangement
    PlaneArrangement currentArrangement(inputPath);
    const map<int, int> &cell2label = currentArrangement.cell2label();
    const CGAL::Bbox_3 &bbox = currentArrangement.bbox();

    // Make splits
    vector<Json> allSplits = splitArrangementInBatch(currentArrangement, shapesAndClasses, classesWithColor.size(),
                                                     step, maxNodes, maxNbPlanes, nbSamplesPerCell, proba, geom, verbose);
    for(int i=0; i < allSplits.size(); i++)
    {
        ofstream outStream(outputPath + padTo(to_string(i), 4) + ".json");
        outStream << allSplits[i];
        outStream.close();
    }

    return 0;
}