// External
#include "OptionParser/option_parser.h"

// Own
#include "RegionGrowing.h"


int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input obj mesh", "");
    opt.add_option("-o", "--output", "Path to the output directory", ".");
    opt.add_option("-ep", "--epsilon_point", "Threshold on point for a triangle to belong to a class", "0.01");
    opt.add_option("-en", "--epsilon_normal", "Threshold on normal for a triangle to belong to a class", "0.005");
    opt.add_option("-sp", "--sigma_point", "Threshold on point for two classes to be matched", "0.04");
    opt.add_option("-sn", "--sigma_normal", "Threshold on normal for two classes to be matched", "0.001");

    //Parsing options
    bool correctParsing = opt.parse_options(argc, argv);
    if (!correctParsing)
        return EXIT_FAILURE;

    if (op::str2bool(opt["-h"])) {
        opt.show_help();
        return 0;
    }

    if(opt["-i"].empty()) {
        cerr << "Input .obj file (-i) is required!" << endl;
        return EXIT_FAILURE;
    }

    string inPath = opt["-i"];
    string outPath = opt["-o"][opt["-o"].size()-1] == '/' ? opt["-o"] : opt["-o"] + "/";
    double epsilonPoint = op::str2double(opt["-ep"]);
    double epsilonNormal = op::str2double(opt["-en"]);
    double sigmaPoint = op::str2double(opt["-sp"]);
    double sigmaNormal = op::str2double(opt["-sn"]);

    RegionGrowing regionGrowing(inPath, epsilonPoint, epsilonNormal, sigmaPoint, sigmaNormal, true);
    int nbOfPrimitives = regionGrowing.run();
    regionGrowing.saveAsObj(outPath + "coloredGtPlanes.obj");
    regionGrowing.saveAsJson(outPath + "coloredGtPlanes.json", true);
    return 0;
}