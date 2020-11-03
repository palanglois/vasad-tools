// External
#include "OptionParser/option_parser.h"

// Own
#include "lib/iogeometry.h"

int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input scan obj file", "");
    opt.add_option("-o", "--output", "Path to the output obj file", "");

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
        opt.show_help();
        return EXIT_FAILURE;
    }

    const string inputPath = opt["-i"];
    const string outputPath = opt["-o"];

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");

    // Loading points with label
    pair<vector<Point>, vector<int>> pointsWithLabel = loadPointsWithLabel(inputPath);

    // Compute colors
    vector<colorTuple> colors;
    int voidLabel = classesWithColor.size();
    for(auto label: pointsWithLabel.second)
        if(label == voidLabel)
            colors.emplace_back(127, 127, 127);
        else
            colors.push_back(get<2>(classesWithColor[label]));

    // Output resulting file
    savePointsAsObjWithColors(pointsWithLabel.first, colors, outputPath);

    return 0;
}