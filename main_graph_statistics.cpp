#include <iostream>

// External
#include "OptionParser/option_parser.h"

// Own
#include "iogeometry.h"

using namespace std;


int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input plane arrangement", "");

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

    const string inputPath = opt["-i"];

    map<int, int> cell2label;
    vector<bool> labels;
    Arrangement arr;
    loadArrangement(inputPath, arr, cell2label, labels);

    return 0;
}