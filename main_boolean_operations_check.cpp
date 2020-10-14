// STD
#include <iostream>

// External
#include "OptionParser/option_parser.h"

// Own
#include "lib/BimObj.h"

using namespace std;

int main(int argc, char *argv[])
{
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input obj mesh", "");

    //Parsing options
    bool correctParsing = opt.parse_options(argc, argv);
    if (!correctParsing)
        return EXIT_FAILURE;

    if (op::str2bool(opt["-h"])) {
        opt.show_help();
        return 0;
    }
    const string inputPath = opt["-i"];

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");
    BimObj bimObj;
    bimObj.loadFromObj(inputPath, classesWithColor);

    for(auto &object: bimObj)
    {
        string closed = object.isClosed() ? "closed" : "open";
        cout << object.getName() << " is " << closed << endl;
        if(object.getName() == "IfcRailing/Railing:900mm_Handrail_Only:235499_9990")
            continue;
        Nef_Polyhedron curPol(*object.toCgal());
    }
    cout << "Debug" << endl;

    return 0;
}
