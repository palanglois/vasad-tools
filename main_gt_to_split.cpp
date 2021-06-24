// External
#include "OptionParser/option_parser.h"

// STD
#include <memory>

// Own
#include "VoxelArrangement.h"
#include "plyInline.h"

using namespace std;
using namespace tinyply;

int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input gt mesh (obj)", "");
    opt.add_option("-o", "--output", "Path to the output directory", "");
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

    const string gtPath = opt["-i"];
    const string outputPath = opt["-o"][opt["-o"].size() - 1] == '/' ? opt["-o"] : opt["-o"] + '/';
    const bool verbose = op::str2bool(opt["-v"]);

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");
    if (verbose)
        for (int i = 0; i < classesWithColor.size(); i++)
            cout << endl << "Class " << i << " is " << get<0>(classesWithColor[i]);
    cout << endl;

    // Load the ground truth
    cout << "Loading ground truth..." << endl;
    auto shapesAndClasses = loadTreesFromObj(gtPath, classesWithColor);
    cout << "Ground truth loaded." << endl;

    for(int i=0; i < shapesAndClasses.size(); i++)
    {
        const auto& shape = shapesAndClasses[i];
        // Filling points and faces
        vector<float3> _points;
        vector<uint3> _faces;
        int faceItr = 0;
        for(const auto& tri: get<0>(shape))
        {
            for(int j=0; j < 3; j++)
                _points.push_back({float(tri[j].x()), float(tri[j].y()), float(tri[j].z())});
            _faces.push_back({uint32_t(faceItr), uint32_t(faceItr + 1), uint32_t(faceItr + 2)});
            faceItr += 3;
        }

        // Open the output stream in binary mode
        string paddedIdx = padTo(to_string(i), 8);
        string path = outputPath + paddedIdx + "_" + to_string(get<1>(shape)) + ".ply";
        filebuf fb_binary;
        fb_binary.open(path, ios::out | ios::binary);
        ostream outstream_binary(&fb_binary);
        if (outstream_binary.fail()) throw runtime_error("failed to open " + path);

        // Creating the properties of the output file
        PlyFile out_file;

        // Points
        out_file.add_properties_to_element("vertex", { "x", "y", "z" },
                                           Type::FLOAT32, _points.size(), reinterpret_cast<uint8_t*>(_points.data()), Type::INVALID, 0);

        // Faces
        out_file.add_properties_to_element("face", { "vertex_indices" },
                                           Type::UINT32, _faces.size(), reinterpret_cast<uint8_t*>(_faces.data()), Type::UINT8, 3);

        // Write the output binary file
        out_file.write(outstream_binary, true);

    }

    return 0;
}