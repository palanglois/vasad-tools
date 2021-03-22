// External
#include "OptionParser/option_parser.h"

#include "iogeometry.h"

int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input txt point cloud with normals", "");
    opt.add_option("-o", "--output", "Path to the output directory", ".");

    //Parsing options
    bool correctParsing = opt.parse_options(argc, argv);
    if (!correctParsing)
        return EXIT_FAILURE;

    if (op::str2bool(opt["-h"])) {
        opt.show_help();
        return 0;
    }

    if (opt["-i"].empty()) {
        cerr << "Input txt point cloud (-i) must be specified!" << endl;
        return EXIT_FAILURE;
    }

    const string inPath = opt["-i"];
    string outPath = opt["-o"][opt["-o"].size() - 1] == '/' ? opt["-o"] : opt["-o"] + "/";


    //Loading the obj data
    ifstream inputStream(inPath.c_str());
    if (!inputStream) {
        cerr << "Could not load file located at : " << inPath << endl;
    }
    vector<Point> points;
    vector<Vector> normals;
    string currentLine;
    while (getline(inputStream, currentLine)) {
        stringstream ss(currentLine);
        string firstCaracter;
        float vx, vy, vz, nx, ny, nz;
        ss >> vx >> vy >> vz >> nx >> ny >> nz;
        points.emplace_back(vx, vy, vz);
        normals.emplace_back(nx, ny, nz);
    }
    vector<double> features = getThicknessFeatures(points, normals);

    string outPathFile = outPath + "thickness.txt";
    ofstream outputStream(outPathFile.c_str());

    for (const auto &feature: features)
        outputStream << feature << endl;
    outputStream.close();

    return 0;

}