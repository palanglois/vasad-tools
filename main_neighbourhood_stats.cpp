// External
#include "OptionParser/option_parser.h"

// Own
#include "lib/iogeometry.h"

using namespace std;


int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input json arrangement", "");
    opt.add_option("-r", "--radius", "Initial radius", "0.1");
    opt.add_option("-n", "--nb_radius", "Number of radius (i.e nb od layers)", "20");

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
    const double radius = op::str2double(opt["-r"]);
    const int nbNeighbourhoods = op::str2int(opt["-n"]) - 1; // The first layer is for edge feature transfer
    const double PI = 3.141592;

    // Loading plane arrangement
    PlaneArrangement currentArrangement(inputPath);

    // Computing the distances
    vector<double> distances = {radius};
    double volume = PI*4.*pow(radius, 3)/3.;
    for(int i=0; i < nbNeighbourhoods - 1; i++)
    {
        double newRadius = pow((3. * volume) / (4. * PI) + pow(distances[i], 3), 1./3.);
        distances.push_back(newRadius);
    }

    cout << "Computed radius: " ;
    for(double rad: distances)
        cout << rad << " ";
    cout << endl;

    vector<UniqueEdges> neighbourhoods = currentArrangement.euclidianNeighbourhoods(distances);

    vector<double> distForDisplay = {0.};
    transform(distances.begin(), distances.end(), back_inserter(distForDisplay), [](double v){return v;});

    for(int i=0; i < distances.size(); i++)
        cout << "Number of nodes between " << distForDisplay[i] << "m and " << distForDisplay[i + 1] << "m: "
        << neighbourhoods[i].size() << endl;

    return 0;
}