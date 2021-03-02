// External
#include "OptionParser/option_parser.h"

// Own
#include "lib/iogeometry.h"

using namespace std;

pair<vector<Point>, map<Point, int>> sampleMesh(const pair<vector<Triangle>, TriangleClassMap> &meshWithLabel,
                                                const vector<classKeywordsColor> &classes, int nbSamples)
{
    // Building the cumulative histogram of the triangle areas
    double accumulator = 0.;
    vector<double> cumulHisto;
    for(const auto& triangle: meshWithLabel.first)
    {
        accumulator += sqrt(triangle.squared_area());
        cumulHisto.push_back(accumulator);
    }

    // Normalizing the histogram
    for(double &cumulatedArea: cumulHisto)
        cumulatedArea /= accumulator;

    // Actual sampling
    vector<Point> sampledPoints;
    map<Point, int> pointClasses;
#pragma omp parallel for
    for(int i=0; i < nbSamples; i++)
    {
        // Select a random triangle according to the areas distribution
        double r = ((double) rand() / (RAND_MAX));
        size_t foundIndex = 0;
        for (size_t j = 0; j < cumulHisto.size() && r > cumulHisto[j]; j++)
            foundIndex = j + 1;

        // Draw a random point in this triangle
        double r1 = ((double) rand() / (RAND_MAX));
        double r2 = ((double) rand() / (RAND_MAX));
        const Point& A = meshWithLabel.first[foundIndex][0];
        const Point& B = meshWithLabel.first[foundIndex][1];
        const Point& C = meshWithLabel.first[foundIndex][2];
        Point P = CGAL::ORIGIN + (1 - sqrt(r1)) * (A  - CGAL::ORIGIN)
                  + (sqrt(r1) * (1 - r2)) * (B - CGAL::ORIGIN)
                  + (sqrt(r1) * r2) * (C - CGAL::ORIGIN);
#pragma omp critical
        {
            sampledPoints.push_back(P);
            pointClasses[P] = meshWithLabel.second.at(meshWithLabel.first[foundIndex]);
        }
    }
    return make_pair(sampledPoints, pointClasses);
}

int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input obj mesh", "");
    opt.add_option("-o", "--output", "Path to the output directory", ".");
    opt.add_option("-n", "--number", "Number of ray to shoot per point of view", "10000");


    //Parsing options
    bool correctParsing = opt.parse_options(argc, argv);
    if (!correctParsing)
        return EXIT_FAILURE;

    if (op::str2bool(opt["-h"])) {
        opt.show_help();
        return 0;
    }

    const string inputPath = opt["-i"];
    const string outPath = opt["-o"][opt["-o"].size()-1] == '/' ? opt["-o"] : opt["-o"] + "/";

    int nbShoot = op::str2int(opt["-n"]);

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");

    // Load 3D model
    vector<Triangle> triangles;
    pair<vector<Triangle>, TriangleClassMap> trianglesAndClasses = loadTrianglesFromObj(inputPath, classesWithColor);

    // Sample the mesh
    pair<vector<Point>, map<Point, int>> samplesWithLabel = sampleMesh(trianglesAndClasses, classesWithColor, nbShoot);

    // Adapt the mapping
    map<Point, vector<double>> adaptedMapping;
    for(const auto& mapElem: samplesWithLabel.second)
        adaptedMapping[mapElem.first] = {(double) mapElem.second};
    pair<vector<Point>, map<Point, vector<double>>> samplesWithAdaptedLabels = make_pair(samplesWithLabel.first, adaptedMapping);

    // Save it
    savePointsAsObjWithLabel(samplesWithAdaptedLabels, outPath + "samplesWithLabel.obj");
    return 0;
}
