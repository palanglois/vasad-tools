// External
#include "OptionParser/option_parser.h"
#include "json/json.hpp"

// Own
#include "lib/iogeometry.h"

using namespace std;
using Json = nlohmann::json;

Json getVoxelsFromMesh(const vector<Triangle>& triangles, double distance)
{
    CGAL::Bbox_3 bbox;
    for(const auto& triangle: triangles)
        bbox += triangle.bbox();

    Json planesData;

    // Planes
    vector<double> ranges = {bbox.xmax() - bbox.xmin(),
                             bbox.ymax() - bbox.ymin(),
                             bbox.zmax() - bbox.zmin()};
    double totalArea = 0.;
    for(int i=0; i < ranges.size(); i++)
        totalArea += (ceil(ranges[i] / distance) + 1) * ranges[(i + 1) % 3] * ranges[(i + 2) % 3];
    vector<double> mins = {bbox.xmin(), bbox.ymin(), bbox.zmin()};
    vector<double> maxs = {bbox.xmax(), bbox.ymax(), bbox.zmax()};
    vector<Json> normals = {{1., 0., 0.}, {0., 1., 0.}, {0., 0., 1.}};
    vector<Json> pointCloud;
    int faceAccum = 0;
    for(int i=0; i < ranges.size(); i++) {
        int nbPlanes = ceil(ranges[i] / distance);
        Json& normal = normals[i];
        for (int j = 0; j < nbPlanes + 1; j++) {
            double curAxisCoordinate = min(mins[i] + distance * j, maxs[i]);
            Json lowLeft = {0., 0., 0.};
            lowLeft[i] = curAxisCoordinate;
            cout << lowLeft[i] << " " << maxs[i] << endl;
            lowLeft[(i + 1) % 3] = mins[(i + 1) % 3];
            lowLeft[(i + 2) % 3] = mins[(i + 2) % 3];
            pointCloud.push_back(lowLeft);
            Json lowRight = {0., 0., 0.};
            lowRight[i] = curAxisCoordinate;
            lowRight[(i + 1) % 3] = maxs[(i + 1) % 3];
            lowRight[(i + 2) % 3] = mins[(i + 2) % 3];
            pointCloud.push_back(lowRight);
            Json topLeft = {0., 0., 0.};
            topLeft[i] = curAxisCoordinate;
            topLeft[(i + 1) % 3] = mins[(i + 1) % 3];
            topLeft[(i + 2) % 3] = maxs[(i + 2) % 3];
            pointCloud.push_back(topLeft);
            Json topRight = {0., 0., 0.};
            topRight[i] = curAxisCoordinate;
            topRight[(i + 1) % 3] = maxs[(i + 1) % 3];
            topRight[(i + 2) % 3] = maxs[(i + 2) % 3];
            pointCloud.push_back(topRight);
            Json planeData = {{"normal", normal},
                               {"inlier", lowLeft}};
            planeData["area"] = (ranges[(i + 1) % 3] * ranges[(i + 2) % 3]);

            // Triangulating the plane
            vector<vector<int>> faces = {{faceAccum, faceAccum + 1, faceAccum + 2},
                                         {faceAccum + 1, faceAccum + 2, faceAccum + 3}};
            planeData["faces"] = faces;
            faceAccum += 4;
            planesData.push_back(planeData);
        }
    }
    // Sort the planes from the biggest to the smallest
    sort(planesData.begin(), planesData.end(),
         [](const Json & a, const Json & b) -> bool
         {
             return a["area"] > b["area"];
         });

    // Compute cumulated percentage of reconstructed surface
    double areaCumulator = 0.;
    for(auto &plane: planesData) {
        areaCumulator += plane["area"].get<double>();
        plane["cumulatedPercentage"] = areaCumulator / totalArea;
        cout << plane["cumulatedPercentage"] << endl;
    }

    // Map
    Json map;
    map["NOMAP"] = 0;

    // Labels
    Json labels;
    labels.push_back(true);

    // Number of planes
    Json nbPlanes = planesData.size();

    // Compile data
    Json outputData = {{"planes",   planesData},
                       {"map",      map},
                       {"labels",   labels},
                       {"bbox",     bbox},
                       {"nbPlanes", nbPlanes}};

    outputData["pointCloud"] = pointCloud;

    cout << "Bounding box: " << bbox << endl;
    cout << "Number of planes: " << planesData.size() << endl;

    return outputData;
}

int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input obj mesh", "");
    opt.add_option("-o", "--output", "Path to the output directory", ".");
    opt.add_option("-d", "--distance", "Side distance of a voxel", "0.1");

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
        cerr << "Input obj mesh path (-i) is required!" << endl;
        return EXIT_FAILURE;
    }

    const string inputPath = opt["-i"];
    const string outPath = opt["-o"][opt["-o"].size()-1] == '/' ? opt["-o"] : opt["-o"] + "/";
    double distance = op::str2double(opt["-d"]);

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");

    // Load 3D model
    pair<vector<Triangle>, TriangleClassMap> trianglesAndClasses = loadTrianglesFromObj(inputPath, classesWithColor);

    // Compute voxels
    Json jsonVoxels = getVoxelsFromMesh(trianglesAndClasses.first, distance);

    // Save results as a json file
    const string outPathFile = outPath + "voxels.json";
    ofstream outStream(outPathFile);
    outStream << jsonVoxels;
    outStream.close();

    return 0;
}