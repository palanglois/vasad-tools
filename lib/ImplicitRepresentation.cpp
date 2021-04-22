#include "ImplicitRepresentation.h"

using namespace std;
using namespace cnpy;
namespace fs = std::experimental::filesystem;

vector<Point> sampleInBbox(const CGAL::Bbox_3 &bbox, int nbSamples)
{

    // Draw points in the arrangement
    vector<Point> sampledPoints(nbSamples);
    default_random_engine generator;
    uniform_real_distribution<double> xDist(bbox.xmin(), bbox.xmax());
    uniform_real_distribution<double> yDist(bbox.ymin(), bbox.ymax());
    uniform_real_distribution<double> zDist(bbox.zmin(), bbox.zmax());
#pragma omp parallel for
    for(int i=0; i < nbSamples; i++)
        sampledPoints[i] = Point(xDist(generator), yDist(generator), zDist(generator));
    return sampledPoints;
}

ImplicitRepresentation::ImplicitRepresentation(const CGAL::Bbox_3 &inBbox, int inNbFilesToGenerate,
                                               int inNbSurfacicPerFiles, int inNbVolumicPerFiles) :
        bbox(inBbox), nbFilesToGenerate(inNbFilesToGenerate), nbSurfacicPerFiles(inNbSurfacicPerFiles),
        nbVolumicPerFiles(inNbVolumicPerFiles) {

}

void ImplicitRepresentation::computeSurfacicFromPointCloud(const vector<Point> &pointCloud,
                                                           const vector<Vector> &normals) {

    for(int i=0; i < pointCloud.size(); i++)
    {
        if(CGAL::do_overlap(pointCloud[i].bbox(), bbox))
        {
            surfacicPoints.push_back({pointCloud[i].x(), pointCloud[i].y(), pointCloud[i].z()});
            surfacicNormals.push_back({normals[i].x(), normals[i].y(), normals[i].z()});
        }
    }
}

void ImplicitRepresentation::computeVolumicPoints(vector<facesLabelName> &labeledShapes, int nbClasses,
                                                  const vector<Point> &sampledPoints, bool verbose) {

    // Label the points
    vector<int> labels = assignLabelToPoints(sampledPoints, labeledShapes, nbClasses, bbox);

    // Concatenate the new data to the corresponding attributes
    for(int i=0; i < sampledPoints.size(); i++)
        volumicPoints.push_back({sampledPoints[i].x(), sampledPoints[i].y(), sampledPoints[i].z()});
    for(int i=0; i < labels.size(); i++)
        occupancies.push_back(labels[i] == nbClasses ? -1 : labels[i]);
    occupancies.insert(occupancies.end(), labels.begin(), labels.end());

}

void ImplicitRepresentation::generateRandomVolumicPoints(vector<facesLabelName> &labeledShapes, int nbClasses,
                                                         bool verbose) {
    vector<Point> sampledPoints = sampleInBbox(bbox, nbVolumicPerFiles * nbFilesToGenerate);
    computeVolumicPoints(labeledShapes, nbClasses, sampledPoints, verbose);
}

int random_num_in_range(int range) {
    int x;

    do {
        x = rand();
    } while (x >= (RAND_MAX - RAND_MAX % range));

    return x % range;
}

// Samples randomly from (b, e) into o, n elements
template<typename It, typename OutIt>
void sampleWithReplacement(It b, It e, OutIt o, size_t n)
{
    // Number of elements in range.
    const size_t s = std::distance(b, e);
    // Generate n samples.
    for(size_t i = 0; i < n; ++i)
    {
        It it = b;
        // Move b iterator random number of steps forward.
        std::advance(it, random_num_in_range(s));
        // Write into output
        *(o++) = *it;
    }
}


void ImplicitRepresentation::save(const string &path) const {

    if(surfacicPoints.empty())
    {
        cout << "Not saving chunk " << path << " because there is no surfacic point in it." << endl;
        return;
    }

    // Make the current directory
    fs::create_directories(path);

    // Surfacic Point Cloud
    string surfacicPcPath = path + "/pointcloud";
    fs::create_directories(surfacicPcPath);
    for(int i=0; i < nbFilesToGenerate; i++)
    {
        // Current file path
        string fileOutPath = surfacicPcPath + "/pointcloud_" + padTo(to_string(i), 2) + ".npz";
        // Subset with replacement
        vector<int> wholeIdx, subIdx;
        wholeIdx.reserve(surfacicPoints.size());
        for(int j=0; j < surfacicPoints.size(); j++)
            wholeIdx.push_back(j);
        sampleWithReplacement(wholeIdx.begin(), wholeIdx.end(), back_inserter(subIdx),
               nbSurfacicPerFiles);
        vector<double> curPoints;
        vector<double> curNormals;
        for(int j=0; j < subIdx.size(); j++)
            for(int k=0; k < 3; k++) {
                curPoints.push_back(surfacicPoints[subIdx[j]][k]);
                curNormals.push_back(surfacicNormals[subIdx[j]][k]);
            }
        npz_save(fileOutPath, "points", &curPoints[0], {(size_t) nbSurfacicPerFiles, 3}, "w");
        npz_save(fileOutPath, "normals", &curNormals[0], {(size_t) nbSurfacicPerFiles, 3}, "a");
    }

    // Volumic Point Cloud
    string volumicPcPath = path + "/points_iou";
    fs::create_directories(volumicPcPath);
    vector<double> zscale = {0};
    for(int i=0; i < nbFilesToGenerate; i++)
    {
        // Current file path
        string fileOutPath = volumicPcPath + "/points_iou_" + padTo(to_string(i), 2) + ".npz";

        vector<double> curPoints;
        vector<unsigned char> curOccupancies;
        string runningBits;
        vector<double> semantic;
        for(int j=0; j < nbVolumicPerFiles; j++)
        {
            // Use the points that have been generated
            for(int k=0; k < 3; k++)
                curPoints.push_back(volumicPoints[nbVolumicPerFiles * i + j][k]);

            // Occupancies
            if(j % 8 == 0 && j != 0)
            {
                auto x = runningBits.c_str();
                bitset<8> b(x);
                curOccupancies.push_back(b.to_ulong());
                runningBits = "";
            }
            runningBits += to_string(int(occupancies[nbVolumicPerFiles * i + j] != -1));
        }
        auto x = runningBits.c_str();
        bitset<8> b(x);
        curOccupancies.push_back(b.to_ulong());
        npz_save(fileOutPath, "points", &curPoints[0], {(size_t) nbVolumicPerFiles, 3}, "w");
        npz_save(fileOutPath, "occupancies", &curOccupancies[0], {curOccupancies.size()}, "a");
        npz_save(fileOutPath, "z_scale", &zscale[0], {1}, "a");
        npz_save(fileOutPath, "semantic", &occupancies[0], {occupancies.size()}, "a");
    }
}

const std::vector<std::vector<double>> &ImplicitRepresentation::getSurfacicPoints() const {
    return surfacicPoints;
}

const std::vector<std::vector<double>> &ImplicitRepresentation::getSurfacicNormals() const {
    return surfacicNormals;
}

const std::vector<std::vector<double>> &ImplicitRepresentation::getVolumicPoints() const {
    return volumicPoints;
}

const std::vector<int> &ImplicitRepresentation::getOccupancies() const {
    return occupancies;
}

int splitBimInImplicit(vector<facesLabelName> &labeledShapes, const vector<Point> &pointOfViews,
                       const vector<Point> &pointCloud, const vector<Vector> &pointCloudNormals,
                       int nbClasses, double bboxSize, int nbFilesToGenerate, int nbSurfacicPerFiles,
                       int nbVolumicPerFiles, const string &path, bool verbose)
{
    // Compute initial bounding box
    CGAL::Bbox_3 initialBbox;
    for(const auto& shape: labeledShapes)
        for(const auto& triangle: get<0>(shape))
            initialBbox += triangle.bbox();

    // Split it into dense bboxes
    vector<CGAL::Bbox_3> allBboxes = splitBigBbox(initialBbox, bboxSize);

    // Generate the chunks
    for(int i=0; i < allBboxes.size(); i++) {
        const CGAL::Bbox_3 &curBbox = allBboxes[i];
        if (verbose) {
            cout << endl << "Bbox \033[1;31m" << i << "\033[0m out of " << allBboxes.size() << endl;
            cout << "Current bbox: " << curBbox << endl;
        }

        // Make an implicit representation structure
        auto implicitRep = ImplicitRepresentation(curBbox, nbFilesToGenerate, nbSurfacicPerFiles,
                                                  nbVolumicPerFiles);

        // Compute the surfacic points
        implicitRep.computeSurfacicFromPointCloud(pointCloud, pointCloudNormals);

        // Discard if we have no surfacic point
        if(implicitRep.getSurfacicPoints().empty()) continue;

        // Compute the volumic points
        implicitRep.generateRandomVolumicPoints(labeledShapes, nbClasses, verbose);

        // Save current chunk
        string outPath(path + padTo(to_string(i), 5));
        implicitRep.save(outPath);
    }
    return allBboxes.size();
}
