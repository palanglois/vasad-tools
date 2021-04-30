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
        nbVolumicPerFiles(inNbVolumicPerFiles), areVolumicDataComputed(false) {

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

inline int intersectLineAndShape(const Tree &tree, const Point &point, const Vector &direction1,
                                 Point &firstIntersection, Point &secondIntersection, Vector &firstNormal,
                                 Vector &secondNormal)
{
    Vector direction2 = -1. * direction1;

    // 1st intersection
    Ray curRayOne(point, direction1);
    Ray_intersection intersection = tree.first_intersection(curRayOne);
    if (!intersection) return 1;
    if (!boost::get<Point>(&(intersection->first))) return 1;
    const Point *interOne = boost::get<Point>(&(intersection->first));
    Triangle& curTri = *boost::get<Primitive_id>(intersection->second);
    Vector curNormal = CGAL::normal(curTri.vertex(0), curTri.vertex(1), curTri.vertex(2));
    curNormal /= sqrt(curNormal.squared_length());

    // 2nd intersection
    Ray curRayTwo(point, direction2);
    Ray_intersection intersectionTwo = tree.first_intersection(curRayTwo);
    if (!intersectionTwo) return 1;
    if (!boost::get<Point>(&(intersectionTwo->first))) return 1;
    const Point *interTwo = boost::get<Point>(&(intersectionTwo->first));
    Triangle& curTriTwo = *boost::get<Primitive_id>(intersectionTwo->second);
    Vector curNormalTwo = CGAL::normal(curTriTwo.vertex(0), curTriTwo.vertex(1), curTriTwo.vertex(2));
    curNormalTwo /= sqrt(curNormalTwo.squared_length());

    firstIntersection = *interOne;
    secondIntersection = *interTwo;
    firstNormal = curNormal;
    secondNormal = curNormalTwo;

    return 0;
}

int processDirection(const Tree &tree, const Point &point, const Vector &direction1, double &bestSquaredDistance,
        Point &bestPointOne, Point &bestPointTwo, Vector &bestNormalOne, Vector &bestNormalTwo)
{
    Point firstIntersection, secondIntersection;
    Vector firstNormal, secondNormal;
    int error = intersectLineAndShape(tree, point, direction1, firstIntersection, secondIntersection,
                                      firstNormal, secondNormal);
    if(error == 1) return 1;

    // Process current distance
    double curSquaredDistance = (secondIntersection - firstIntersection).squared_length();
    if(curSquaredDistance < bestSquaredDistance)
    {
        bestSquaredDistance = curSquaredDistance;
        bestPointOne = firstIntersection;
        bestPointTwo = secondIntersection;
        bestNormalOne = firstNormal;
        bestNormalTwo = secondNormal;
    }
    return 0;
}

/***
 * Default orientation: the 1st non-zero coordinate of axis (with threshold epsilon) should be positive
 * ***/
void axisOrientation(Vector &axis, const double epsilon)
{
    if(abs(axis.x()) > epsilon) {
        if (axis.x() < 0.)
            axis *= -1.;
    }
    else
    {
        if(abs(axis.y()) > epsilon) {
            if(axis.y() < 0.)
                axis *= -1;
        }
        else
        {
            if(axis.z() < 0.)
                axis *= -1;
        }
    }
}

void ImplicitRepresentation::computeBoxes(vector<facesLabelName> &labeledShapes, int nbClasses,
                                          const vector<Point> &sampledPoints, int nbShoots) {
    if(areVolumicDataComputed) {
        cerr << "Error: the volumic data have already been computed!" << endl;
        return;
    }

    const double epsilon = 1e-5;

    // Init the boxes
    boxes = vector<vector<double>>(sampledPoints.size(), vector<double>(10, -1.));

    // Compute the labels and store them
    vector<CGAL::Bbox_3> bboxes = computeShapeBboxes(labeledShapes);
    vector<int> labels = assignLabelToPointsWithBboxes(sampledPoints, labeledShapes, nbClasses, bbox, bboxes);
    storeVolumicPoints(sampledPoints, labels, nbClasses);

    // Normal distribution
    normal_distribution<double> normalDist(0., 1.);
    default_random_engine generator(random_device{}());

    Tree tree;
    auto tqLabeledShapes = tq::trange(labeledShapes.size());
    tqLabeledShapes.set_prefix("Computing box for each point: ");
    for (int j : tqLabeledShapes)
    {
        if(!CGAL::do_overlap(bboxes[j], bbox)) continue;
        tree.rebuild(get<0>(labeledShapes[j]).begin(), get<0>(labeledShapes[j]).end());
#pragma omp parallel for schedule(static)
        for(int i=0; i < sampledPoints.size(); i++) {
            const Point& point = sampledPoints[i];
            if(!CGAL::do_overlap(point.bbox(), bboxes[j])) continue;
            if(labels[i] == nbClasses) continue; // Void
            // 1 - Find smallest dimension axisOne of the current box
            // 1a - Shoot in random directions
            auto bestSquaredDistance = DBL_MAX;
            Point bestPointOne, bestPointTwo = Point(0., 0., 0.);
            Vector bestNormalOne, bestNormalTwo = Vector(0., 0., 0.);
            bool isSpaceIntersectValid = false;
            for(int k=0; k < nbShoots; k++) {
                Vector direction1(normalDist(generator), normalDist(generator), normalDist(generator));
                if(processDirection(tree, point, direction1, bestSquaredDistance, bestPointOne, bestPointTwo,
                                 bestNormalOne, bestNormalTwo) == 0) isSpaceIntersectValid = true;
            }
            // 1b - Refinement thanks to the obtained normals;
            vector<Vector> additionalDirections = {bestNormalOne, bestNormalTwo};
            for (const auto &direction: additionalDirections)
                if(processDirection(tree, point, direction, bestSquaredDistance, bestPointOne, bestPointTwo,
                                 bestNormalOne, bestNormalTwo) == 0) isSpaceIntersectValid = true;
            if(!isSpaceIntersectValid) continue;
            Vector axisOne = bestPointTwo - bestPointOne;
            axisOne = axisOne / sqrt(axisOne.squared_length());
            axisOrientation(axisOne, epsilon);

            // 2 - Find the smallest direction in the orthogonal plane to axisOne (similarly as before)
            // 2a - Shoot in random directions in the plane
            PlaneCgal orthogonalPlane(point, axisOne);
            auto bestSquaredDistancePlane = DBL_MAX;
            Point bestPointOnePlane, bestPointTwoPlane = Point(0., 0., 0.);
            Vector bestNormalOnePlane, bestNormalTwoPlane = Vector(0., 0., 0.);
            bool isPlaneIntersectValid = false;
            for(int k=0; k < nbShoots; k++) {
                Vector direction1(normalDist(generator), normalDist(generator), normalDist(generator));
                // Project direction1 on the plane
                Vector projectedDirection = orthogonalPlane.projection(point + direction1) - point;
                if(projectedDirection.squared_length() < 0.001) continue;
                if(processDirection(tree, point, projectedDirection, bestSquaredDistancePlane,
                                 bestPointOnePlane, bestPointTwoPlane,
                                 bestNormalOnePlane, bestNormalTwoPlane) == 0)
                    isPlaneIntersectValid = true;
            }
            // 2b - Refinement thanks to the obtained normals;
            vector<Vector> additionalDirectionsPlane = {bestNormalOnePlane, bestNormalTwoPlane};
            for (const auto &direction: additionalDirectionsPlane) {
                Vector projectedDirection = orthogonalPlane.projection(point + direction) - point;
                if(projectedDirection.squared_length() < 0.001) continue;
                if(processDirection(tree, point, projectedDirection, bestSquaredDistancePlane,
                                 bestPointOnePlane, bestPointTwoPlane,
                                 bestNormalOnePlane, bestNormalTwoPlane) == 0)
                    isPlaneIntersectValid = true;
            }
            if(!isPlaneIntersectValid) continue;
            Vector axisTwo = bestPointTwoPlane - bestPointOnePlane;
            axisTwo = axisTwo / sqrt(axisTwo.squared_length());
            axisOrientation(axisTwo, epsilon);
            // 3 - The last direction is obtained thanks to the cross product
            Vector axisThree = CGAL::cross_product(axisOne, axisTwo);
            auto bestSquaredDistanceLine = DBL_MAX;
            Point bestPointOneLine, bestPointTwoLine = Point(0., 0., 0.);
            Vector bestNormalOneLine, bestNormalTwoLine = Vector(0., 0., 0.);
            if (processDirection(tree, point, axisThree, bestSquaredDistanceLine,
                                 bestPointOneLine, bestPointTwoLine,
                                 bestNormalOneLine, bestNormalTwoLine) != 0) continue;


            // 4 - Encode the box
            vector<double> &currentBox = boxes[i];
            // Scaling
            currentBox[0] = sqrt(bestSquaredDistance);
            currentBox[1] = sqrt(bestSquaredDistancePlane);
            currentBox[2] = sqrt(bestSquaredDistanceLine);
            // Translation (intersection of 3 planes)
            PlaneCgal planeOne(CGAL::ORIGIN + ((bestPointOne + (bestPointTwo - CGAL::ORIGIN)) - CGAL::ORIGIN) / 2,
                               bestPointTwo - bestPointOne);
            PlaneCgal planeTwo(
                    CGAL::ORIGIN + ((bestPointOnePlane + (bestPointTwoPlane - CGAL::ORIGIN)) - CGAL::ORIGIN) / 2,
                    bestPointTwoPlane - bestPointOnePlane);
            PlaneCgal planeThree(
                    CGAL::ORIGIN + ((bestPointOneLine + (bestPointTwoLine - CGAL::ORIGIN)) - CGAL::ORIGIN) / 2,
                    bestPointTwoLine - bestPointOneLine);
            CGAL::cpp11::result_of<Intersect(PlaneCgal, PlaneCgal)>::type firstIntersect = intersection(planeOne,
                                                                                                        planeTwo);
            assert(firstIntersect);
            const Line *interLine = boost::get<Line>(&*firstIntersect);
            CGAL::cpp11::result_of<Intersect(Line, PlaneCgal)>::type secondIntersect = intersection(*interLine,
                                                                                                    planeThree);
            assert(secondIntersect);
            const Point *center = boost::get<Point>(&*secondIntersect);
            currentBox[7] = center->x();
            currentBox[8] = center->y();
            currentBox[9] = center->z();
            // Rotation
            Eigen::Matrix3f mat;
            mat(0, 0) = axisOne.x();
            mat(1, 0) = axisOne.y();
            mat(2, 0) = axisOne.z();
            mat(0, 1) = axisTwo.x();
            mat(1, 1) = axisTwo.y();
            mat(2, 1) = axisTwo.z();
            mat(0, 2) = axisThree.x();
            mat(1, 2) = axisThree.y();
            mat(2, 2) = axisThree.z();
            Eigen::Quaternionf q(mat);
            currentBox[3] = q.w();
            currentBox[4] = q.vec().x();
            currentBox[5] = q.vec().y();
            currentBox[6] = q.vec().z();
        }
    }
    areVolumicDataComputed = true;
}

void ImplicitRepresentation::storeVolumicPoints(const vector<Point> &sampledPoints, const vector<int> &labels, int nbClasses)
{
    // Concatenate the new data to the corresponding attributes
    for(int i=0; i < sampledPoints.size(); i++)
        volumicPoints.push_back({sampledPoints[i].x(), sampledPoints[i].y(), sampledPoints[i].z()});
    for(int i=0; i < labels.size(); i++)
        occupancies.push_back(labels[i] == nbClasses ? -1 : labels[i]);
}

void ImplicitRepresentation::computeVolumicPoints(vector<facesLabelName> &labeledShapes, int nbClasses,
                                                  const vector<Point> &sampledPoints, bool verbose) {
    if(areVolumicDataComputed) {
        cerr << "Error: the volumic data have already been computed!" << endl;
        return;
    }

    // Label the points
    vector<int> labels = assignLabelToPoints(sampledPoints, labeledShapes, nbClasses, bbox);

    // Store it
    storeVolumicPoints(sampledPoints, labels, nbClasses);

    areVolumicDataComputed = true;

}

void ImplicitRepresentation::generateRandomVolumicPoints(vector<facesLabelName> &labeledShapes, int nbClasses,
                                                         int nbBoxShoots, bool verbose) {
    vector<Point> sampledPoints = sampleInBbox(bbox, nbVolumicPerFiles * nbFilesToGenerate);
    if(nbBoxShoots != -1)
        computeBoxes(labeledShapes, nbClasses, sampledPoints, nbBoxShoots);
    else
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

void ImplicitRepresentation::normalizeClouds() {
    auto normalize = [](double x, double minBbox, double maxBbox) {
        return (2.*x - maxBbox - minBbox) / (2.*(maxBbox - minBbox));
    };

    // Surfacic points
    for(auto &point: surfacicPoints)
    {
        point[0] = normalize(point[0], bbox.xmin(), bbox.xmax());
        point[1] = normalize(point[1], bbox.ymin(), bbox.ymax());
        point[2] = normalize(point[2], bbox.zmin(), bbox.zmax());
    }

    // Volumic points
    for(auto &point: volumicPoints)
    {
        point[0] = normalize(point[0], bbox.xmin(), bbox.xmax());
        point[1] = normalize(point[1], bbox.ymin(), bbox.ymax());
        point[2] = normalize(point[2], bbox.zmin(), bbox.zmax());
    }

    // Boxes
    for(auto &box: boxes)
    {
        // Scaling (be careful, this may not work if the scaling is anisotropic)
        box[0] = box[0] / (bbox.xmax() - bbox.xmin());
        box[1] = box[1] / (bbox.ymax() - bbox.ymin());
        box[2] = box[2] / (bbox.zmax() - bbox.zmin());

        // Translation
        box[7] = box[7] / (bbox.xmax() - bbox.xmin());
        box[8] = box[8] / (bbox.ymax() - bbox.ymin());
        box[9] = box[9] / (bbox.zmax() - bbox.zmin());
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

    // Parameters of the current chunk
    string paramsPath = path + "/item_dict.npz";
    vector<double> curBbox = {bbox.xmin(), bbox.ymin(), bbox.zmin(), bbox.xmax(), bbox.ymax(), bbox.zmax()};
    npz_save(paramsPath, "bbox", &curBbox[0], {6}, "w");

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

const std::vector<std::vector<double>>& ImplicitRepresentation::getBoxes() const {
    return boxes;
}

void saveLists(const vector<string> &allChunks, const string &outputPath)
{
    size_t firstThreshold = size_t(allChunks.size() * 0.75);
    size_t secondThreshold = size_t(allChunks.size() * 0.80);

    vector<string> trainSet, valSet, testSet;
    for(int i=0; i < allChunks.size(); i++)
    {
        if(i < firstThreshold)
            trainSet.push_back(allChunks[i]);
        if(firstThreshold <= i && i < secondThreshold)
            valSet.push_back(allChunks[i]);
        if(secondThreshold <= i)
            testSet.push_back(allChunks[i]);
    }
    ofstream trainStream(outputPath + "train.lst");
    for(const auto& chunk: trainSet)
        trainStream << chunk << endl;
    ofstream valStream(outputPath + "val.lst");
    for(const auto& chunk: valSet)
        valStream << chunk << endl;
    ofstream testStream(outputPath + "test.lst");
    for(const auto& chunk: testSet)
        testStream << chunk << endl;
}

int splitBimInImplicit(vector<facesLabelName> &labeledShapes, const vector<Point> &pointOfViews,
                       const vector<Point> &pointCloud, const vector<Vector> &pointCloudNormals,
                       int nbClasses, double bboxSize, int nbFilesToGenerate, int nbSurfacicPerFiles,
                       int nbVolumicPerFiles, const string &path, int nbBoxShoots, bool verbose)
{
    // Compute initial bounding box
    CGAL::Bbox_3 initialBbox;
    for(const auto& shape: labeledShapes)
        for(const auto& triangle: get<0>(shape))
            initialBbox += triangle.bbox();

    // Split it into dense bboxes
    vector<CGAL::Bbox_3> allBboxes = splitBigBbox(initialBbox, bboxSize);

    // Generate the chunks
    vector<string> allChunks;
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
        implicitRep.generateRandomVolumicPoints(labeledShapes, nbClasses, nbBoxShoots, verbose);

        // Normalize the points
        implicitRep.normalizeClouds();

        // Save current chunk
        string currentChunk = padTo(to_string(i), 8);
        allChunks.push_back(currentChunk);
        string outPath(path + currentChunk);
        implicitRep.save(outPath);
    }
    saveLists(allChunks, path);
    return allBboxes.size();
}
