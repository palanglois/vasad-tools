#include "RegionGrowing.h"

using namespace std;
using Json = nlohmann::json;

vector<string> split(const string &str, const string &delim) {
    vector<string> tokens;
    size_t prev = 0, pos = 0;
    do {
        pos = str.find(delim, prev);
        if (pos == string::npos) pos = str.length();
        string token = str.substr(prev, pos - prev);
        tokens.push_back(token);
        prev = pos + delim.length();
    } while (pos < str.length() && prev < str.length());
    return tokens;
}

/**
 * Computes the color gradiant
 * color: the output vector
 * x: the gradiant (beetween 0 and 360)
 * min and max: variation of the RGB channels (Move3D 0 -> 1)
 */
void GroundColorMix(double *color, double x, double min, double max) {
    /*
     * Red = 0
     * Green = 1
     * Blue = 2
     */
    double posSlope = (max - min) / 60;
    double negSlope = (min - max) / 60;

    if (x < 60) {
        color[0] = max;
        color[1] = posSlope * x + min;
        color[2] = min;
        return;
    } else if (x < 120) {
        color[0] = negSlope * x + 2 * max + min;
        color[1] = max;
        color[2] = min;
        return;
    } else if (x < 180) {
        color[0] = min;
        color[1] = max;
        color[2] = posSlope * x - 2 * max + min;
        return;
    } else if (x < 240) {
        color[0] = min;
        color[1] = negSlope * x + 4 * max + min;
        color[2] = max;
        return;
    } else if (x < 300) {
        color[0] = posSlope * x - 4 * max + min;
        color[1] = min;
        color[2] = max;
        return;
    } else {
        color[0] = max;
        color[1] = min;
        color[2] = negSlope * x + 6 * max;
        return;
    }
}

RegionGrowing::RegionGrowing(const string &inFile, double _epsilonPoint, double _epsilonNormal, double _sigmaPoint,
                             double _sigmaNormal, bool _verbose) : epsilonPoint(_epsilonPoint),
                                                                   epsilonNormal(_epsilonNormal),
                                                                   sigmaPoint(_sigmaPoint), sigmaNormal(_sigmaNormal),
                                                                   verbose(_verbose) {
    loadObjFile(inFile);
    buildAdjacencyGraph();
}

void RegionGrowing::loadObjFile(const string &inFile) {

    if (verbose)
        cout << "Loading obj file: " << inFile << endl;
    // Open the file
    ifstream fileCount(inFile.c_str());
    int nbVertice = 0;
    int nbNormals = 0;
    int nbFaces = 0;
    for (string line; getline(fileCount, line);) {
        if (line[0] == 'v' && line[1] == ' ')
            nbVertice++;
        else if (line[0] == 'v' && line[1] == 'n')
            nbNormals++;
        else if (line[0] == 'f' && line[1] == ' ')
            nbFaces++;
    }
    fileCount.close();

    // Filling the containers
    pointCloud.resize(nbVertice, 3);
    Normals rawNormals;
    rawNormals.resize(nbNormals, 3);
    faces.resize(nbFaces, 3);
    Faces faceNormalCoord;
    faceNormalCoord.resize(nbFaces, 3);
    faceNormals = Normals::Zero(nbFaces, 3);
    int pointIterator = 0;
    int normalIterator = 0;
    int faceIterator = 0;
    ifstream fileFill(inFile.c_str());
    for (string line; getline(fileFill, line);) {
        if (line[0] == 'v' && line[1] == ' ') {
            //Vertice
            istringstream iss(line);
            vector<string> itemsInString{istream_iterator<string>{iss},
                                         istream_iterator<string>{}};
            pointCloud(pointIterator, 0) = stod(itemsInString[1]);
            pointCloud(pointIterator, 1) = stod(itemsInString[2]);
            pointCloud(pointIterator, 2) = stod(itemsInString[3]);
            pointIterator++;
        } else if (line[0] == 'v' && line[1] == 'n') {
            //Normal
            istringstream iss(line);
            vector<string> itemsInString{istream_iterator<string>{iss},
                                         istream_iterator<string>{}};
            rawNormals(normalIterator, 0) = stod(itemsInString[1]);
            rawNormals(normalIterator, 1) = stod(itemsInString[2]);
            rawNormals(normalIterator, 2) = stod(itemsInString[3]);
            normalIterator++;
        } else if (line[0] == 'f' && line[1] == ' ') {
            //Face
            istringstream iss(line);
            vector<string> itemsInString{istream_iterator<string>{iss},
                                         istream_iterator<string>{}};
            assert(itemsInString.size() == 4);
            vector<string> elemsOne = split(itemsInString[1], "/");
            vector<string> elemsTwo = split(itemsInString[2], "/");
            vector<string> elemsThree = split(itemsInString[3], "/");
            assert(elemsOne.size() == 3);
            assert(elemsTwo.size() == 3);
            assert(elemsThree.size() == 3);
            faces(faceIterator, 0) = stoi(elemsOne[0]) - 1;
            faces(faceIterator, 1) = stoi(elemsTwo[0]) - 1;
            faces(faceIterator, 2) = stoi(elemsThree[0]) - 1;
            faceNormalCoord(faceIterator, 0) = stoi(elemsOne[2]) - 1;
            faceNormalCoord(faceIterator, 1) = stoi(elemsTwo[2]) - 1;
            faceNormalCoord(faceIterator, 2) = stoi(elemsThree[2]) - 1;
            faceIterator++;
        }
    }
    // Update face normals, areas and centers
    faceCenters.resize(nbFaces, 3);
    for (int i = 0; i < nbFaces; i++) {
        for (int j = 0; j < 3; j++)
            faceNormals.row(i) += rawNormals.row(faceNormalCoord(i, j));
        faceNormals.row(i).normalize();
        faceAreas.push_back(0.5 * (pointCloud.row(faces(i, 2)) - pointCloud.row(faces(i, 0))).cross(
                pointCloud.row(faces(i, 1)) - pointCloud.row(faces(i, 0))).norm());
        faceCenters.row(i) =
                (pointCloud.row(faces(i, 0)) + pointCloud.row(faces(i, 1)) + pointCloud.row(faces(i, 2))) / 3.;
    }
    if (verbose)
        cout << "Obj file loaded!" << endl;
}

void RegionGrowing::buildAdjacencyGraph() {
    if (verbose)
        cout << "Building adjacency graph..." << endl;
    triangleToEdge = vector<vector<Edge>>(faces.rows(), vector<Edge>(0));
    for (int i = 0; i < faces.rows(); i++) {
        vector<Edge> currentEdges;
        for (int j = 0; j < 3; j++) {
            // Build the 2 possible versions of the current edge
            Edge curEdge1, curEdge2, correctEdge;
            curEdge1.row(0) = pointCloud.row(faces(i, j % 3));
            curEdge1.row(1) = pointCloud.row(faces(i, (j + 1) % 3));
            curEdge2.row(1) = pointCloud.row(faces(i, j % 3));
            curEdge2.row(0) = pointCloud.row(faces(i, (j + 1) % 3));

            // Check if one version already exists. Otherwise, insert versionOne
            if (edgeToTriangle.find(curEdge1) != edgeToTriangle.end()) {
                //Version one is already there
                edgeToTriangle.at(curEdge1).push_back(i);
                correctEdge = curEdge1;
            } else if (edgeToTriangle.find(curEdge2) != edgeToTriangle.end()) {
                //Version two is already there
                edgeToTriangle.at(curEdge2).push_back(i);
                correctEdge = curEdge2;
            } else {
                // We insert version one
                edgeToTriangle[curEdge1] = {i};
                correctEdge = curEdge1;
            }
            currentEdges.push_back(correctEdge);
        }
        triangleToEdge[i] = currentEdges;
    }
    if (verbose)
        cout << "Adjacency graph built!" << endl;
}

int RegionGrowing::run() {
    // Main algorithm function
    if (verbose)
        cout << "Performing region growing" << endl;
    vector<PlaneClass> grownClasses(0);
    Eigen::Matrix<bool, Eigen::Dynamic, 1> isTriangleAdded;
    isTriangleAdded.resize(faces.rows(), 1);
    isTriangleAdded.setZero();
    vector<int> permutation = computeTriangleOrder();
    int globalIt = 0;
    while (!isTriangleAdded.all()) {
        if ((globalIt % (1 + int(permutation.size() / 10)) == 0) && verbose)
            cout << "Progress: " << round(100. * double(globalIt) / permutation.size()) << "%" << endl;
        // We check that the current triangle has not been visited yet
        int triangleIt = permutation[globalIt];
        if (isTriangleAdded(triangleIt)) {
            globalIt++;
            continue;
        }
        isTriangleAdded(triangleIt) = true;

        // We initiate a new class with the current unseen triangle
        PlaneClass newClass;
        newClass.addTriangle(faceAreas[triangleIt], triangleIt, faceNormals.row(triangleIt),
                             faceCenters.row(triangleIt));
        // We gather the none visited neighbours of current triangle
        vector<bool> visitedNeighbor(faces.rows(), false);
        visitedNeighbor[triangleIt] = true;
        deque<int> neighboursList;
        for (const auto &edge: triangleToEdge.at(triangleIt))
            for (const auto &neighbourTriIdx: edgeToTriangle.at(edge))
                //Check that neighbour is not the current triangle and it has not been added already
                if (neighbourTriIdx != triangleIt && !isTriangleAdded(neighbourTriIdx)) {
                    neighboursList.push_back(neighbourTriIdx);
                    visitedNeighbor[neighbourTriIdx] = true;
                }
        // We propagate to the neighbours
        while (!neighboursList.empty()) {
            const int &nextTriangleId = neighboursList.back();
            neighboursList.pop_back();
            const PointRg &nextTriangleAverage = faceCenters.row(nextTriangleId);
            if (isTriangleInClass(nextTriangleAverage, faceNormals.row(nextTriangleId), newClass)) {
                newClass.addTriangle(faceAreas[nextTriangleId], nextTriangleId, faceNormals.row(nextTriangleId),
                                     nextTriangleAverage);
                // When we get an inlier, we also test its neighbors so that the region grows
                for (const auto &edge: triangleToEdge.at(nextTriangleId)) {
                    for (const auto &neighbourTriIdx: edgeToTriangle.at(edge)) {

                        if (neighbourTriIdx != nextTriangleId && !isTriangleAdded(neighbourTriIdx) &&
                            !visitedNeighbor[neighbourTriIdx]) {
                            neighboursList.push_back(neighbourTriIdx);
                            visitedNeighbor[neighbourTriIdx] = true;
                        }
                    }
                }
            }
        }
        grownClasses.push_back(newClass);
    }

    // Primitive merging
    if (verbose)
        cout << "Merging the primitives" << endl;
    sort(grownClasses.begin(), grownClasses.end(),
         [&](const PlaneClass &a, const PlaneClass &b) -> bool { return a.getTotalArea() > b.getTotalArea(); });
    for (int i = 0; i < grownClasses.size(); i++) {
        if ((i % (1 + int(grownClasses.size() / 10)) == 0) && verbose)
            cout << "Progress: " << round(100. * double(i) / grownClasses.size()) << "%" << endl;
        bool classFound = false;
        for (auto &curClass: computedClasses)
            if (areClassesSame(curClass, grownClasses[i])) {
                classFound = true;
                curClass.mergeWith(grownClasses[i]);
                break;
            }
        if (!classFound)
            computedClasses.push_back(grownClasses[i]);
    }
    if (verbose)
        cout << "Algorithm completed! Found " << computedClasses.size() << " primitives." << endl;
    return computedClasses.size();
}

vector<int> RegionGrowing::computeTriangleOrder() const {
    vector<int> idx(faces.rows(), 0);
    vector<double> areas(faces.rows(), 0.);
    for (int i = 0; i < faces.rows(); i++) {
        auto vertexA = pointCloud.row(faces(i, 0));
        auto vertexB = pointCloud.row(faces(i, 1));
        auto vertexC = pointCloud.row(faces(i, 2));
        idx[i] = i;
        areas[i] = 0.5 * (vertexB - vertexA).cross(vertexC - vertexA).norm();
    }
    sort(idx.begin(), idx.end(), [&](const int &a, const int &b) -> bool { return areas[a] > areas[b]; });
    return idx;
}

const Faces &RegionGrowing::getFaces() const {
    return faces;
}

const PointCloud &RegionGrowing::getPointCloud() const {
    return pointCloud;
}

inline bool RegionGrowing::isTriangleInClass(const PointRg &averagePoint, const PointRg &triangleNormal,
                                             const PlaneClass &planeClass) const {
    const PointRg &classNormal = planeClass.getNormal();
    const PointRg &classAveragePoint = planeClass.getRefPoint();
//    bool pointContributionOne = abs((averagePoint - classAveragePoint).dot(classNormal)) < epsilonPoint;
//    bool pointContributionTwo = abs((averagePoint - classAveragePoint).dot(triangleNormal)) < epsilonPoint;
//    bool normalContribution = 1. - abs(triangleNormal.dot(classNormal)) < epsilonNormal;
//    return pointContributionOne && pointContributionTwo && normalContribution;
    if (abs((averagePoint - classAveragePoint).dot(classNormal)) >= epsilonPoint) return false;
    if (abs((averagePoint - classAveragePoint).dot(triangleNormal)) < epsilonPoint) return false;
    return (1. - abs(triangleNormal.dot(classNormal))) < epsilonNormal;
}

inline bool RegionGrowing::areClassesSame(const PlaneClass &planeClassOne, const PlaneClass &planeClassTwo) {
//    bool pointContributionOne = abs((planeClassOne.getRefPoint() - planeClassTwo.getRefPoint()).dot(planeClassOne.getNormal())) < sigmaPoint;
//    bool pointContributionTwo = abs((planeClassOne.getRefPoint() - planeClassTwo.getRefPoint()).dot(planeClassTwo.getNormal())) < sigmaPoint;
//    bool normalContribution = 1. - abs((planeClassOne.getNormal().dot(planeClassTwo.getNormal()))) < sigmaNormal;
//    return pointContributionOne && pointContributionTwo && normalContribution;
    if (abs((planeClassOne.getRefPoint() - planeClassTwo.getRefPoint()).dot(planeClassOne.getNormal())) >=
        sigmaPoint)
        return false;
    if (abs((planeClassOne.getRefPoint() - planeClassTwo.getRefPoint()).dot(planeClassTwo.getNormal())) >=
        sigmaPoint)
        return false;
    return (1. - abs((planeClassOne.getNormal().dot(planeClassTwo.getNormal())))) < sigmaNormal;
//    Point difRef = planeClassOne.getRefPoint() - planeClassTwo.getRefPoint();
//    if(abs(inner_product(difRef.data(), difRef.data() + difRef.size(), planeClassOne.getNormal().data(), 0.)) >= sigmaPoint) return false;
//    if(abs(inner_product(difRef.data(), difRef.data() + difRef.size(), planeClassTwo.getNormal().data(), 0.)) >= sigmaPoint) return false;
//    return (1. - abs((planeClassOne.getNormal().dot(planeClassTwo.getNormal())))) < sigmaNormal;
}

void RegionGrowing::saveAsObj(const string &outFile) const {
    stringstream outStream;
    int vertexId = 1;
    vector<int> permutation(computedClasses.size(), 0);
    for (int i = 0; i < permutation.size(); i++)
        permutation[i] = i;
    unsigned seed = chrono::system_clock::now().time_since_epoch().count();
    shuffle(permutation.begin(), permutation.end(), default_random_engine(seed));
    for (int j = 0; j < computedClasses.size(); j++) {
        const int &i = permutation[j];
        auto color = new double[3];
        double intensity = double(j) / computedClasses.size() * 360.;
        GroundColorMix(color, intensity, 0, 255);
        vector<int> intColor = {int(color[0]), int(color[1]), int(color[2])};
        for (const auto &faceIdx: computedClasses[i].getInlierIndex()) {
            for (int k = 0; k < 3; k++) {
                const PointRg &curPoint = pointCloud.row(faces(faceIdx, k));
                outStream << "v " << curPoint(0) << " "
                          << curPoint(1) << " "
                          << curPoint(2) << " "
                          << int(color[0]) << " "
                          << int(color[1]) << " "
                          << int(color[2]) << " " << endl;
            }
            outStream << "f " << vertexId << " " << vertexId + 1 << " " << vertexId + 2 << endl;
            vertexId += 3;
        }
        delete[] color;
    }
    ofstream outStreamReal(outFile.c_str());
    outStreamReal << outStream.rdbuf();
}
/***
 *
 * @param outFile Path to the output jsons file
 * @param fullPlaneData Adds information about the inlier triangles for each computed face as well as
 * the cumulated percentage of the covered area for each face
 */
void RegionGrowing::saveAsJson(const string &outFile, bool fullPlaneData) {
    // Sort computed classes by decreasing area order
    sort(computedClasses.begin(), computedClasses.end(),
         [&](const PlaneClass &a, const PlaneClass &b) -> bool { return a.getTotalArea() > b.getTotalArea(); });

    // Compute total area
    double totalArea = 0.;
    for(const auto& planeClass: computedClasses)
        totalArea += planeClass.getTotalArea();

    // Planes
    Json planesData;
    double areaAccumulator = 0.;
    for (const auto &planeClass: computedClasses) {
        Json planeNormal = {planeClass.getNormal()(0), planeClass.getNormal()(1), planeClass.getNormal()(2)};
        Json planeInlier = {planeClass.getRefPoint()(0), planeClass.getRefPoint()(1), planeClass.getRefPoint()(2)};
        Json planeData = {{"normal", planeNormal},
                          {"inlier", planeInlier}};

        if (fullPlaneData) {
            // Cumulated percentage of the covered area
            areaAccumulator += planeClass.getTotalArea();
            planeData["cumulatedPercentage"] = areaAccumulator / totalArea;

            // Faces that belong to current plane
            vector<vector<int>> allFaces(planeClass.getInlierIndex().size(), vector<int>(3));
            for (int i = 0; i < allFaces.size(); i++)
                allFaces[i] = {faces(planeClass.getInlierIndex()[i], 0), faces(planeClass.getInlierIndex()[i], 1), faces(planeClass.getInlierIndex()[i], 2)};
            planeData["faces"] = allFaces;
        }

        planesData.push_back(planeData);
    }

    // Map
    Json map;
    map["NOMAP"] = 0;

    // Labels
    Json labels;
    labels.push_back(true);

    // Bbox
    PointRg minPoint = pointCloud.colwise().minCoeff();
    PointRg maxPoint = pointCloud.colwise().maxCoeff();
    Json bbox = {minPoint(0), minPoint(1), minPoint(2), maxPoint(0), maxPoint(1), maxPoint(2)};

    // Number of planes
    Json nbPlanes = computedClasses.size();

    // Compile data
    Json outputData = {{"planes",   planesData},
                       {"map",      map},
                       {"labels",   labels},
                       {"bbox",     bbox},
                       {"nbPlanes", nbPlanes}};

    if (fullPlaneData) {
        vector<vector<double>> points(pointCloud.rows(), vector<double>(3));
        for(int i=0; i < pointCloud.rows(); i++)
            points[i] = {pointCloud(i, 0), pointCloud(i, 1), pointCloud(i, 2)};
        outputData["pointCloud"] = points;
    }

    ofstream outStream(outFile.c_str());
    outStream << outputData;
}

PlaneClass::PlaneClass() {
    normalAccumulator.setZero();
    averagePointAccumulator.setZero();
    normal.setZero();
    averagePoint.setZero();
    triangleAreaAccumulator = 0;
}

void PlaneClass::addTriangle(const double &triangleArea, int index, const PointRg &curNormal, const PointRg &average) {

    // Updating the index
    inlierIndex.push_back(index);

    // Check the normal orientation and update
    if (normal.dot(curNormal) < 0)
        normal -= curNormal * triangleArea;
    else
        normal += curNormal * triangleArea;

    // Update the rest of the attributes
    averagePointAccumulator += average * triangleArea;
    triangleAreaAccumulator += triangleArea;

    // Update the class characteristics
    updateNormal();
    updateRefPoint();
}

const PointRg &PlaneClass::getNormal() const {
    return normal;
}

const PointRg &PlaneClass::getRefPoint() const {
    return averagePoint;
}

void PlaneClass::updateNormal() {
    normal /= normal.norm();
}

void PlaneClass::updateRefPoint() {
    averagePoint = averagePointAccumulator / triangleAreaAccumulator;
}

double PlaneClass::getArea() const {
    return triangleAreaAccumulator;
}

void PlaneClass::mergeWith(const PlaneClass &other) {
    // Concatenate points and their indexes
    inlierIndex.insert(inlierIndex.end(), other.inlierIndex.begin(), other.inlierIndex.end());

    // Check the normal orientation and update
    if (normal.dot(other.normal) < 0)
        normal -= other.normal;
    else
        normal += other.normal;

    // Update the rest of the attributes
    averagePointAccumulator += other.averagePointAccumulator;
    triangleAreaAccumulator += other.triangleAreaAccumulator;

    // Update the class characteristics
    updateNormal();
    updateRefPoint();
}

const double &PlaneClass::getTotalArea() const {
    return triangleAreaAccumulator;
}

const vector<int> &PlaneClass::getInlierIndex() const {
    return inlierIndex;
}
