#include "RegionGrowing.h"

using namespace std;

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

RegionGrowing::RegionGrowing(const string &inFile, double _epsilonPoint, double _epsilonNormal, double _sigmaPoint,
                             double _sigmaNormal, bool _verbose) : epsilonPoint(_epsilonPoint),
                             epsilonNormal(_epsilonNormal), sigmaPoint(_sigmaPoint), sigmaNormal(_sigmaNormal),
                             verbose(_verbose){
    loadObjFile(inFile);
    buildAdjacencyGraph();
}

void RegionGrowing::loadObjFile(const string &inFile) {

    if(verbose)
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
    // Update face normals
    for (int i = 0; i < nbFaces; i++) {
        for (int j = 0; j < 3; j++)
            faceNormals.row(i) += rawNormals.row(faceNormalCoord(i, j));
        faceNormals.row(i).normalize();
    }
    if(verbose)
        cout << "Obj file loaded!" << endl;
}

void RegionGrowing::buildAdjacencyGraph() {
    if(verbose)
        cout << "Building adjacency graph..." << endl;
    triangleToEdge = vector<vector<Edge>>(faces.rows(), vector<Edge>(0));
    for(int i=0; i < faces.rows(); i++)
    {
        vector<Edge> currentEdges;
        for(int j=0; j < 3; j++)
        {
            // Build the 2 possible versions of the current edge
            Edge curEdge1, curEdge2, correctEdge;
            curEdge1.row(0) = pointCloud.row(faces(i, j % 3));
            curEdge1.row(1) = pointCloud.row(faces(i, (j + 1) % 3));
            curEdge2.row(1) = pointCloud.row(faces(i, j % 3));
            curEdge2.row(0) = pointCloud.row(faces(i, (j + 1) % 3));

            // Check if one version already exists. Otherwise, insert versionOne
            if(edgeToTriangle.find(curEdge1) != edgeToTriangle.end())
            {
                //Version one is already there
                edgeToTriangle.at(curEdge1).push_back(i);
                correctEdge = curEdge1;
            }
            else if(edgeToTriangle.find(curEdge2) != edgeToTriangle.end())
            {
                //Version two is already there
                edgeToTriangle.at(curEdge2).push_back(i);
                correctEdge = curEdge2;
            }
            else
            {
                // We insert version one
                edgeToTriangle[curEdge1] = {i};
                correctEdge = curEdge1;
            }
            currentEdges.push_back(correctEdge);
        }
        triangleToEdge[i] = currentEdges;
    }
    if(verbose)
        cout << "Adjacency graph built!" << endl;
}

void RegionGrowing::run() {
    // Main algorithm function
    vector<PlaneClass> grownClasses(0);
    Eigen::Matrix<bool, Eigen::Dynamic, 1> isTriangleAdded;
    isTriangleAdded.resize(faces.rows(), 1);
    isTriangleAdded.setZero();
    vector<int> permutation = computeTriangleOrder();
    int globalIt = 0;
    while(!isTriangleAdded.all())
    {
        // We check that the current triangle has not been visited yet
        int triangleIt = permutation[globalIt];
        if(isTriangleAdded(triangleIt))
        {
            globalIt++;
            continue;
        }
        isTriangleAdded(triangleIt) = true;

        // We initiate a new class with the current unseen triangle
        PlaneClass newClass;
        Triangle curTriangle = getTriangle(triangleIt);
        Point averagePoint = curTriangle.rowwise().mean();
        newClass.addTriangle(curTriangle, triangleIt, faceNormals.row(triangleIt), averagePoint);
        //TODO
    }

}

vector<int> RegionGrowing::computeTriangleOrder() const {
    vector<int> idx(faces.rows(), 0);
    vector<double> areas(faces.rows(), 0.);
    for(int i=0; i < faces.rows(); i++)
    {
        auto vertexA = pointCloud.row(faces(i, 0));
        auto vertexB = pointCloud.row(faces(i, 1));
        auto vertexC = pointCloud.row(faces(i, 2));
        idx[i] = i;
        areas[i] = 0.5*(vertexB - vertexA).cross(vertexC - vertexA).norm();
    }
    sort(idx.begin(), idx.end(), [&](const int& a, const int& b) -> bool {return areas[a] > areas[b];});
    return idx;
}

const Faces &RegionGrowing::getFaces() const {
    return faces;
}

const PointCloud &RegionGrowing::getPointCloud() const {
    return pointCloud;
}

Triangle RegionGrowing::getTriangle(int i) const {
    assert(i >= 0);
    assert(i < faces.size());
    Triangle triangle;
    for (int j = 0; j < 3; j++)
        triangle.row(j) = pointCloud.row(faces(i, j));
    return triangle;
}

PlaneClass::PlaneClass() {
    normalAccumulator.setZero();
    averagePointAccumulator.setZero();
    normal.setZero();
    averagePoint.setZero();
    triangleAreaAccumulator = 0;
}

void PlaneClass::addTriangle(const Triangle &triangle, int index, const Point &curNormal, const Point &average) {
    double triangleArea = 0.5 * (triangle.row(1) - triangle.row(0)).cross(triangle.row(2) - triangle.row(0)).norm();

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
    normal = getNormal();
    averagePoint = getRefPoint();

    // Insert the points;
    for (int i = 0; i < 3; i++)
        inlierPoints.emplace_back(triangle.row(i));
}

Point PlaneClass::getNormal() const {
    return normal / normal.norm();
}

Point PlaneClass::getRefPoint() const {
    return averagePointAccumulator / triangleAreaAccumulator;
}

double PlaneClass::getArea() const {
    return triangleAreaAccumulator;
}

void PlaneClass::mergeWith(const PlaneClass &other) {
    // Concatenate points and their indexes
    inlierIndex.insert(inlierIndex.end(), other.inlierIndex.begin(), other.inlierIndex.end());
    inlierPoints.insert(inlierPoints.end(), other.inlierPoints.begin(), other.inlierPoints.end());

    // Check the normal orientation and update
    if (normal.dot(other.normal) < 0)
        normal -= other.normal;
    else
        normal += other.normal;

    // Update the rest of the attributes
    averagePointAccumulator += other.averagePointAccumulator;
    triangleAreaAccumulator += other.triangleAreaAccumulator;

    // Update the class characteristics
    normal = getNormal();
    averagePoint = getRefPoint();
}
