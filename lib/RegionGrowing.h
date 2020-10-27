#ifndef BIM_DATA_REGIONGROWING_H
#define BIM_DATA_REGIONGROWING_H

// STD
#include <iostream>
#include <iterator>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <deque>
#include <random>
#include <chrono>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// Json
#include <json/json.hpp>

typedef Eigen::Vector3d                        PointRg;      // A type for points
typedef Eigen::Matrix<double, 2, 3>            Edge;       // A type for edges
typedef Eigen::Matrix<double,Eigen::Dynamic,3> PointCloud; // A type for the point clouds
typedef Eigen::Matrix<double,Eigen::Dynamic,3> Normals;    // A type for the normals
typedef Eigen::Matrix<int,Eigen::Dynamic,3>    Faces;      // A type for the faces

// Hash for Eigen Matrix
template<typename T>
struct matrix_hash : std::unary_function<T, size_t> {
    std::size_t operator()(T const& matrix) const {
        // Note that it is oblivious to the storage order of Eigen matrix (column- or
        // row-major). It will give you the same hash value for two different matrices if they
        // are the transpose of each other in different storage order.
        size_t seed = 0;
        for (size_t i = 0; i < matrix.size(); ++i) {
            auto elem = *(matrix.data() + i);
            seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

class PlaneClass
{
public:
    PlaneClass();

    // Main functions
    void addTriangle(const double &triangleArea, int index, const PointRg& curNormal, const PointRg& average);
    void mergeWith(const PlaneClass& other);

    // Accessors
    const PointRg& getNormal() const;
    const PointRg& getRefPoint() const;
    const double& getTotalArea() const;
    double getArea() const;
    const std::vector<int>& getInlierIndex() const;

protected:
    // Internal functions
    void updateNormal();
    void updateRefPoint();

private:
    std::vector<int> inlierIndex;
    PointRg normalAccumulator;
    PointRg averagePointAccumulator;
    double triangleAreaAccumulator;
    PointRg normal;
    PointRg averagePoint;
};

class RegionGrowing
{
public:
    RegionGrowing(const std::string& inFile, double _epsilonPoint, double _epsilonNormal,
                  double _sigmaPoint, double _sigmaNormal, bool _verbose=false);
    // Main functions
    int run();
    void saveAsObj(const std::string& outFile) const;
    void saveAsJson(const std::string& outFile, bool fullPlaneData=false);

    // Region growing criteria
    inline bool isTriangleInClass(const PointRg &averagePoint, const PointRg &triangleNormal, const PlaneClass &planeClass) const;
    inline bool areClassesSame(const PlaneClass &planeClassOne, const PlaneClass &planeClassTwo);

    // Internal functions
    void loadObjFile(const std::string& inFile);
    void buildAdjacencyGraph();
    std::vector<int> computeTriangleOrder() const;

    // Accessors
    const Faces& getFaces() const;
    const PointCloud& getPointCloud() const;
private:
    // Mesh
    PointCloud pointCloud;
    Faces faces;
    Normals faceNormals;
    std::vector<double> faceAreas;
    PointCloud faceCenters;

    // Algorithm attributes
    std::vector<PlaneClass> computedClasses;
    std::vector<std::vector<Edge>> triangleToEdge;
    std::unordered_map<Edge, std::vector<int>, matrix_hash<Edge>> edgeToTriangle;

    // Parameters
    const double epsilonPoint;
    const double epsilonNormal;
    const double sigmaPoint;
    const double sigmaNormal;

    //Verbosity
    const bool verbose;
};

#endif //BIM_DATA_REGIONGROWING_H
