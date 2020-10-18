#ifndef BIM_DATA_REGIONGROWING_H
#define BIM_DATA_REGIONGROWING_H

#include <iostream>
#include <iterator>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Geometry>

typedef Eigen::Vector3d                        Point;      // A type for points
typedef Eigen::Matrix3d                        Triangle;   // A type for triangles
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
    void addTriangle(const Triangle& triangle, int index, const Point& curNormal, const Point& average);
    Point getNormal() const;
    Point getRefPoint() const;
    double getArea() const;
    void mergeWith(const PlaneClass& other);

private:
    std::vector<int> inlierIndex;
    std::vector<Point> inlierPoints;
    Point normalAccumulator;
    Point averagePointAccumulator;
    double triangleAreaAccumulator;
    Point normal;
    Point averagePoint;
};

class RegionGrowing
{
public:
    RegionGrowing(const std::string& inFile, double _epsilonPoint, double _epsilonNormal,
                  double _sigmaPoint, double _sigmaNormal, bool _verbose=false);
    void loadObjFile(const std::string& inFile);
    void buildAdjacencyGraph();
    std::vector<int> computeTriangleOrder() const;
    void run();

    const Faces& getFaces() const;
    const PointCloud& getPointCloud() const;
    Triangle getTriangle(int i) const;
private:
    // Mesh
    PointCloud pointCloud;
    Faces faces;
    Normals faceNormals;

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
