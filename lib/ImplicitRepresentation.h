#ifndef BIM_DATA_IMPLICITREPRESENTATION_H
#define BIM_DATA_IMPLICITREPRESENTATION_H

#include "graphStats.h"
#include "VoxelArrangement.h"


class ImplicitRepresentation {
public:
    // Constructor
    explicit ImplicitRepresentation(const CGAL::Bbox_3 &inBbox);

    // Surfacic points
    void computeSurfacicFromPointCloud(const std::vector<Point> &pointCloud, const std::vector<Vector> &normals);

    // Volumic points
    void computeVolumicPoints(std::vector<facesLabelName> &labeledShapes, int nbClasses,
                              const std::vector<Point> &sampledPoints, bool verbose);

    // Getters
    [[nodiscard]] const std::vector<std::vector<double>> &getSurfacicPoints() const;

    [[nodiscard]] const std::vector<std::vector<double>> &getSurfacicNormals() const;

    [[nodiscard]] const std::vector<std::vector<double>> &getVolumicPoints() const;

    [[nodiscard]] const std::vector<int> &getOccupancies() const;

private:
    // Bounding box
    CGAL::Bbox_3 bbox;

    // Surface data
    std::vector<std::vector<double>> surfacicPoints;
    std::vector<std::vector<double>> surfacicNormals;

    // Volumic data
    std::vector<std::vector<double>> volumicPoints;
    std::vector<int> occupancies;

};

std::vector<Point> sampleInBbox(const CGAL::Bbox_3 &bbox, int nbSamples);


#endif //BIM_DATA_IMPLICITREPRESENTATION_H
