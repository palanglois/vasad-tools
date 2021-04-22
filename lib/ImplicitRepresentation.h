#ifndef BIM_DATA_IMPLICITREPRESENTATION_H
#define BIM_DATA_IMPLICITREPRESENTATION_H

// Ours
#include "graphStats.h"
#include "VoxelArrangement.h"

// STD
#include <experimental/filesystem>
#include <bitset>

// External
#include "cnpy/cnpy.h"


class ImplicitRepresentation {
public:
    // Constructor
    explicit ImplicitRepresentation(const CGAL::Bbox_3 &inBbox, int inNbFilesToGenerate, int inNbSurfacicPerFiles,
                                    int inNbVolumicPerFile);

    // Surfacic points
    void computeSurfacicFromPointCloud(const std::vector<Point> &pointCloud, const std::vector<Vector> &normals);

    // Volumic points
    void computeVolumicPoints(std::vector<facesLabelName> &labeledShapes, int nbClasses,
                              const std::vector<Point> &sampledPoints, bool verbose);

    void generateRandomVolumicPoints(std::vector<facesLabelName> &labeledShapes, int nbClasses, bool verbose);

    // Save current chunk
    void save(const std::string& path) const;

    // Getters
    [[nodiscard]] const std::vector<std::vector<double>> &getSurfacicPoints() const;

    [[nodiscard]] const std::vector<std::vector<double>> &getSurfacicNormals() const;

    [[nodiscard]] const std::vector<std::vector<double>> &getVolumicPoints() const;

    [[nodiscard]] const std::vector<int> &getOccupancies() const;

private:
    // Bounding box
    CGAL::Bbox_3 bbox;

    // Number of files to generate
    int nbFilesToGenerate;

    // Number of points per file
    int nbSurfacicPerFiles;
    int nbVolumicPerFiles;

    // Surface data
    std::vector<std::vector<double>> surfacicPoints;
    std::vector<std::vector<double>> surfacicNormals;

    // Volumic data
    std::vector<std::vector<double>> volumicPoints;
    std::vector<int> occupancies;

};

std::vector<Point> sampleInBbox(const CGAL::Bbox_3 &bbox, int nbSamples);

int splitBimInImplicit(std::vector<facesLabelName> &labeledShapes, const std::vector<Point> &pointOfViews,
                       const std::vector<Point> &pointCloud, const std::vector<Vector> &pointCloudNormals,
                       int nbClasses, double bboxSize, int nbFilesToGenerate, int nbSurfacicPerFiles,
                       int nbVolumicPerFiles, const std::string &path, bool verbose);


#endif //BIM_DATA_IMPLICITREPRESENTATION_H
