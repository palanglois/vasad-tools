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
                                    int inNbVolumicPerFile, int inNbClasses);

    // Surfacic points
    void computeSurfacicFromPointCloud(const std::vector<Point> &pointCloud, const std::vector<Vector> &normals,
                                       const std::vector<std::vector<double>> &richFeatures = std::vector<std::vector<double>>(0));

    // Volumic boxes
    void computeBoxes(std::vector<facesLabelName> &labeledShapes, const std::vector<Point> &sampledPoints,
            int nbShoots);

    // Volumic points
    void storeVolumicPoints(const std::vector<Point> &sampledPoints, const std::vector<int> &labels, int nbClasses);
    void computeVolumicPoints(std::vector<facesLabelName> &labeledShapes,
                              const std::vector<Point> &sampledPoints, bool verbose);

    void generateRandomVolumicPoints(std::vector<facesLabelName> &labeledShapes,  int nbBoxShoots=-1,
            bool verbose=false);

    // Normalization
    void normalizeClouds();

    // Save current chunk
    void save(const std::string& path) const;

    // Getters
    [[nodiscard]] const std::vector<std::vector<double>> &getSurfacicPoints() const;

    [[nodiscard]] const std::vector<std::vector<double>> &getSurfacicNormals() const;

    [[nodiscard]] const std::vector<std::vector<double>> &getVolumicPoints() const;

    [[nodiscard]] const std::vector<int> &getOccupancies() const;

    [[nodiscard]] const std::vector<std::vector<double>> &getBoxes() const;

private:
    // Bounding box
    CGAL::Bbox_3 bbox;

    // Number of files to generate
    int nbFilesToGenerate;
    int nbClasses;

    // Number of points per file
    int nbSurfacicPerFiles;
    int nbVolumicPerFiles;

    // Surface data
    std::vector<std::vector<double>> surfacicPoints;
    std::vector<std::vector<double>> surfacicNormals;
    std::vector<double> surfacicLabels;

    // Volumic data
    std::vector<std::vector<double>> volumicPoints;
    std::vector<int> occupancies;
    std::vector<std::vector<double>> boxes;
    bool areVolumicDataComputed;

};

std::vector<Point> sampleInBbox(const CGAL::Bbox_3 &bbox, int nbSamples);

int splitBimInImplicit(std::vector<facesLabelName> &labeledShapes, const std::vector<Point> &pointOfViews,
                       const std::vector<Point> &pointCloud, const std::vector<Vector> &pointCloudNormals,
                       const std::vector<std::vector<double>> &richFeatures,
                       int nbClasses, double bboxSize, int nbFilesToGenerate, int nbSurfacicPerFiles,
                       int nbVolumicPerFiles, const std::string &path, int nbBoxShoots=-1, int randomChunks=-1,
                       bool verbose=false);


#endif //BIM_DATA_IMPLICITREPRESENTATION_H
