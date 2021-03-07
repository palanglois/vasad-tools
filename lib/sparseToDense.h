#ifndef BIM_DATA_SPARSETODENSE_H
#define BIM_DATA_SPARSETODENSE_H

#include "VoxelArrangementInline.h"
#include "VoxelSparse.h"

class SparseToDense
{
public:
    // Constructor
    SparseToDense(const CGAL::Bbox_3 &inDenseBbox, double inVoxelSide, int nbSparseVoxelSide);

    // Internal functions
    [[nodiscard]] CGAL::Bbox_3 getSparseBbox(const CGAL::Bbox_3 &inDenseBbox, int nbSparseVoxelSide) const;

    // Features, visibility, labels (rich features)
    void computeFeaturesFromPointCloud(const std::vector<Point> &pointCloud, const std::vector<Vector> &normals);
    void computeVisibility(const std::vector<Point> &points, const std::vector<Point> &pointOfViews, bool verbose);
    void computeRichFeatures(std::vector<facesLabelName> &labeledShapes, int nbClasses, bool verbose);

    // Save as hdf5
    void saveAsHdf(const std::string &path);

    // Getters
    [[nodiscard]] const CGAL::Bbox_3 &sparseBbox() const;
    bool areSparseFeaturesEmpty() const;
private:
    double voxelSide;
    CGAL::Bbox_3 denseBbox;
    CGAL::Bbox_3 _sparseBbox;
    VoxelSparse sparseVoxels;
    VoxelArrangement denseVoxels;
};

int splitArrangementInVoxelsDts(std::vector<facesLabelName> &labeledShapes,
                                const std::vector<Point> &pointOfViews,
                                const std::vector<Point> &pointCloud,
                                const std::vector<Vector> &pointCloudNormals,
                                double voxelSide, int nbClasses,
                                const std::string &path, int nbDenseVoxelsAlongAxis,
                                int nbSparseVoxelsAlongAxis, bool verbose);


#endif //BIM_DATA_SPARSETODENSE_H
