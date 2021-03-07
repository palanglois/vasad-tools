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
    CGAL::Bbox_3 getSparseBbox(const CGAL::Bbox_3 &inDenseBbox, int nbSparseVoxelSide);

    // Getters
    [[nodiscard]] const CGAL::Bbox_3 sparseBbox() const;
private:
    double voxelSide;
    CGAL::Bbox_3 denseBbox;
    CGAL::Bbox_3 _sparseBbox;
    VoxelSparse sparseVoxels;
    VoxelArrangement denseVoxels;
    vector<Point> pointCloud;
};


#endif //BIM_DATA_SPARSETODENSE_H
