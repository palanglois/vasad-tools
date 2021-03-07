#include "sparseToDense.h"

using namespace std;

SparseToDense::SparseToDense(const CGAL::Bbox_3 &inDenseBbox, double inVoxelSide, int nbSparseVoxelSide) :
        voxelSide(inVoxelSide), denseBbox(inDenseBbox), _sparseBbox(getSparseBbox(inDenseBbox, nbSparseVoxelSide)),
        sparseVoxels(VoxelSparse(_sparseBbox, inVoxelSide)), denseVoxels(inDenseBbox, inVoxelSide)
{

}

CGAL::Bbox_3 SparseToDense::getSparseBbox(const CGAL::Bbox_3 &inDenseBbox, int nbSparseVoxelSide) {
    double centerX = (inDenseBbox.xmax() + inDenseBbox.xmin()) / 2.;
    double centerY = (inDenseBbox.ymax() + inDenseBbox.ymin()) / 2.;
    double centerZ = (inDenseBbox.zmax() + inDenseBbox.zmin()) / 2.;
    double sparseBboxSize = voxelSide * nbSparseVoxelSide;
    return CGAL::Bbox_3(centerX - sparseBboxSize / 2., centerY - sparseBboxSize / 2., centerZ - sparseBboxSize / 2.,
                        centerX + sparseBboxSize / 2., centerY + sparseBboxSize / 2., centerZ + sparseBboxSize / 2.);
}

const CGAL::Bbox_3 SparseToDense::sparseBbox() const
{
    return _sparseBbox;
}