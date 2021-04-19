#include "sparseToDense.h"

using namespace std;

SparseToDense::SparseToDense(const CGAL::Bbox_3 &inDenseBbox, double inVoxelSide, int nbSparseVoxelSide) :
        voxelSide(inVoxelSide), denseBbox(inDenseBbox), _sparseBbox(getSparseBbox(inDenseBbox, nbSparseVoxelSide)),
        sparseVoxels(VoxelSparse(_sparseBbox, inVoxelSide)), denseVoxels(inDenseBbox, inVoxelSide)
{

}

CGAL::Bbox_3 SparseToDense::getSparseBbox(const CGAL::Bbox_3 &inDenseBbox, int nbSparseVoxelSide) const {
    double centerX = (inDenseBbox.xmax() + inDenseBbox.xmin()) / 2.;
    double centerY = (inDenseBbox.ymax() + inDenseBbox.ymin()) / 2.;
    double centerZ = (inDenseBbox.zmax() + inDenseBbox.zmin()) / 2.;
    double sparseBboxSize = voxelSide * nbSparseVoxelSide;
    return CGAL::Bbox_3(centerX - sparseBboxSize / 2., centerY - sparseBboxSize / 2., centerZ - sparseBboxSize / 2.,
                        centerX + sparseBboxSize / 2., centerY + sparseBboxSize / 2., centerZ + sparseBboxSize / 2.);
}

const CGAL::Bbox_3 &SparseToDense::sparseBbox() const {
    return _sparseBbox;
}

void SparseToDense::computeFeaturesFromPointCloud(const vector<Point> &pointCloud, const vector<Vector> &normals) {
    sparseVoxels.computeFeaturesFromPointCloud(pointCloud, normals);
}

void SparseToDense::computeVisibility(const vector<Point> &points, const vector<Point> &pointOfViews, bool verbose) {
    denseVoxels.computeVisibility(points, pointOfViews, verbose);
}

void SparseToDense::computeRichFeatures(vector<facesLabelName> &labeledShapes, int nbClasses, bool verbose) {
    denseVoxels.computeRichFeatures(labeledShapes, nbClasses, verbose);
}

void SparseToDense::saveAsHdf(const string &path) {

    // Create the hdf5 file
    H5Easy::File file(path, H5Easy::File::Overwrite);
    auto options = H5Easy::DumpOptions(H5Easy::Compression(), H5Easy::DumpMode::Overwrite);

    // Planes
    const vector<PlaneSimple> &planes = denseVoxels.planes();
    if(!planes.empty()) {
        vector<vector<double>> inliers(planes.size(), vector<double>(3, 0.));
        vector<vector<double>> normals(planes.size(), vector<double>(3, 0.));
        vector<vector<vector<int>>> faces;
        vector<double> cumulatedPercentages;
        int planeIt = 0;
        for (const auto &pl: planes) {
            Point inlier = pl.inlier;
            Vector normal = pl.normal;
            inliers[planeIt] = {inlier.x(), inlier.y(), inlier.z()};
            normals[planeIt] = {normal.x(), normal.y(), normal.z()};
            faces.push_back(pl.faces);
            cumulatedPercentages.push_back(pl.cumulatedPercentage);
            planeIt++;
        }
        H5Easy::dump(file, "/planes/inliers", inliers, options);
        H5Easy::dump(file, "/planes/normals", normals, options);
        H5Easy::dump(file, "/planes/faces", faces, options);
        H5Easy::dump(file, "/planes/cumulatedPercentages", cumulatedPercentages, options);
    }

    // NbPlanes
    H5Easy::dump(file, "/nbPlanes", planes.size(), options);

    // Features
    H5Easy::dump(file, "/sparseFeatures/coordinates", sparseVoxels.coord(), options);
    H5Easy::dump(file, "/sparseFeatures/values", sparseVoxels.normalizedValues(), options);

    // Labels (rich features)
    H5Easy::dump(file, "/richFeatures", denseVoxels.richFeatures(), options);

    // Visibility
    H5Easy::dump(file, "/visibility", denseVoxels.visibility(), options);

    // Dense bbox
    vector<double> denseBboxVec = {denseBbox.xmin(), denseBbox.ymin(), denseBbox.zmin(),
                                   denseBbox.xmax(), denseBbox.ymax(), denseBbox.zmax()};
    H5Easy::dump(file, "/bbox", denseBboxVec, options);

    // Sparse bbox
    vector<double> sparseBboxVec = {_sparseBbox.xmin(), _sparseBbox.ymin(), _sparseBbox.zmin(),
                                    _sparseBbox.xmax(), _sparseBbox.ymax(), _sparseBbox.zmax()};
    H5Easy::dump(file, "/sparseBbox", sparseBboxVec, options);

    // Voxel side size
    H5Easy::dump(file, "/voxelSide", voxelSide, options);

    // Point cloud
    vector<vector<double>> pointCloudArray(sparseVoxels.pointCloud().size(), vector<double>(3, 0.));
    for(int i=0; i < sparseVoxels.pointCloud().size(); i++) {
        pointCloudArray[i][0] = sparseVoxels.pointCloud()[i].x();
        pointCloudArray[i][1] = sparseVoxels.pointCloud()[i].y();
        pointCloudArray[i][2] = sparseVoxels.pointCloud()[i].z();
    }
    H5Easy::dump(file, "/pointCloud", pointCloudArray, options);

}

bool SparseToDense::areSparseFeaturesEmpty() const {
    return sparseVoxels.coord().empty();
}

bool SparseToDense::areRichFeaturesEmpty() const {
    return denseVoxels.areRichFeaturesEmpty();
}

int splitArrangementInVoxelsDts(vector<facesLabelName> &labeledShapes,
                                const vector<Point> &pointOfViews,
                                const vector<Point> &pointCloud,
                                const vector<Vector> &pointCloudNormals,
                                double voxelSide, int nbClasses,
                                const string &path, int nbDenseVoxelsAlongAxis,
                                int nbSparseVoxelsAlongAxis, bool verbose) {
    // Compute initial bounding box
    CGAL::Bbox_3 initialBbox;
    for(const auto& shape: labeledShapes)
        for(const auto& triangle: get<0>(shape))
            initialBbox += triangle.bbox();

    // Split it into dense bboxes
    vector<CGAL::Bbox_3> denseBboxes = splitBigBbox(initialBbox, nbDenseVoxelsAlongAxis * voxelSide);

    // Generate the chunks
    for(int i=0; i < denseBboxes.size(); i++) {
        const CGAL::Bbox_3 &curBbox = denseBboxes[i];
        if (verbose) {
            cout << endl << "Bbox \033[1;31m" << i << "\033[0m out of " << denseBboxes.size() << endl;
            cout << "Current bbox: " << curBbox << endl;
        }

        // We build the chunk corresponding to the current dense bounding box
        auto stdStructure = SparseToDense(curBbox, voxelSide, nbSparseVoxelsAlongAxis);

        // We compute the features, visibility and labels (rich features)
        stdStructure.computeFeaturesFromPointCloud(pointCloud, pointCloudNormals);
        if(stdStructure.areSparseFeaturesEmpty()) continue;
        stdStructure.computeVisibility(pointCloud, pointOfViews, verbose);
        stdStructure.computeRichFeatures(labeledShapes, nbClasses, verbose);
        if(stdStructure.areRichFeaturesEmpty()) continue;

        // We save the current chunk
        string outPath(path + padTo(to_string(i), 5) + ".h5");
        stdStructure.saveAsHdf(outPath);
    }
    return denseBboxes.size();
}
